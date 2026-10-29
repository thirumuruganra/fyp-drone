"""
Phase 4: upload a planned mission to ArduCopter SITL (Gazebo JSON backend), fly it
in AUTO, and log MAVLink telemetry + Gazebo ground truth.

    # terminal 1: gazebo/launch_region.sh thiru_demo missions/<run_id>
    # terminal 2: sim_vehicle.py ... --custom-location=...   (printed by the launcher)
    # terminal 3:
    python sitl_mission_runner.py missions/<run_id>
    python sitl_mission_runner.py missions/<run_id> --url udpin:127.0.0.1:14551 --no-fly   # upload + verify only

Checks before flight: mission ↔ world bundle hash, complete front, weather gate,
SITL home == depot pad (2 m / 1 m), mission accepted and read back identical.
Writes Simulation/runs/<run_id>__<time>/ (telemetry CSVs, gz_pose.csv,
params.json, run_manifest.json, mission.json copy) for evaluate_gazebo_run.py.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")

import _paths
import sim_config as scfg
from sim_mission_export import load_mission
from simulation_logger import GazeboPoseLogger, MavlinkLogger

# Waypoint speed: ArduCopter ≤ 4.6 WPNAV_SPEED (cm/s); 4.7+ WP_SPD (m/s).  The mission's
# DO_CHANGE_SPEED item sets the same speed; the parameter covers the take-off leg.
SPEED_PARAMS = (("WP_SPD", scfg.SIM_CRUISE_SPEED_MS), ("WPNAV_SPEED", scfg.SIM_CRUISE_SPEED_MS * 100.0))
SIM_PARAMS = {
    "TERRAIN_ENABLE": 0,          # frame 3 missions; terrain-following (frame 10) deferred
    "FENCE_ENABLE": 0,
    "AUTO_OPTIONS": 3,            # bit0 allow arming in AUTO, bit1 take off without throttle
}
STREAM_HZ = {"GLOBAL_POSITION_INT": 10, "LOCAL_POSITION_NED": 10, "ATTITUDE": 10,
             "MISSION_CURRENT": 2, "EXTENDED_SYS_STATE": 2, "BATTERY_STATUS": 1, "HOME_POSITION": 0.5}


class PreflightError(RuntimeError):
    pass


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    r = 6371008.8
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def preflight_identity(meta: dict, *, worlds_root: Path = None, allow_fixture: bool = False) -> list:
    """Offline checks on the planned mission; returns warnings, raises PreflightError on blockers."""
    warnings = []
    if meta["front"].get("truncated"):
        raise PreflightError("mission comes from a TRUNCATED Tier 1 front (approximate)")
    from env_data import WeatherSnapshot
    reasons = WeatherSnapshot.from_dict(meta["weather"]).no_go_reasons()
    if reasons:
        raise PreflightError("weather gate NO-GO: " + "; ".join(reasons))
    wm_path = Path(worlds_root or _paths.WORLDS_DIR) / meta["region_id"] / "world_manifest.json"
    if wm_path.exists():
        wm = json.loads(wm_path.read_text(encoding="utf-8"))
        if wm["bundle_hash"] != meta["bundle_hash"]:
            raise PreflightError(f"world {wm_path.parent.name} built from bundle {wm['bundle_hash']}, "
                                 f"mission planned on {meta['bundle_hash']} — rebuild one of them")
    else:
        warnings.append(f"no world manifest at {wm_path} (cannot confirm world ↔ mission bundle)")
    if meta.get("fixture_data") and not allow_fixture:
        warnings.append("mission planned on HAND-MADE FIXTURE data (smoke test only)")
    return warnings


def world_vehicle_model(meta: dict, *, worlds_root: Path = None) -> str:
    """Vehicle the world was built with (worlds from before the model change still carry iris_with_gimbal)."""
    wm_path = Path(worlds_root or _paths.WORLDS_DIR) / meta["region_id"] / "world_manifest.json"
    if wm_path.exists():
        return json.loads(wm_path.read_text(encoding="utf-8")).get("vehicle_model", scfg.SIM_VEHICLE_MODEL)
    return scfg.SIM_VEHICLE_MODEL


def item_tuple(it) -> tuple:
    return (it.frame, it.command, round(it.p1, 3), round(it.p2, 3), round(it.p3, 3),
            int(round(it.lat * 1e7)), int(round(it.lon * 1e7)), round(it.alt, 2))


class SitlLink:
    """Thin pymavlink wrapper; every received message is passed to the logger."""

    def __init__(self, url: str, logger: MavlinkLogger = None, timeout: float = 60.0):
        from pymavlink import mavutil
        self.mavutil = mavutil
        self.ml = mavutil.mavlink
        self.logger = logger
        print(f"[SITL] Connecting {url} …")
        self.m = mavutil.mavlink_connection(url, source_system=254, source_component=191, autoreconnect=True)
        hb = self.m.wait_heartbeat(timeout=timeout)
        if hb is None:
            raise PreflightError(f"no heartbeat on {url} within {timeout:.0f} s")
        self.ts, self.tc = self.m.target_system, self.m.target_component or 1
        print(f"[SITL] Heartbeat from system {self.ts} ({self.m.flightmode})")
        self.home = None
        self.position = None          # (lat, lon, rel_alt_m) from GLOBAL_POSITION_INT
        self.armed_ever = False
        self.landed_state = None
        self.max_seq = -1             # highest mission item made current (ArduPilot rewinds to 1 after landing)

    # message pump ------------------------------------------------------------
    def recv(self, types=None, timeout: float = 1.0):
        end = time.time() + timeout
        while time.time() < end:
            msg = self.m.recv_match(blocking=True, timeout=max(0.01, end - time.time()))
            if msg is None:
                continue
            t = msg.get_type()
            if t == "BAD_DATA":
                continue
            if self.logger is not None:
                self.logger.record(msg)
            if t == "HOME_POSITION":
                self.home = (msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000.0)
            elif t == "GLOBAL_POSITION_INT":
                self.position = (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0)
            elif t == "MISSION_CURRENT" and self.armed_ever:
                self.max_seq = max(self.max_seq, msg.seq)
            elif t == "EXTENDED_SYS_STATE":
                self.landed_state = msg.landed_state
            elif t == "STATUSTEXT":
                print(f"[FC] {msg.text}")
            if types is None or t in types:
                return msg
        return None

    def command(self, cmd, *params, wait_ack: bool = True, timeout: float = 5.0):
        p = list(params) + [0] * (7 - len(params))
        self.m.mav.command_long_send(self.ts, self.tc, cmd, 0, *p)
        if not wait_ack:
            return None
        end = time.time() + timeout
        while time.time() < end:
            ack = self.recv({"COMMAND_ACK"}, timeout=end - time.time())
            if ack is not None and ack.command == cmd:
                return ack.result
        return None

    def set_intervals(self):
        for name, hz in STREAM_HZ.items():
            mid = getattr(self.ml, f"MAVLINK_MSG_ID_{name}")
            self.command(self.ml.MAV_CMD_SET_MESSAGE_INTERVAL, mid, int(1e6 / hz), wait_ack=False)

    def set_param(self, name: str, value: float, timeout: float = 5.0) -> bool:
        self.m.mav.param_set_send(self.ts, self.tc, name.encode(), float(value),
                                  self.ml.MAV_PARAM_TYPE_REAL32)
        end = time.time() + timeout
        while time.time() < end:
            msg = self.recv({"PARAM_VALUE"}, timeout=end - time.time())
            if msg is not None and msg.param_id.rstrip("\x00") == name:
                return abs(msg.param_value - value) < 1e-3
        return False

    def fetch_params(self, timeout: float = 30.0) -> dict:
        self.m.mav.param_request_list_send(self.ts, self.tc)
        params, total, end = {}, None, time.time() + timeout
        while time.time() < end:
            msg = self.recv({"PARAM_VALUE"}, timeout=2.0)
            if msg is None:
                if total is not None and len(params) >= total:
                    break
                continue
            params[msg.param_id.rstrip("\x00")] = msg.param_value
            total = msg.param_count
            if len(params) >= total:
                break
        return params

    def wait_home(self, timeout: float = 120.0):
        end = time.time() + timeout
        while time.time() < end and self.home is None:
            self.command(self.ml.MAV_CMD_REQUEST_MESSAGE, self.ml.MAVLINK_MSG_ID_HOME_POSITION, wait_ack=False)
            self.recv({"HOME_POSITION"}, timeout=2.0)
        if self.home is None:
            raise PreflightError("SITL never reported HOME_POSITION (GPS/EKF not ready?)")
        return self.home

    # mission protocol --------------------------------------------------------
    def upload(self, items, timeout: float = 30.0):
        self.m.mav.mission_clear_all_send(self.ts, self.tc)
        self.recv({"MISSION_ACK"}, timeout=2.0)
        self.m.mav.mission_count_send(self.ts, self.tc, len(items))
        end, sent = time.time() + timeout, set()
        while time.time() < end:
            msg = self.recv({"MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"}, timeout=5.0)
            if msg is None:
                continue
            if msg.get_type() == "MISSION_ACK":
                if msg.type == self.ml.MAV_MISSION_ACCEPTED and len(sent) == len(items):
                    return True
                raise PreflightError(f"mission rejected: MISSION_ACK type {msg.type}")
            it = items[msg.seq]
            self.m.mav.mission_item_int_send(
                self.ts, self.tc, it.index, it.frame, it.command, it.current, it.autocontinue,
                it.p1, it.p2, it.p3, it.p4, int(round(it.lat * 1e7)), int(round(it.lon * 1e7)), float(it.alt))
            sent.add(msg.seq)
        raise PreflightError("mission upload timed out")

    def download(self, timeout: float = 30.0) -> list:
        self.m.mav.mission_request_list_send(self.ts, self.tc)
        cnt = self.recv({"MISSION_COUNT"}, timeout=5.0)
        if cnt is None:
            raise PreflightError("no MISSION_COUNT on read-back")
        out = []
        for seq in range(cnt.count):
            self.m.mav.mission_request_int_send(self.ts, self.tc, seq)
            msg = self.recv({"MISSION_ITEM_INT"}, timeout=5.0)
            if msg is None or msg.seq != seq:
                raise PreflightError(f"read-back failed at item {seq}")
            out.append(msg)
        self.m.mav.mission_ack_send(self.ts, self.tc, self.ml.MAV_MISSION_ACCEPTED)
        return out

    # flight ------------------------------------------------------------------
    def set_mode(self, name: str, timeout: float = 10.0):
        mapping = self.m.mode_mapping()
        self.m.set_mode(mapping[name])
        end = time.time() + timeout
        while time.time() < end:
            self.recv({"HEARTBEAT"}, timeout=1.0)
            if self.m.flightmode == name:
                return True
        raise PreflightError(f"mode {name} not accepted (still {self.m.flightmode})")

    def arm(self, timeout: float = 60.0):
        end = time.time() + timeout
        while time.time() < end:
            self.command(self.ml.MAV_CMD_COMPONENT_ARM_DISARM, 1, timeout=3.0)
            for _ in range(3):
                self.recv({"HEARTBEAT"}, timeout=1.0)
                if self.m.motors_armed():
                    self.armed_ever = True
                    return True
        raise PreflightError("arming failed (see [FC] PreArm messages)")


def verify_readback(sent, got) -> list:
    """Differences between uploaded and read-back items (home row 0 is replaced by the FC)."""
    diffs = []
    if len(got) != len(sent):
        return [f"count {len(got)} != {len(sent)}"]
    for it, msg in zip(sent[1:], got[1:]):
        a = item_tuple(it)
        b = (msg.frame, msg.command, round(msg.param1, 3), round(msg.param2, 3), round(msg.param3, 3),
             msg.x, msg.y, round(msg.z, 2))
        positional = it.command < 80                    # MAV_CMD_NAV_*; DO_* items come back frame-less
        if (a[1] != b[1] or (positional and a[0] != b[0]) or abs(a[5] - b[5]) > 2 or abs(a[6] - b[6]) > 2
                or abs(a[7] - b[7]) > 0.05 or (not positional and a[2:5] != b[2:5])):
            diffs.append(f"item {it.index}: sent {a} got {b}")
    return diffs


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mission_dir")
    p.add_argument("--url", default=scfg.SIM_MAVLINK_URL, help="pymavlink URL (tcp:127.0.0.1:5762 = SITL SERIAL1)")
    p.add_argument("--no-fly", action="store_true", help="checks + upload + read-back only")
    p.add_argument("--no-gz-log", action="store_true")
    p.add_argument("--vehicle-model", default=None,
                   help="Gazebo model for ground-truth pose; default: the world manifest's, else sim_config")
    p.add_argument("--timeout", type=float, default=scfg.SIM_MISSION_TIMEOUT_S)
    p.add_argument("--allow-fixture", action="store_true")
    p.add_argument("--start-file", type=Path,
                   help="after upload, hold disarmed on the pad until this file exists (writes <file>.ready first)")
    a = p.parse_args(argv)

    ready = None
    if a.start_file is not None:                # stale signals from an earlier (crashed) run
        ready = a.start_file.with_name(a.start_file.name + ".ready")
        a.start_file.unlink(missing_ok=True)
        ready.unlink(missing_ok=True)

    mdir = Path(a.mission_dir).resolve()
    meta, items = load_mission(mdir)
    try:
        for w in preflight_identity(meta, allow_fixture=a.allow_fixture):
            print(f"[Preflight] WARNING: {w}")
    except PreflightError as exc:
        print(f"[Preflight] BLOCKED: {exc}")
        return 2

    run_dir = _paths.RUNS_DIR / f"{meta['run_id']}__{datetime.now().strftime('%H%M%S')}"
    run_dir.mkdir(parents=True, exist_ok=True)
    for name in ("mission.json", "front_world_xyz.json", "mission.plan"):
        if (mdir / name).exists():
            shutil.copy(mdir / name, run_dir / name)
    logger = MavlinkLogger(run_dir)
    gz = None
    manifest = {"run_id": meta["run_id"], "mission_dir": str(mdir), "url": a.url,
                "started_wall": datetime.now().isoformat(timespec="seconds"), "status": "started"}

    def save_manifest(**kw):
        manifest.update(kw)
        (run_dir / "run_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    try:
        link = SitlLink(a.url, logger)
        link.set_intervals()
        depot = meta["pads"]["depot"]
        home = link.wait_home()
        dh = haversine_m(home[0], home[1], depot["lat"], depot["lon"])
        dv = abs(home[2] - meta["h_home_amsl_m"])
        print(f"[Preflight] SITL home {home[0]:.7f}, {home[1]:.7f}, {home[2]:.2f} m → "
              f"{dh:.2f} m / {dv:.2f} m from the depot pad")
        if dh > scfg.SIM_HOME_TOL_H_M or dv > scfg.SIM_HOME_TOL_V_M:
            raise PreflightError("SITL home does not match the depot pad — start sim_vehicle.py with "
                                 f"the --custom-location printed in worlds/{meta['region_id']}/sitl_command.txt")
        # Home only moves on arming: the vehicle itself must also stand on the depot pad
        # (after a delivery it sits on the destination pad until Gazebo + SITL are restarted).
        end = time.time() + 60.0
        while link.position is None:
            if time.time() > end:
                raise PreflightError("SITL never reported GLOBAL_POSITION_INT within 60 s (EKF not ready?)")
            link.recv({"GLOBAL_POSITION_INT"}, timeout=2.0)
        dp = haversine_m(link.position[0], link.position[1], depot["lat"], depot["lon"])
        if dp > scfg.SIM_HOME_TOL_H_M or abs(link.position[2]) > scfg.SIM_HOME_TOL_V_M:
            raise PreflightError(f"vehicle is {dp:.0f} m from the depot pad at {link.position[2]:.1f} m above home "
                                 "— restart Gazebo and SITL so the drone starts on the depot pad")
        params = link.fetch_params()
        wanted = dict(SIM_PARAMS)
        speed = next(((n, v) for n, v in SPEED_PARAMS if n in params), None)
        if speed:
            wanted[speed[0]] = speed[1]
        for name, value in wanted.items():
            if name not in params:
                print(f"[Preflight] WARNING: parameter {name} does not exist on this firmware")
            elif link.set_param(name, value):
                params[name] = value
            else:
                print(f"[Preflight] WARNING: parameter {name}={value} not confirmed")
        (run_dir / "params.json").write_text(json.dumps(params, indent=1, sort_keys=True), encoding="utf-8")

        t0 = time.time()
        link.upload(items)
        diffs = verify_readback(items, link.download())
        if diffs:
            raise PreflightError("read-back mismatch: " + "; ".join(diffs[:5]))
        upload_s = time.time() - t0
        print(f"[SITL] Mission accepted and verified ({len(items)} items, {upload_s:.2f} s)")
        logger.event("mission_uploaded", detail=f"{len(items)} items {upload_s:.3f}s")
        save_manifest(status="uploaded", sitl_home=home, home_offset_m=[dh, dv], upload_s=upload_s)
        if a.no_fly:
            return 0

        if a.start_file is not None:            # recording: hold on the pad until released
            save_manifest(status="waiting_for_start", start_file=str(a.start_file))
            logger.event("waiting_for_start")
            ready.touch()
            print(f"[SITL] READY ON GROUND, disarmed. Waiting for the start signal ({a.start_file.name}).",
                  flush=True)
            while not a.start_file.exists():
                link.recv(timeout=0.5)
            a.start_file.unlink(missing_ok=True)
            ready.unlink(missing_ok=True)
            logger.event("start_released")
            print("[SITL] Start signal received.", flush=True)

        if not a.no_gz_log:
            gz = GazeboPoseLogger(meta["world_name"], a.vehicle_model or world_vehicle_model(meta), run_dir)
            gz.start()
        link.set_mode("GUIDED")
        link.arm()
        link.set_mode("AUTO")
        link.command(link.ml.MAV_CMD_MISSION_START, 0, 0)
        logger.event("auto_start")
        t_start = time.time()
        print("[SITL] AUTO mission started")
        last_seq, n_items = None, len(items)
        land_seq = n_items - 1
        status = "timeout"
        while time.time() - t_start < a.timeout:
            link.recv(timeout=0.5)
            seq = logger.last_mission_seq
            if seq != last_seq and seq is not None:
                if seq > (last_seq or 0):
                    print(f"[SITL] → item {seq}/{land_seq}")
                last_seq = seq
            if link.armed_ever and not link.m.motors_armed():
                status = "landed_disarmed" if link.max_seq >= land_seq else "disarmed_early"
                break
        flight_s = time.time() - t_start
        if status == "timeout":
            print("[SITL] TIMEOUT — switching to LAND")
            link.set_mode("LAND")
        logger.event("end", detail=status)
        print(f"[SITL] Flight ended: {status} after {flight_s:.0f} s")
        save_manifest(status=status, flight_wall_s=flight_s, max_mission_seq=link.max_seq,
                      ended_wall=datetime.now().isoformat(timespec="seconds"))
        print(f"[SITL] Run logs → {run_dir}\n[SITL] Next: python evaluate_gazebo_run.py {run_dir}")
        return 0 if status == "landed_disarmed" else 1
    except PreflightError as exc:
        print(f"[SITL] ABORT: {exc}")
        save_manifest(status="aborted", error=str(exc))
        return 2
    except KeyboardInterrupt:
        save_manifest(status="interrupted")
        print("[SITL] Interrupted — take over in QGC / MAVProxy (e.g. mode LAND)")
        return 130
    finally:
        if gz is not None:
            gz.stop()
            manifest["gz_pose_rows"] = gz.rows
        logger.close()
        if manifest.get("status") != "started":
            (run_dir / "run_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")


if __name__ == "__main__":
    sys.exit(main())
