"""
Run logging for SITL flights: MAVLink telemetry CSVs and Gazebo ground-truth pose.

    MavlinkLogger(run_dir).record(msg)     → global_position.csv, local_position.csv,
                                             attitude.csv, mission_events.csv,
                                             battery.csv, statustext.log
    GazeboPoseLogger(world, model, run_dir) → gz_pose.csv (sim time, x, y, z, quaternion)
                                             from `gz topic -e -t /world/<w>/dynamic_pose/info`

Times: every row carries wall-clock seconds (`wall_s`); MAVLink rows also carry
the autopilot `time_boot_ms`, Gazebo rows the simulation time `sim_s`.
"""

from __future__ import annotations

import csv
import json
import re
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

TELEMETRY_FIELDS = {
    "GLOBAL_POSITION_INT": ("global_position.csv",
                            ["wall_s", "time_boot_ms", "lat", "lon", "alt_msl_m", "rel_alt_m",
                             "vx_ms", "vy_ms", "vz_ms", "hdg_deg"]),
    "LOCAL_POSITION_NED": ("local_position.csv", ["wall_s", "time_boot_ms", "x_n", "y_e", "z_d", "vx", "vy", "vz"]),
    "ATTITUDE": ("attitude.csv", ["wall_s", "time_boot_ms", "roll", "pitch", "yaw"]),
    "BATTERY_STATUS": ("battery.csv", ["wall_s", "voltage_v", "current_a", "remaining_pct"]),
}
EVENT_FIELDS = ["wall_s", "event", "seq", "detail"]


class MavlinkLogger:
    def __init__(self, run_dir: Path):
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self._files, self._writers = {}, {}
        for mtype, (fname, cols) in TELEMETRY_FIELDS.items():
            fh = open(self.run_dir / fname, "w", newline="", encoding="utf-8")
            self._files[mtype] = fh
            self._writers[mtype] = csv.writer(fh)
            self._writers[mtype].writerow(cols)
        self._ev_fh = open(self.run_dir / "mission_events.csv", "w", newline="", encoding="utf-8")
        self._ev = csv.writer(self._ev_fh)
        self._ev.writerow(EVENT_FIELDS)
        self._txt = open(self.run_dir / "statustext.log", "w", encoding="utf-8")
        self.last_mission_seq: Optional[int] = None

    def event(self, event: str, seq=None, detail: str = ""):
        self._ev.writerow([f"{time.time():.3f}", event, "" if seq is None else seq, detail])
        self._ev_fh.flush()

    def record(self, msg) -> None:
        t = msg.get_type()
        now = f"{time.time():.3f}"
        w = self._writers.get(t)
        if t == "GLOBAL_POSITION_INT":
            w.writerow([now, msg.time_boot_ms, msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0,
                        msg.relative_alt / 1000.0, msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0,
                        msg.hdg / 100.0 if msg.hdg != 65535 else ""])
        elif t == "LOCAL_POSITION_NED":
            w.writerow([now, msg.time_boot_ms, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz])
        elif t == "ATTITUDE":
            w.writerow([now, msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw])
        elif t == "BATTERY_STATUS":
            v = [x for x in msg.voltages if x != 65535]
            w.writerow([now, sum(v) / 1000.0 if v else "", msg.current_battery / 100.0
                        if msg.current_battery >= 0 else "", msg.battery_remaining])
        elif t == "MISSION_CURRENT":
            if msg.seq != self.last_mission_seq:
                self.last_mission_seq = msg.seq
                self.event("mission_current", msg.seq)
        elif t == "MISSION_ITEM_REACHED":
            self.event("item_reached", msg.seq)
        elif t == "STATUSTEXT":
            self._txt.write(f"{now} [{msg.severity}] {msg.text}\n")
            self._txt.flush()

    def close(self):
        for fh in self._files.values():
            fh.close()
        self._ev_fh.close()
        self._txt.close()


# ── Gazebo ground truth ──────────────────────────────────────────────────────

_NUM = r"(-?[0-9.eE+-]+)"


def parse_pose_message(text: str, model: str) -> Optional[dict]:
    """
    Pose of `model` from one gz.msgs.Pose_V message, JSON (--json-output) or
    protobuf text format.  None when the model is not in the message.
    """
    text = text.strip()
    if text.startswith("{"):
        try:
            d = json.loads(text)
        except ValueError:
            return None
        stamp = d.get("header", {}).get("stamp", {})
        sim_s = float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) * 1e-9
        for p in d.get("pose", []):
            if p.get("name") == model:
                pos, ori = p.get("position", {}), p.get("orientation", {})
                return {"sim_s": sim_s, "x": float(pos.get("x", 0)), "y": float(pos.get("y", 0)),
                        "z": float(pos.get("z", 0)), "qw": float(ori.get("w", 1)), "qx": float(ori.get("x", 0)),
                        "qy": float(ori.get("y", 0)), "qz": float(ori.get("z", 0))}
        return None
    # protobuf text format: find the pose block whose name matches
    sec = re.search(r"stamp\s*\{\s*sec:\s*(\d+)(?:\s*nsec:\s*(\d+))?", text)
    sim_s = (float(sec.group(1)) + float(sec.group(2) or 0) * 1e-9) if sec else 0.0
    for block in re.split(r"\n(?=pose\s*\{)", text):
        if not re.search(r'name:\s*"' + re.escape(model) + r'"', block):
            continue

        def grab(section, key, default):
            m = re.search(section + r"\s*\{([^}]*)\}", block)
            if not m:
                return default
            k = re.search(r"\b" + key + r":\s*" + _NUM, m.group(1))
            return float(k.group(1)) if k else default
        return {"sim_s": sim_s, "x": grab("position", "x", 0.0), "y": grab("position", "y", 0.0),
                "z": grab("position", "z", 0.0), "qw": grab("orientation", "w", 1.0),
                "qx": grab("orientation", "x", 0.0), "qy": grab("orientation", "y", 0.0),
                "qz": grab("orientation", "z", 0.0)}
    return None


class GazeboPoseLogger:
    """Background `gz topic -e` reader writing the vehicle's world pose to gz_pose.csv."""

    def __init__(self, world: str, model: str, run_dir: Path, gz: str = "gz", min_period_s: float = 0.05):
        self.world, self.model = world, model
        self.path = Path(run_dir) / "gz_pose.csv"
        self.gz = gz
        self.min_period_s = min_period_s
        self.proc = None
        self.thread = None
        self.rows = 0
        self.available = shutil.which(gz) is not None

    def start(self) -> bool:
        if not self.available:
            print("[Log] `gz` not on PATH — no Gazebo ground-truth pose (MAVLink only)")
            return False
        topic = f"/world/{self.world}/dynamic_pose/info"
        cmd = [self.gz, "topic", "-e", "-t", topic, "--json-output"]
        try:
            self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        except OSError as exc:
            print(f"[Log] cannot start {' '.join(cmd)}: {exc}")
            return False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        print(f"[Log] Gazebo pose ← {topic} (model {self.model})")
        return True

    def _run(self):
        with open(self.path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["wall_s", "sim_s", "x", "y", "z", "qw", "qx", "qy", "qz"])
            buf, depth, last = [], 0, 0.0
            for line in self.proc.stdout:
                buf.append(line)
                depth += line.count("{") - line.count("}")
                if depth > 0:
                    continue
                msg, buf, depth = "".join(buf), [], 0
                if not msg.strip():
                    continue
                now = time.time()
                if now - last < self.min_period_s:
                    continue
                pose = parse_pose_message(msg, self.model)
                if pose is None:
                    continue
                last = now
                w.writerow([f"{now:.3f}", f"{pose['sim_s']:.4f}"] +
                           [f"{pose[k]:.4f}" for k in ("x", "y", "z", "qw", "qx", "qy", "qz")])
                self.rows += 1
                if self.rows % 20 == 0:
                    fh.flush()

    def stop(self):
        if self.proc is not None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        if self.thread is not None:
            self.thread.join(timeout=3)
