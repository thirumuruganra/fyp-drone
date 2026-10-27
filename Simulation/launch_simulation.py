"""
One-command simulated delivery (WSL2 / Linux): region → world → mission → Gazebo + SITL → fly → evaluate.

    source gazebo/env.sh                                   # once per shell (finds ~/ardupilot, ~/ardupilot_gazebo)
    python launch_simulation.py                            # thiru_demo, mixed_cargo
    python launch_simulation.py --preset fragile_medical   # same stored Tier 1 front, new Tier 2 pick
    python launch_simulation.py --scene-only               # world + route markers, no flight
    python launch_simulation.py --gcs-host 127.0.0.1       # QGC address (default: Windows host on WSL2)
    python launch_simulation.py --wait-for-start           # recording: hold on the pad until Enter

Replaces the three terminals of README Phase 4. Each launch starts a fresh Gazebo and a fresh
ArduCopter SITL (wiped EEPROM, home = depot pad), so no manual restart is needed between flights.
SITL runs as the bare binary with the same --model JSON and Iris parameter files as
sitl_command.txt; telemetry goes to QGroundControl on UDP 14550 and to the mission runner on TCP 5762.
On WSL2 the Windows QGroundControl is opened if it is not already running (--no-ground-station skips it).
Camera (--view): takeoff (default) films the lift-off from beside the depot pad and switches to a close
follow camera at 6 m (gazebo/camera_director.py, system python3); follow / wide follow from the start.
Close the Gazebo window or press Ctrl+C to stop everything.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import os
import signal
import socket
import subprocess
import shutil
import sys
import threading
import time
from pathlib import Path

import _paths
import sim_config as scfg

SITL_TCP_PORT = int(scfg.SIM_MAVLINK_URL.rsplit(":", 1)[1])
GCS_UDP_PORT = 14550


def wait_tcp(port: int, process: subprocess.Popen, timeout: float = 90.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("ArduPilot exited; inspect ardupilot.log")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=1):
                return
        except OSError:
            time.sleep(1)
    raise RuntimeError(f"ArduPilot did not open TCP {port}")


def stop(process: subprocess.Popen) -> None:
    """Terminate a child started with start_new_session=True, including its own children."""
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def under_wsl2() -> bool:
    return "microsoft-standard-WSL2" in os.uname().release


def gcs_host(choice: str) -> str:
    """QGroundControl address. 'auto': the Windows host on WSL2 (default gateway, as sim_vehicle.py
    does; correct under NAT networking), else 127.0.0.1. 'nat' forces the gateway; an IP is used as given."""
    if choice == "auto" and not under_wsl2():
        return "127.0.0.1"
    if choice not in ("auto", "nat"):
        return choice
    out = subprocess.run(["ip", "route", "show", "default"], capture_output=True, text=True, check=True).stdout
    parts = out.split()
    if "via" not in parts:
        raise RuntimeError("no default route; pass --gcs-host <windows-ip>")
    return parts[parts.index("via") + 1]


def windows_exe(name: str) -> str:
    return shutil.which(name) or f"/mnt/c/Windows/System32/{name}"


def open_ground_station(exe: str) -> str:
    """Start Windows QGroundControl from WSL2 unless one is already running; returns what happened."""
    if not under_wsl2():
        return "not started (only on WSL2)"
    listing = subprocess.run([windows_exe("tasklist.exe"), "/FI", "IMAGENAME eq QGroundControl*", "/NH"],
                             capture_output=True, text=True, cwd="/mnt/c", check=False).stdout
    if "QGroundControl" in listing:
        return "already running"
    unix = subprocess.run(["wslpath", "-u", exe], capture_output=True, text=True, check=False).stdout.strip()
    if not unix or not Path(unix).is_file():
        return f"not found at {exe} (set FYP_QGC_EXE or --qgc-exe)"
    subprocess.Popen([windows_exe("powershell.exe"), "-NoProfile", "-Command",
                      f"Start-Process -FilePath '{exe}'"], cwd="/mnt/c", stdin=subprocess.DEVNULL,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    return f"opened {exe}"


def camera_director(sim: Path, world: str, model: str, view: str) -> subprocess.Popen | None:
    """Start gazebo/camera_director.py under the system python3 (Gazebo's Python bindings live there)."""
    python = "/usr/bin/python3"
    probe = subprocess.run([python, "-c", "import gz.transport13, gz.msgs10"], capture_output=True, check=False)
    if probe.returncode:
        print("Camera: python3-gz-transport13 / python3-gz-msgs10 missing; gui.config camera only.", flush=True)
        return None
    if view == "takeoff":
        place = ["--takeoff-pos", *map(str, scfg.SIM_CAMERA_VIEWS["takeoff"]),
                 "--follow-offset", *map(str, scfg.SIM_CAMERA_VIEWS["follow"])]
    else:
        place = ["--follow-offset", *map(str, scfg.SIM_CAMERA_VIEWS[view])]
    return subprocess.Popen([python, "-u", str(sim / "gazebo/camera_director.py"), "--world", world,
                             "--model", model, "--view", view, "--switch-alt", str(scfg.SIM_CAMERA_SWITCH_ALT_M),
                             "--parent-pid", str(os.getpid()), *place],
                            cwd=sim, stdin=subprocess.DEVNULL, start_new_session=True)


def wait_for_enter(start_file: Path, stop: threading.Event) -> None:
    """Release the runner's pad hold (--start-file) when the user presses Enter."""
    ready = start_file.with_name(start_file.name + ".ready")
    while not ready.exists():
        if stop.is_set():
            return
        time.sleep(0.5)
    print("\n>>> READY ON GROUND (disarmed). Start recording, then press Enter to take off.", flush=True)
    try:
        sys.stdin.readline()        # EOF (no terminal attached) releases immediately
    except (OSError, ValueError):
        pass
    if not stop.is_set():
        start_file.touch()


def prepare_args(region_id: str, custom: list | None = None) -> list:
    if custom:
        return ["--strict", "--region-id", region_id, *custom]
    if region_id == "smoke_offline":
        return ["--offline", "--region-id", region_id]
    if region_id == scfg.SIM_REGION_ID:
        return ["--strict", "--region-id", region_id]      # pads, box and centre from sim_config.py
    raise RuntimeError(f"Region '{region_id}' missing. Run prepare_sim_region.py for it first.")


def latest_mission(region_id: str, label: str) -> Path:
    candidates = []
    for p in _paths.MISSIONS_DIR.glob("*/mission.json"):
        meta = json.loads(p.read_text(encoding="utf-8"))
        if meta.get("region_id") == region_id and meta.get("label") == label:
            candidates.append(p)
    if not candidates:
        raise RuntimeError(f"run_sim_mission.py wrote no '{label}' mission for {region_id}")
    return max(candidates, key=lambda p: p.stat().st_mtime).parent


def latest_run(mission: Path) -> Path | None:
    runs = sorted(_paths.RUNS_DIR.glob(f"{mission.name}__*"), key=lambda p: p.stat().st_mtime)
    return runs[-1] if runs else None


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--region-id", default=scfg.SIM_REGION_ID)
    parser.add_argument("--spacing-m", type=float, default=scfg.SIM_SPACING_M)
    parser.add_argument("--preset", default="mixed_cargo")
    parser.add_argument("--dest-index", type=int, default=0)
    parser.add_argument("--center", nargs=2, type=float, metavar=("LAT", "LON"),
                        help="NEW region: lattice centre (with --region-id; prepared on first launch)")
    parser.add_argument("--depot", nargs=2, type=float, metavar=("LAT", "LON"), help="NEW region: depot pad")
    parser.add_argument("--dest", nargs=2, type=float, action="append", metavar=("LAT", "LON"),
                        help="NEW region: delivery pad (repeat for several)")
    parser.add_argument("--box-m", type=float, help="NEW region: box size in metres")
    parser.add_argument("--scene-only", action="store_true", help="show world and route markers without flying")
    parser.add_argument("--view", choices=sorted(scfg.SIM_CAMERA_VIEWS), default="takeoff",
                        help="takeoff: pad shot, then follow at 6 m; follow: close chase; wide: distant chase")
    parser.add_argument("--wait-for-start", action="store_true",
                        help="hold disarmed on the depot pad after upload until Enter is pressed")
    parser.add_argument("--no-ground-station", action="store_true", help="do not open Windows QGroundControl")
    parser.add_argument("--qgc-exe", default=os.environ.get("FYP_QGC_EXE", scfg.SIM_QGC_EXE),
                        help="Windows path of QGroundControl.exe (env FYP_QGC_EXE)")
    parser.add_argument("--terrain-res-m", type=float, default=scfg.SIM_DISPLAY_TERRAIN_RES_M,
                        help="Gazebo terrain mesh step; display only, planner uses the bundle DEM")
    parser.add_argument("--no-route-markers", action="store_true",
                        help="do not draw the Tier 1 front / Tier 2 route lines in Gazebo (clean footage)")
    parser.add_argument("--no-sky", action="store_true", help="lighter rendering (WSLg / software GL)")
    parser.add_argument("--render-engine", choices=("ogre", "ogre2"), help="pass ogre if WSLg shows a black window")
    parser.add_argument("--vehicle-model", default=scfg.SIM_VEHICLE_MODEL)
    parser.add_argument("--gcs-host", default="auto",
                        help="QGroundControl address: auto (Windows host on WSL2, else 127.0.0.1), nat, or an IP")
    parser.add_argument("--no-evaluate", action="store_true", help="skip evaluate_gazebo_run.py after landing")
    args = parser.parse_args(argv)

    sim = _paths.SIM_DIR
    (sim / ".local").mkdir(exist_ok=True)
    lock = (sim / ".local/launch.lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError("Simulation already running. Close its Gazebo window first.")
    if not args.scene_only:
        with socket.socket() as probe:
            try:
                probe.bind(("127.0.0.1", SITL_TCP_PORT))
            except OSError:
                raise RuntimeError(f"TCP {SITL_TCP_PORT} occupied. Stop the previous SITL (or PX4) first.")
    ap = Path(os.environ.get("FYP_ARDUPILOT_DIR", Path.home() / "ardupilot"))
    binary = ap / "build/sitl/bin/arducopter"
    if not args.scene_only and not binary.is_file():
        raise RuntimeError(f"ArduCopter SITL not built: {binary} (README Phase 0, or set FYP_ARDUPILOT_DIR)")

    custom = []
    for flag, val in (("--center", args.center), ("--depot", args.depot), ("--box-m", args.box_m)):
        if val is not None:
            custom += [flag, *map(str, val if isinstance(val, list) else [val])]
    for dest in args.dest or []:
        custom += ["--dest", *map(str, dest)]
    if (_paths.REGIONS_DIR / args.region_id / "manifest.json").exists():
        if custom:
            print(f"[Launch] region '{args.region_id}' already exists: --center/--depot/--dest/--box-m ignored "
                  f"(pick a new --region-id, or delete regions/{args.region_id})", flush=True)
    else:
        subprocess.run([sys.executable, str(sim / "prepare_sim_region.py"), *prepare_args(args.region_id, custom)],
                       cwd=sim, check=True)
    from gazebo_world_builder import build_world
    from sim_region import RegionBundle
    bundle = RegionBundle.load(args.region_id)
    print("Data: HAND-MADE FIXTURES (smoke test, not real data)" if bundle.is_fixture
          else "Data: frozen region bundle", flush=True)
    # World meshes are cheap to rebuild; the clearance graph and Tier 1 front stay cached.
    build_world(args.region_id, terrain_res_m=args.terrain_res_m, vehicle_model=args.vehicle_model,
                sky=not args.no_sky, gui_view=args.view)
    subprocess.run([sys.executable, str(sim / "run_sim_mission.py"), "--region-id", args.region_id,
                    "--preset", args.preset, "--dest-index", str(args.dest_index),
                    "--spacing-m", str(args.spacing_m)], cwd=sim, check=True)
    mission = latest_mission(args.region_id, args.preset)

    session = _paths.RUNS_DIR / ("launch_" + time.strftime("%Y%m%d_%H%M%S"))
    session.mkdir(parents=True)
    print(f"Mission: {mission}\nSession logs: {session}\nClose Gazebo or press Ctrl+C to stop.", flush=True)
    gz_extra = ["--render-engine", args.render_engine] if args.render_engine else []
    world = json.loads((_paths.WORLDS_DIR / args.region_id / "world_manifest.json").read_text())["world_name"]
    start_file = sim / ".local/start-flight"
    stop_prompt = threading.Event()
    children = []
    try:
        with (session / "gazebo.log").open("w") as gz_log, (session / "ardupilot.log").open("w") as ap_log:
            gazebo = subprocess.Popen(["bash", str(sim / "gazebo/launch_region.sh"), args.region_id,
                                       str(mission), *gz_extra], cwd=sim, stdout=gz_log,
                                      stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                      env={**os.environ, **({"NO_ROUTE_MARKERS": "1"} if args.no_route_markers else {})},
                                      start_new_session=True)
            children.append(gazebo)
            director = camera_director(sim, world, args.vehicle_model, args.view)
            if director is not None:
                children.append(director)
            if not args.no_ground_station:
                print(f"QGroundControl: {open_ground_station(args.qgc_exe)}", flush=True)
            if args.scene_only:
                return gazebo.wait()

            defaults = ap / "Tools/autotest/default_params"
            home = f"{bundle.depot[0]:.7f},{bundle.depot[1]:.7f},{bundle.h_home:.3f},0"
            gcs = gcs_host(args.gcs_host)
            # Same JSON backend, Iris defaults + project overrides and home as sitl_command.txt, no MAVProxy.
            sitl = subprocess.Popen([str(binary), "-w", "--model", "JSON", "--speedup", "1",
                                     "--defaults", f"{defaults / 'copter.parm'},{defaults / 'gazebo-iris.parm'},"
                                                   f"{_paths.SITL_OVERRIDES}",
                                     "--home", home, "--serial0", f"udpclient:{gcs}:{GCS_UDP_PORT}",
                                     "--serial1", f"tcp:{SITL_TCP_PORT}"],
                                    cwd=session, stdout=ap_log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                    start_new_session=True)
            children.append(sitl)
            wait_tcp(SITL_TCP_PORT, sitl)
            print(f"QGroundControl: telemetry → {gcs}:{GCS_UDP_PORT} (UDP; QGC auto-connects).", flush=True)

            runner_args = [sys.executable, "-u", str(sim / "sitl_mission_runner.py"), str(mission),
                           "--vehicle-model", args.vehicle_model]
            if args.wait_for_start:
                for stale in (start_file, start_file.with_name(start_file.name + ".ready")):
                    stale.unlink(missing_ok=True)
                runner_args += ["--start-file", str(start_file)]
                threading.Thread(target=wait_for_enter, args=(start_file, stop_prompt), daemon=True).start()
            runner = subprocess.Popen(runner_args, cwd=sim, stdin=subprocess.DEVNULL, start_new_session=True)
            children.append(runner)
            while runner.poll() is None:
                if gazebo.poll() is not None:
                    print("Gazebo closed; stopping the flight.", flush=True)
                    return 0
                if sitl.poll() is not None:
                    raise RuntimeError("ArduPilot exited; inspect ardupilot.log")
                time.sleep(1)
            if runner.returncode:
                print(f"Mission stopped (exit {runner.returncode}); scene stays open. Inspect the run logs.",
                      flush=True)
            else:
                print("Delivery finished. Scene stays open; close the window to stop.", flush=True)
                run = latest_run(mission)
                if run is not None and not args.no_evaluate:
                    subprocess.run([sys.executable, str(sim / "evaluate_gazebo_run.py"), str(run)],
                                   cwd=sim, check=False)
            return gazebo.wait()
    except KeyboardInterrupt:
        return 130
    finally:
        stop_prompt.set()
        for process in reversed(children):
            stop(process)


if __name__ == "__main__":
    def _interrupt(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _interrupt)
    signal.signal(signal.SIGINT, _interrupt)
    try:
        sys.exit(main())
    except (RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"Simulation launch failed: {exc}", file=sys.stderr)
        sys.exit(1)
