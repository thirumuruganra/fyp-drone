"""Launch the project world, visible Gazebo GUI and an ArduPilot delivery.

Run through Launch Simulation.command (or source gazebo/env.sh in bash first).
Default scene covers real Thiruvanmiyur/Adyar. First preparation uses keyless
public data; subsequent launches reuse the frozen bundle. Synthetic fixture
demonstration remains available with --region-id smoke_offline.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

import _paths
import sim_config as scfg


def wait_tcp(port, process, timeout=90):
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


def stop(process):
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--region-id", default=scfg.DESKTOP_REGION_ID)
    parser.add_argument("--spacing-m", type=float, default=scfg.DESKTOP_SPACING_M)
    parser.add_argument("--preset", default="mixed_cargo")
    parser.add_argument("--route-style", choices=("turns", "planner"), default="turns",
                        help="ten-point recording corridor by default; planner for free route selection")
    parser.add_argument("--scene-only", action="store_true", help="show world without flying")
    parser.add_argument("--view", choices=scfg.DESKTOP_CAMERA_OFFSETS, default="takeoff")
    parser.add_argument("--wait-for-start", action="store_true",
                        help="hold disarmed on depot until .local/start-flight file is created")
    parser.add_argument("--no-sky", action="store_true", help="lighter rendering without clouds")
    parser.add_argument("--no-ground-station", action="store_true", help="skip opening QGroundControl")
    parser.add_argument("--vehicle-model", default="iris_with_ardupilot",
                        help="camera-free Iris by default; avoids optional video plugin dependencies")
    args = parser.parse_args(argv)
    sim = _paths.SIM_DIR
    (sim / ".local").mkdir(exist_ok=True)
    lock = (sim / ".local/desktop.lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError("Simulation already running. Close its Gazebo window before restarting.")
    start_file = sim / ".local/start-flight"
    if args.wait_for_start:
        start_file.unlink(missing_ok=True)
    if not args.scene_only:
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(("127.0.0.1", 5762))
            except OSError:
                raise RuntimeError("TCP 5762 occupied. Stop previous SITL instance before launching.")
    ap = Path(os.environ.get("FYP_ARDUPILOT_DIR", sim / ".local/ardupilot"))
    binary = ap / "build/sitl/bin/arducopter"
    if not args.scene_only and not binary.is_file():
        raise RuntimeError(f"Build ArduCopter SITL first: {binary}")
    region = _paths.REGIONS_DIR / args.region_id
    if not (region / "manifest.json").exists():
        if args.region_id == "smoke_offline":
            prepare = ["--offline", "--region-id", args.region_id]
        elif args.region_id == scfg.DESKTOP_REGION_ID:
            prepare = ["--keyless", "--strict", "--region-id", args.region_id,
                       "--center", *map(str, scfg.DESKTOP_CENTER),
                       "--box-m", str(scfg.DESKTOP_BOX_M),
                       "--osm-radius-m", str(scfg.DESKTOP_OSM_RADIUS_M),
                       "--depot", *map(str, scfg.DESKTOP_DEPOT),
                       "--dest", *map(str, scfg.DESKTOP_DESTINATION)]
        else:
            raise RuntimeError("Region missing. Run prepare_sim_region.py for this region first.")
        subprocess.run([sys.executable, str(sim / "prepare_sim_region.py"), *prepare], cwd=sim, check=True)
    from gazebo_world_builder import build_world
    from sim_region import RegionBundle
    bundle = RegionBundle.load(args.region_id)
    print("Data: SYNTHETIC FIXTURES (setup demonstration)" if bundle.is_fixture else "Data: frozen region bundle",
          flush=True)
    # Rebuild cheap scene assets, retaining the cached Pareto front.
    build_world(args.region_id, sky=not args.no_sky, gui_view=args.view, vehicle_model=args.vehicle_model,
                terrain_res_m=scfg.DESKTOP_TERRAIN_RES_M)
    mission_args = [sys.executable, str(sim / "run_sim_mission.py"), "--region-id", args.region_id,
                    "--preset", args.preset, "--spacing-m", str(args.spacing_m)]
    if args.route_style == "turns":
        mission_args.append("--recording-turns")
    subprocess.run(mission_args, cwd=sim, check=True)
    candidates = [p for p in _paths.MISSIONS_DIR.glob("*/mission.json")
                  if json.loads(p.read_text())["region_id"] == args.region_id]
    mission = max(candidates, key=lambda p: p.stat().st_mtime).parent
    session = _paths.RUNS_DIR / ("desktop_" + time.strftime("%Y%m%d_%H%M%S"))
    session.mkdir(parents=True)
    print(f"Session logs: {session}\nClose Gazebo window or press Ctrl+C to stop.", flush=True)
    children = []
    try:
        with (session / "gazebo.log").open("w") as gz_log, (session / "ardupilot.log").open("w") as ap_log:
            gazebo = subprocess.Popen(["bash", str(sim / "gazebo/launch_region.sh"), args.region_id,
                                       str(mission)], cwd=sim, stdout=gz_log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            children.append(gazebo)
            if not args.scene_only:
                defaults = ap / "Tools/autotest/default_params"
                home = f"{bundle.depot[0]:.7f},{bundle.depot[1]:.7f},{bundle.h_home:.3f},0"
                # Same JSON backend and explicit Iris defaults as sim_vehicle.py;
                # direct binary avoids opening additional Terminal/MAVProxy windows.
                sitl = subprocess.Popen([str(binary), "-w", "--model", "JSON", "--speedup", "1",
                    "--defaults", f"{defaults / 'copter.parm'},{defaults / 'gazebo-iris.parm'}",
                    "--home", home, "--serial0", "udpclient:127.0.0.1:14550",
                    "--serial1", "tcp:5762"], cwd=session, stdout=ap_log, stderr=subprocess.STDOUT,
                    start_new_session=True)
                children.append(sitl)
                wait_tcp(5762, sitl)
                if sys.platform == "darwin" and not args.no_ground_station:
                    ground_station = Path(scfg.DESKTOP_QGC_APP)
                    if ground_station.is_dir():
                        subprocess.run(["open", str(ground_station)], check=False)
                        print("QGroundControl: UDP 14550 auto-connect; select Satellite map in Fly view.", flush=True)
                    else:
                        print("QGroundControl not installed; Gazebo flight continues.", flush=True)
                # Allow GPS/EKF to settle; runner retains home, arming and mission checks.
                runner_args = [sys.executable, "-u", str(sim / "sitl_mission_runner.py"),
                               str(mission), "--vehicle-model", args.vehicle_model]
                if args.wait_for_start:
                    runner_args.extend(["--start-file", str(start_file)])
                runner = subprocess.Popen(runner_args,
                                          cwd=sim, start_new_session=True)
                children.append(runner)
                while runner.poll() is None:
                    if gazebo.poll() is not None:
                        return 0
                    if sitl.poll() is not None:
                        raise RuntimeError("ArduPilot exited; inspect ardupilot.log")
                    time.sleep(1)
                if runner.returncode:
                    print(f"Mission stopped (exit {runner.returncode}); scene remains open. Inspect run logs.", flush=True)
                else:
                    print("Delivery finished. Scene remains open; close window to stop.", flush=True)
            return gazebo.wait()
    except KeyboardInterrupt:
        return 130
    finally:
        for process in reversed(children):
            stop(process)


if __name__ == "__main__":
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGINT, interrupt)
    try:
        sys.exit(main())
    except (RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"Simulation launch failed: {exc}", file=sys.stderr)
        sys.exit(1)
