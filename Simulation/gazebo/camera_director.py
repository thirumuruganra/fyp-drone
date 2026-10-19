"""
Gazebo GUI camera for recordings: a takeoff shot from beside the depot pad, then a close follow camera.

    /usr/bin/python3 gazebo/camera_director.py --world fyp_thiru_demo --model iris_with_ardupilot --view takeoff

Run with the system python3: the Gazebo bindings (python3-gz-transport13, python3-gz-msgs10) are Debian
packages, not in the planner venv. Standalone on purpose (no project imports); launch_simulation.py passes
the offsets from sim_config.py.

takeoff  camera placed at --takeoff-pos (world frame, depot = origin), TRACK (look at) the vehicle while it
         lifts off; above --switch-alt it changes to FOLLOW at --follow-offset for the rest of the flight.
follow / wide  FOLLOW from the start.

The GUI drops tracking on Escape or when another entity is selected; the director re-asserts its mode from
/gui/currently_tracked until --parent-pid (the launcher) exits.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import threading
import time

TRACK, FOLLOW = "track", "follow"


def phase(view: str, z: float | None, switched: bool, switch_alt: float) -> str:
    """Camera phase for the current vehicle height (world z). Once switched to FOLLOW it never goes back."""
    if view != "takeoff" or switched:
        return FOLLOW
    return FOLLOW if z is not None and z >= switch_alt else TRACK


def alive(pid: int) -> bool:
    if pid <= 0:
        return True
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def wait_for_gui(parent_pid: int, settle_s: float = 5.0, timeout_s: float = 300.0) -> bool:
    """Block until the GUI serves /gui/move_to/pose, then settle. Joining gz-transport (subscribe/advertise)
    while Gazebo is still starting can crash it: its discovery thread calls getenv() while the GUI is
    still setting environment variables (segfault in NodeShared::OnSubscribers)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and alive(parent_pid):
        try:
            out = subprocess.run(["gz", "service", "-l"], capture_output=True, text=True, timeout=15).stdout
        except (OSError, subprocess.TimeoutExpired):
            out = ""
        if "/gui/move_to/pose" in out.split():
            time.sleep(settle_s)
            return True
        time.sleep(2)
    return False


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--world", required=True)
    p.add_argument("--model", required=True)
    p.add_argument("--view", choices=("takeoff", "follow", "wide"), default="takeoff")
    p.add_argument("--takeoff-pos", nargs=3, type=float, default=(-4.0, -6.0, 3.0))
    p.add_argument("--follow-offset", nargs=3, type=float, default=(-6.0, -8.0, 8.0))
    p.add_argument("--switch-alt", type=float, default=6.0)
    p.add_argument("--parent-pid", type=int, default=0)
    a = p.parse_args(argv)
    if not wait_for_gui(a.parent_pid):
        print("[Camera] Gazebo GUI not found; camera director idle.", flush=True)
        return 1

    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.cameratrack_pb2 import CameraTrack
    from gz.msgs10.gui_camera_pb2 import GUICamera
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    node = Node()
    state = {"z": None, "mode": None, "target": None, "offset": None}
    lock = threading.Lock()

    def on_pose(msg):
        for pose in msg.pose:
            if pose.name == a.model:
                with lock:
                    state["z"] = pose.position.z
                return

    def on_tracked(msg):
        off = msg.follow_offset
        with lock:
            state["mode"] = msg.track_mode
            state["target"] = (msg.follow_target.name if msg.track_mode == CameraTrack.FOLLOW
                               else msg.track_target.name)
            state["offset"] = (off.x, off.y, off.z)

    node.subscribe(Pose_V, f"/world/{a.world}/dynamic_pose/info", on_pose)
    node.subscribe(CameraTrack, "/gui/currently_tracked", on_tracked)
    pub = node.advertise("/gui/track", CameraTrack)

    def track_msg():
        m = CameraTrack()
        m.track_mode = CameraTrack.TRACK
        m.track_target.name = a.model
        m.track_pgain = 0.2
        return m

    def follow_msg():
        m = CameraTrack()
        m.track_mode = CameraTrack.FOLLOW
        m.follow_target.name = a.model
        m.follow_offset.x, m.follow_offset.y, m.follow_offset.z = a.follow_offset
        m.follow_pgain = 0.05
        return m

    def in_mode(want: str) -> bool:
        with lock:
            if state["target"] != a.model:
                return False
            if want == TRACK:
                return state["mode"] == CameraTrack.TRACK
            return (state["mode"] == CameraTrack.FOLLOW and state["offset"] is not None
                    and all(abs(x - y) < 0.01 for x, y in zip(state["offset"], a.follow_offset)))

    # The GUI may still be starting: place the takeoff camera once its move_to service answers.
    if a.view == "takeoff":
        cam = GUICamera()
        cam.pose.position.x, cam.pose.position.y, cam.pose.position.z = a.takeoff_pos
        placed = False
        while not placed and alive(a.parent_pid):
            ok, rep = node.request("/gui/move_to/pose", cam, GUICamera, Boolean, 1000)
            placed = bool(ok and rep.data)
            if not placed:
                time.sleep(0.5)
        print("[Camera] Takeoff shot beside the depot pad (TRACK).", flush=True)

    switched, announced = False, None
    while alive(a.parent_pid):
        with lock:
            z = state["z"]
        want = phase(a.view, z, switched, a.switch_alt)
        if want == FOLLOW and not switched:
            switched = True
        if not in_mode(want):
            pub.publish(follow_msg() if want == FOLLOW else track_msg())
        elif announced != want:
            print(f"[Camera] {want.upper()} locked on {a.model}"
                  + (f" offset {tuple(a.follow_offset)}" if want == FOLLOW else ""), flush=True)
            announced = want
        time.sleep(0.25)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
