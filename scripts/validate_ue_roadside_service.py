#!/usr/bin/env python3
"""Drive one existing short-cargo truck through the approved wide-road bays."""
import argparse
import json
import math
from pathlib import Path
import time

import airsim

from ue_dynamic_crossing_check import absolute_ue, make_pose, xyz
from ue_trailer_driving_check import ue_script
from ue_short_cargo_driving_check import audit_live, stop_monitor, LIVE

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "runs/ue_scene/station_siting/roadside_dynamic.json"
NAME = "Car_0_0"
CASES = {
    "W3": {"start": [-97.89, 54.0], "stops": [[-98, 66], [-98, 80]], "exit": [-98, 88]},
    "W4": {"start": [-97.89, 260.0], "stops": [[-98, 274], [-98, 288]], "exit": [-98, 298]},
}
STRAIGHT_LIMIT = json.loads((ROOT/"config/runtime/ue_speed_limits.json").read_text())["straight_m_s"]


def pose_at(client, origin, x, y, yaw_deg=90, z=.894):
    local = xyz(client.simGetVehiclePose(vehicle_name=NAME))
    world = xyz(client.simGetObjectPose(NAME))
    target = [x-origin[0], y-origin[1], origin[2]-z]
    converted = [local[i]+target[i]-world[i] for i in range(3)]
    q = airsim.to_quaternion(0, 0, math.radians(yaw_deg))
    client.simPause(True)
    try:
        client.setCarControls(airsim.CarControls(brake=1), vehicle_name=NAME)
        client.simSetVehiclePose(make_pose(converted, [q.x_val, q.y_val, q.z_val, q.w_val]),
                                 True, vehicle_name=NAME)
    finally:
        client.simPause(False)


def drive_to(client, origin, target, baseline, samples, timeout=45):
    begin = time.monotonic()
    last_advance = begin
    best_y = -1e6
    stopped = False
    while time.monotonic()-begin < timeout:
        pos, pose = absolute_ue(client, NAME, origin)
        speed = abs(client.getCarState(vehicle_name=NAME).speed)
        yaw = airsim.to_eularian_angles(pose.orientation)[2]
        remaining = target[1]-pos[1]
        if pos[1] > best_y+.15:
            best_y, last_advance = pos[1], time.monotonic()
        row, failures = audit_live()
        collision = client.simGetCollisionInfo(vehicle_name=NAME)
        if collision.has_collided and collision.time_stamp != baseline:
            failures.append("native collision: "+collision.object_name)
        if abs(pos[0]-target[0]) > .9 or not .7 < pos[2] < 1.4:
            failures.append("lane/height deviation")
        if time.monotonic()-last_advance > 10 and remaining > .8:
            failures.append("stalled")
        samples.append({"target": target, "ue_m": pos, "speed_m_s": speed,
                        "yaw_deg": math.degrees(yaw), "body_audit": row})
        if failures:
            raise RuntimeError("; ".join(failures))
        if stopped and speed < .08:
            if abs(remaining) > 1.0:
                raise RuntimeError("stop accuracy: "+str(round(remaining, 2))+" m")
            return {"target_ue_m": target, "stopped_ue_m": pos,
                    "error_m": math.hypot(pos[0]-target[0], remaining),
                    "ground_z_m": pos[2], "body_audit": row}
        if remaining < .55 + speed*speed/5:
            stopped = True
            controls = airsim.CarControls(brake=1)
        else:
            aim_x = target[0]
            alpha = (math.atan2(target[1]-pos[1]+2.7, aim_x-pos[0])-yaw+math.pi)%(2*math.pi)-math.pi
            steer = max(-.3, min(.3, math.atan2(2*2.3285*math.sin(alpha), 2.7)/math.radians(40)))
            target_speed = 1.25 if remaining < 4 else STRAIGHT_LIMIT
            controls = airsim.CarControls(throttle=.55 if speed < target_speed else 0,
                                          steering=steer, brake=0 if speed < target_speed else .12)
        client.setCarControls(controls, vehicle_name=NAME)
        time.sleep(.1)
    raise RuntimeError("drive timeout")


def run_case(client, origin, name):
    data = CASES[name]
    result = {"id": name, "start_ue_m": data["start"], "stops": [],
              "samples": [], "status": "FAIL", "failure": None}
    client.reset()
    client.enableApiControl(True, vehicle_name=NAME)
    pose_at(client, origin, *data["start"])
    time.sleep(.5)
    LIVE.unlink(missing_ok=True)
    ue_script(client, "monitor_short_cargo.py", LIVE)
    baseline = client.simGetCollisionInfo(vehicle_name=NAME).time_stamp
    try:
        for target in data["stops"]:
            stopped = drive_to(client, origin, target, baseline, result["samples"])
            result["stops"].append(stopped)
            time.sleep(.5)
        result["exit"] = drive_to(client, origin, data["exit"], baseline, result["samples"])
        result["status"] = "PASS"
    except Exception as exc:
        result["failure"] = str(exc)
    finally:
        client.setCarControls(airsim.CarControls(brake=1), vehicle_name=NAME)
        stop_monitor(client)
        client.reset()
        time.sleep(.5)
        OUT.parent.mkdir(parents=True, exist_ok=True)
        (OUT.parent / ("roadside_dynamic_" + name + ".json")).write_text(
            json.dumps(result, indent=2) + "\n")
    print(name, result["status"], result["failure"] or "", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=("W3", "W4", "all"), default="all")
    args = parser.parse_args()
    client = airsim.CarClient(timeout_value=5)
    if not client.ping() or client.simIsPause():
        raise RuntimeError("UE PIE must be running and unpaused")
    if any(abs(client.getCarState(vehicle_name=n).speed) > .1 for n in client.listVehicles()):
        raise RuntimeError("fleet must be stationary")
    scene = ue_script(client, "export_scene.py", ROOT/"runs/ue_scene/scene.json")
    origin = [v/100 for v in scene["world_origin_cm"]]
    client.simRunConsoleCommand("py "+str(ROOT/"scripts/ue/prepare_dynamic_validation.py"))
    old_api = client.isApiControlEnabled(vehicle_name=NAME)
    result = {"vehicle": NAME, "cases": [], "world_origin_m": origin,
              "scope": "wide-road loading/unloading stop-and-go with short cargo"}
    try:
        for name in (("W3", "W4") if args.case == "all" else (args.case,)):
            result["cases"].append(run_case(client, origin, name))
    finally:
        client.setCarControls(airsim.CarControls(brake=1), vehicle_name=NAME)
        client.reset()
        client.enableApiControl(old_api, vehicle_name=NAME)
        result["fleet_restored"] = True
        OUT.parent.mkdir(parents=True, exist_ok=True)
        OUT.write_text(json.dumps(result, indent=2)+"\n")
    if any(r["status"] != "PASS" for r in result["cases"]):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
