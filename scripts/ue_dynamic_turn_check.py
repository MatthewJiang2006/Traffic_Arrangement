#!/usr/bin/env python3
"""Drive legal left/right RoadGraph turns with an AirSim ground-truth test controller."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import airsim

from ue_dynamic_crossing_check import absolute_ue, euclidean_xy, make_pose, quaternion, xyz


ROOT = Path(__file__).resolve().parents[1]
VEHICLE = "Car_0_0"
SETTINGS = Path.home()/"Documents/AirSim/settings.json"
OUTPUT = ROOT/"runs/ue_scene/dynamic_turn_validation.json"
WHEELBASE_M = 2.3285  # Measured SUV WheelFL/WheelBL bone separation in PIE.


def project(point, path):
    best = (float("inf"), 0.0)
    cumulative = 0.0
    for a, b in zip(path, path[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        t = max(0.0, min(1.0, ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length**2))
        distance = math.hypot(point[0]-a[0]-t*dx, point[1]-a[1]-t*dy)
        if distance < best[0]:
            best = (distance, cumulative+t*length)
        cumulative += length
    return best


def at_station(path, station):
    for a, b in zip(path, path[1:]):
        length = math.dist(a, b)
        if station <= length:
            return [a[i]+max(0.0, station)/length*(b[i]-a[i]) for i in (0, 1)]
        station -= length
    return path[-1]


def run_case(client, label, path, origin):
    home, _ = absolute_ue(client, VEHICLE, origin)
    old_local = xyz(client.simGetVehiclePose(vehicle_name=VEHICLE))
    old_world = xyz(client.simGetObjectPose(VEHICLE))
    start = [path[0][0], 198.0, home[2]]
    target_ned = [start[0]-origin[0], start[1]-origin[1], origin[2]-start[2]]
    local = [old_local[i]+target_ned[i]-old_world[i] for i in range(3)]
    heading = airsim.to_quaternion(0, 0, math.pi/2)
    result = {"case": label, "junction": "j_2_1", "movement": f"turn_j_2_1_S_{'W' if label == 'right' else 'E'}",
              "reference_path_ue_m": path, "samples": []}
    initial_collision = client.simGetCollisionInfo(vehicle_name=VEHICLE).time_stamp
    total = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    client.enableApiControl(True, vehicle_name=VEHICLE)
    client.simPause(True)
    try:
        client.setCarControls(airsim.CarControls(brake=1), vehicle_name=VEHICLE)
        client.simSetVehiclePose(make_pose(local, [heading.x_val, heading.y_val, heading.z_val, heading.w_val]),
                                 True, vehicle_name=VEHICLE)
    finally:
        client.simPause(False)
    begin = time.monotonic()
    last_print = begin
    reached = False
    try:
        time.sleep(.3)
        while time.monotonic()-begin < 240:
            pos, pose = absolute_ue(client, VEHICLE, origin)
            state = client.getCarState(vehicle_name=VEHICLE)
            yaw = airsim.to_eularian_angles(pose.orientation)[2]
            rear = [pos[0]-.5*WHEELBASE_M*math.cos(yaw), pos[1]-.5*WHEELBASE_M*math.sin(yaw)]
            error, _ = project(pos[:2], path)
            _, station = project(rear, path)
            lookahead = max(2.7, abs(state.speed)*1.3)
            target = at_station(path, station+lookahead)
            alpha = (math.atan2(target[1]-rear[1], target[0]-rear[0])-yaw+math.pi)%(2*math.pi)-math.pi
            distance = max(.5, euclidean_xy(target, rear))
            delta = math.atan2(2*WHEELBASE_M*math.sin(alpha), distance)
            steering = max(-1.0, min(1.0, delta/math.radians(40)))
            collision = client.simGetCollisionInfo(vehicle_name=VEHICLE)
            fresh_collision = bool(collision.has_collided and collision.time_stamp != initial_collision)
            now = time.monotonic()
            result["samples"].append({"t_s": round(now-begin, 3), "ue_position_m": [round(v, 4) for v in pos],
                                       "yaw_deg": round(math.degrees(yaw), 3), "speed_m_s": round(state.speed, 4),
                                       "path_error_m": round(error, 4), "rear_station_m": round(station, 4),
                                       "steering": round(steering, 4), "fresh_collision": fresh_collision,
                                       "collision_object": collision.object_name if fresh_collision else None})
            if now-last_print >= 5:
                print(f"{label}: x={pos[0]:.1f}, y={pos[1]:.1f}, yaw={math.degrees(yaw):.1f}, "
                      f"error={error:.2f}, speed={state.speed:.2f}", flush=True)
                last_print = now
            if fresh_collision:
                result["failure"] = "new collision with "+str(collision.object_name)
                break
            if error > 1.5 or pos[2] < .7 or pos[2] > 1.4:
                result["failure"] = "path or chassis height deviation"
                break
            if station >= total-1.2 and euclidean_xy(pos, path[-1]) < 2.0:
                reached = True
                break
            target_speed = 1.6 if abs(steering) > .15 else 2.0
            controls = airsim.CarControls(throttle=.45 if abs(state.speed) < target_speed else 0,
                                           steering=steering, brake=0 if abs(state.speed) < target_speed else .12)
            client.setCarControls(controls, vehicle_name=VEHICLE)
            time.sleep(.1)
        else:
            result["failure"] = "timeout"
    finally:
        client.setCarControls(airsim.CarControls(brake=1), vehicle_name=VEHICLE)
        client.reset()
        time.sleep(.8)
        restored, _ = absolute_ue(client, VEHICLE, origin)
        result["restore_xy_error_m"] = round(euclidean_xy(restored, home), 4)
        if result["restore_xy_error_m"] > .1:
            raise RuntimeError("reset did not restore test vehicle")
    result["reached_goal"] = reached
    result["max_path_error_m"] = max((v["path_error_m"] for v in result["samples"]), default=None)
    result["status"] = "PASS" if reached and not result.get("failure") else "FAIL"
    print(label, result["status"], "max error", result["max_path_error_m"], flush=True)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=("right", "left", "both"), default="both")
    args = parser.parse_args()
    client = airsim.CarClient(ip="127.0.0.1", port=41451, timeout_value=5)
    if not client.ping() or client.simIsPause():
        raise RuntimeError("AirSim must be running and unpaused")
    scene = json.loads((ROOT/"runs/ue_scene/scene.json").read_text(encoding="utf-8"))
    origin = [v/100 for v in scene["world_origin_cm"]]
    graph = json.loads((ROOT/"config/topology/industrialarea_ue.json").read_text(encoding="utf-8"))
    edges = {e["edge_id"]: e for e in graph["edges"]}
    settings_hash = hashlib.sha256(SETTINGS.read_bytes()).hexdigest()
    if any(abs(client.getCarState(vehicle_name=n).speed) > .1 for n in client.listVehicles()):
        raise RuntimeError("all trucks must be stationary before the turn check")
    cases = ("right", "left") if args.case == "both" else (args.case,)
    original_api = client.isApiControlEnabled(vehicle_name=VEHICLE)
    result = {"map_path": scene["map_path"], "vehicle_id": VEHICLE, "wheelbase_m": WHEELBASE_M,
              "controller": "Ground-truth Pure Pursuit diagnostic; not the production navigation adapter",
              "cases": []}
    try:
        for label in cases:
            connector = edges[f"turn_j_2_1_S_{'W' if label == 'right' else 'E'}_{label}_{label}"]["centerline"]
            endpoint = [-55.0, connector[-1][1]] if label == "right" else [25.0, connector[-1][1]]
            path = [[connector[0][0], 190.0]]+connector+[endpoint]
            row = run_case(client, label, path, origin)
            result["cases"].append(row)
            if row["status"] != "PASS":
                break
    finally:
        client.enableApiControl(original_api, vehicle_name=VEHICLE)
        result["settings_unchanged"] = hashlib.sha256(SETTINGS.read_bytes()).hexdigest() == settings_hash
        result["status"] = "PASS" if len(result["cases"]) == len(cases) and all(
            row["status"] == "PASS" for row in result["cases"]) and result["settings_unchanged"] else "FAIL"
        OUTPUT.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(result["status"], "report", OUTPUT, flush=True)
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
