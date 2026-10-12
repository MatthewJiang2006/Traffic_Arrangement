#!/usr/bin/env python3
"""Drive one AirSim truck across each joined IndustrialArea road, then restore it."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import airsim


ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT/"config/traffic/industrialarea_ue_spawn_plan.json"
SCENE = ROOT/"runs/ue_scene/scene.json"
SETTINGS = Path.home()/"Documents/AirSim/settings.json"
OUTPUT = ROOT/"runs/ue_scene/dynamic_crossing_validation.json"
VEHICLE = "Car_0_0"


def xyz(pose):
    p = pose.position
    return [p.x_val, p.y_val, p.z_val]


def quaternion(pose):
    q = pose.orientation
    return [q.x_val, q.y_val, q.z_val, q.w_val]


def make_pose(position, orientation):
    return airsim.Pose(airsim.Vector3r(*position), airsim.Quaternionr(*orientation))


def euclidean_xy(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def absolute_ue(client, name, origin):
    pose = client.simGetObjectPose(name)
    p = pose.position
    return [p.x_val+origin[0], p.y_val+origin[1], origin[2]-p.z_val], pose


def run_corridor(client, name, label, x, origin, original_local, original_world, original_controls, start_z):
    start = [x, 153.0, start_z]
    target_y = 233.0  # Fully clear the upper junction, including the truck tail.
    initial_collision = client.simGetCollisionInfo(vehicle_name=name).time_stamp
    target_ned = [start[0]-origin[0], start[1]-origin[1], origin[2]-start[2]]
    local_target = [original_local["position"][i]+target_ned[i]-original_world[i] for i in range(3)]
    samples = []
    result = {"corridor": label, "vehicle_id": name, "target_lane_x_m": x,
              "start_ue_m": start, "goal_y_m": target_y, "samples": samples}
    client.simPause(True)
    try:
        client.setCarControls(airsim.CarControls(throttle=0, brake=1), vehicle_name=name)
        target_orientation = airsim.to_quaternion(0, 0, math.pi/2)
        client.simSetVehiclePose(make_pose(local_target, [target_orientation.x_val, target_orientation.y_val,
                                                          target_orientation.z_val, target_orientation.w_val]),
                                 True, vehicle_name=name)
    finally:
        client.simPause(False)
    reached = False
    try:
        time.sleep(.3)
        pos, _ = absolute_ue(client, name, origin)
        if euclidean_xy(pos, start) > .15:
            raise RuntimeError(f"{label} start teleport deviation {euclidean_xy(pos, start):.2f} m")
        print(f"{label}: start x={pos[0]:.2f}, y={pos[1]:.2f}", flush=True)
        start_time = time.monotonic()
        last_print = start_time
        while time.monotonic()-start_time < 240:
            now = time.monotonic()
            pos, pose = absolute_ue(client, name, origin)
            speed = client.getCarState(vehicle_name=name).speed
            yaw = math.degrees(airsim.to_eularian_angles(pose.orientation)[2])
            collision = client.simGetCollisionInfo(vehicle_name=name)
            fresh_collision = bool(collision.has_collided and collision.time_stamp != initial_collision)
            sample = {"t_s": round(now-start_time, 3), "ue_position_m": [round(v, 4) for v in pos],
                      "speed_m_s": round(speed, 4), "yaw_deg": round(yaw, 3),
                      "lateral_error_m": round(pos[0]-x, 4),
                      "fresh_collision": fresh_collision,
                      "collision_object": collision.object_name if fresh_collision else None}
            samples.append(sample)
            if now-last_print >= 5:
                print(f"{label}: y={pos[1]:.1f}, speed={speed:.2f}, dx={pos[0]-x:.2f}, z={pos[2]:.2f}", flush=True)
                last_print = now
            if fresh_collision:
                result["failure"] = "new collision with "+str(collision.object_name)
                break
            if abs(pos[0]-x) > 1.5 or abs((yaw-90+180)%360-180) > 20:
                result["failure"] = "lane or heading deviation"
                break
            if pos[2] < .75 or pos[2] > 1.6:
                result["failure"] = "abnormal chassis height"
                break
            if pos[1] >= target_y:
                reached = True
                break
            # Limit speed without steering the truck away from its measured lane.
            if abs(speed) < 3.0:
                controls = airsim.CarControls(throttle=.45, steering=0, brake=0)
            else:
                controls = airsim.CarControls(throttle=0, steering=0, brake=.15)
            client.setCarControls(controls, vehicle_name=name)
            time.sleep(.12)
        else:
            result["failure"] = "timeout before upper region"
    finally:
        client.setCarControls(airsim.CarControls(throttle=0, steering=0, brake=1), vehicle_name=name)
        # A teleport preserves PhysX velocity. Reset restores the configured
        # fleet poses and physics state together, preventing post-test drift.
        client.reset()
        time.sleep(.8)
        client.enableApiControl(True, vehicle_name=name)
        restored, _ = absolute_ue(client, name, origin)
        expected = [original_world[0]+origin[0], original_world[1]+origin[1]]
        result["restore_xy_error_m"] = round(euclidean_xy(restored, expected), 4)
        if result["restore_xy_error_m"] > .1:
            raise RuntimeError("truck failed to return to configured spawn after reset")
    result["reached_upper_region"] = reached
    result["sample_count"] = len(samples)
    result["max_lateral_error_m"] = max(abs(v["lateral_error_m"]) for v in samples)
    result["min_chassis_z_m"] = min(v["ue_position_m"][2] for v in samples)
    result["max_chassis_z_m"] = max(v["ue_position_m"][2] for v in samples)
    result["max_speed_m_s"] = max(abs(v["speed_m_s"]) for v in samples)
    result["status"] = "PASS" if reached and not result.get("failure") else "FAIL"
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--corridor", choices=("left", "middle", "both"), default="both")
    args = parser.parse_args()
    client = airsim.CarClient(ip="127.0.0.1", port=41451, timeout_value=5)
    if not client.ping() or client.simIsPause():
        raise RuntimeError("AirSim PIE must be running and unpaused")
    scene = json.loads(SCENE.read_text(encoding="utf-8"))
    origin = [v/100 for v in scene["world_origin_cm"]]
    settings_hash = hashlib.sha256(SETTINGS.read_bytes()).hexdigest()
    plan = json.loads(PLAN.read_text(encoding="utf-8"))
    home = next(item for item in plan["targets"] if item["vehicle_id"] == VEHICLE)["target_ue_position_m"]
    if any(abs(client.getCarState(vehicle_name=item["vehicle_id"]).speed) > .1 for item in plan["targets"]):
        raise RuntimeError("another truck is already moving")
    home_now, _ = absolute_ue(client, VEHICLE, origin)
    if euclidean_xy(home_now, home) > .2:
        raise RuntimeError("test truck is not at its planned spawn")
    old_pose = client.simGetVehiclePose(vehicle_name=VEHICLE)
    original_local = {"position": xyz(old_pose), "orientation": quaternion(old_pose)}
    original_world = xyz(client.simGetObjectPose(VEHICLE))
    original_controls = client.getCarControls(vehicle_name=VEHICLE)
    original_api = client.isApiControlEnabled(vehicle_name=VEHICLE)
    client.enableApiControl(True, vehicle_name=VEHICLE)
    labels = ("left", "middle") if args.corridor == "both" else (args.corridor,)
    positions = {"left": -92.1936-1.9, "middle": -10.204-1.9}
    result = {"map_path": scene["map_path"], "world_origin_cm": scene["world_origin_cm"],
              "settings_sha256": settings_hash, "vehicle_id": VEHICLE, "corridors": []}
    try:
        for label in labels:
            row = run_corridor(client, VEHICLE, label, positions[label], origin, original_local,
                               original_world, original_controls, home_now[2])
            result["corridors"].append(row)
            print(f"{label}: {row['status']}, {row['sample_count']} samples, "
                  f"max lateral error {row['max_lateral_error_m']:.2f} m", flush=True)
            if row["status"] != "PASS":
                break
    finally:
        client.enableApiControl(original_api, vehicle_name=VEHICLE)
        if hashlib.sha256(SETTINGS.read_bytes()).hexdigest() != settings_hash:
            raise RuntimeError("AirSim settings changed during dynamic test")
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        result["status"] = "PASS" if len(result["corridors"]) == len(labels) and all(
            row["status"] == "PASS" for row in result["corridors"]) else "FAIL"
        OUTPUT.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(result["status"], "report", OUTPUT, flush=True)
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
