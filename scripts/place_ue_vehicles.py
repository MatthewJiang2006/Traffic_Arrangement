#!/usr/bin/env python3
"""Place idle AirSim trucks on the surveyed UE road lanes.

Requires the active IndustrialArea PIE session and the locally installed AirSim
Python client. Backs up the live poses and settings, and rolls live poses back
if placement or verification fails.
"""

import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time

import airsim


ROOT = Path(__file__).resolve().parents[1]
PLAN_PATH = ROOT / "config/traffic/industrialarea_ue_spawn_plan.json"
SURFACE_PATH = ROOT / "runs/ue_scene/spawn_surface_validation.json"
SCENE_PATH = ROOT / "runs/ue_scene/scene.json"
SETTINGS_PATH = Path.home() / "Documents/AirSim/settings.json"
OUTPUT = ROOT / "runs/ue_scene/live_placement.json"


def position(pose):
    p = pose.position
    return [p.x_val, p.y_val, p.z_val]


def orientation(pose):
    q = pose.orientation
    return [q.w_val, q.x_val, q.y_val, q.z_val]


def make_pose(coords, quaternion):
    return airsim.Pose(airsim.Vector3r(*coords),
                       airsim.Quaternionr(quaternion[1], quaternion[2], quaternion[3], quaternion[0]))


def main() -> int:
    plan = json.loads(PLAN_PATH.read_text(encoding="utf-8"))
    surfaces = json.loads(SURFACE_PATH.read_text(encoding="utf-8"))
    if not surfaces["all_road_surfaces"] or surfaces["vehicle_count"] != 24:
        raise RuntimeError("UE road surface validation must pass for all 24 vehicles")
    surveyed = {row["vehicle_id"]: row for row in surfaces["vehicles"]}
    for item in plan["targets"]:
        target = item["target_ue_position_m"]
        row = surveyed[item["vehicle_id"]]
        centre = row["samples"][1]["ue_xy_m"]
        if math.hypot(centre[0]-target[0], centre[1]-target[1]) > .002:
            raise RuntimeError(f"stale road surface survey for {item['vehicle_id']}")
        if abs(row["target_actor_z_m_for_visual_tire_contact"]-target[2]) > .03:
            raise RuntimeError(f"uncalibrated height for {item['vehicle_id']}")
    client = airsim.CarClient(ip="127.0.0.1", port=41451, timeout_value=5)
    if not client.ping():
        raise RuntimeError("AirSim is not responding")
    before_mtime = SCENE_PATH.stat().st_mtime_ns if SCENE_PATH.exists() else 0
    command = "py " + str((ROOT/"scripts/ue/export_scene.py").resolve())
    if not client.simRunConsoleCommand(command):
        raise RuntimeError("UE refused the read-only scene export")
    for _ in range(100):
        if SCENE_PATH.exists() and SCENE_PATH.stat().st_mtime_ns > before_mtime:
            break
        time.sleep(.05)
    else:
        raise RuntimeError("UE scene export did not complete")
    scene = json.loads(SCENE_PATH.read_text(encoding="utf-8"))
    if len(scene["pie_vehicles"]) != 24:
        raise RuntimeError("expected 24 live trucks")
    origin = [value/100 for value in scene["world_origin_cm"]]
    settings_bytes = SETTINGS_PATH.read_bytes()
    settings = json.loads(settings_bytes)
    if set(settings["Vehicles"]) != {item["vehicle_id"] for item in plan["targets"]}:
        raise RuntimeError("settings vehicle names do not match the plan")

    before = {}
    for item in plan["targets"]:
        name = item["vehicle_id"]
        speed = client.getCarState(vehicle_name=name).speed
        if abs(speed) > .05:
            raise RuntimeError(f"{name} is moving at {speed:.2f} m/s; placement cancelled")
        local = client.simGetVehiclePose(vehicle_name=name)
        world = client.simGetObjectPose(name)
        before[name] = {"local_position": position(local), "local_quaternion": orientation(local),
                        "world_position": position(world)}

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    backup = OUTPUT.parent/"settings_before_spawn_plan.json"
    # Keep the original pre-placement settings across later lane refinements.
    if not backup.exists():
        backup.write_bytes(settings_bytes)
    moved = []
    was_paused = client.simIsPause()
    client.simPause(True)
    try:
        for item in plan["targets"]:
            name = item["vehicle_id"]
            old = before[name]
            target = item["target_ue_position_m"]
            target_ned_xy = [target[0]-origin[0], target[1]-origin[1]]
            target_ned = target_ned_xy + [origin[2]-target[2]]
            local_xyz = [old["local_position"][i] + target_ned[i] - old["world_position"][i]
                         for i in (0, 1, 2)]
            yaw = math.radians(item["target_ue_yaw_deg"])
            q = airsim.to_quaternion(0, 0, yaw)
            new_pose = airsim.Pose(airsim.Vector3r(*local_xyz), q)
            client.simSetVehiclePose(new_pose, True, vehicle_name=name)
            moved.append(name)
        time.sleep(.3)
        observations = []
        for item in plan["targets"]:
            name = item["vehicle_id"]
            actual = client.simGetObjectPose(name)
            p = actual.position
            target = item["target_ue_position_m"]
            actual_ue = [p.x_val+origin[0], p.y_val+origin[1], origin[2]-p.z_val]
            error = math.hypot(actual_ue[0]-target[0], actual_ue[1]-target[1])
            z_error = abs(actual_ue[2]-target[2])
            yaw = math.degrees(airsim.to_eularian_angles(actual.orientation)[2])
            yaw_error = abs((yaw-item["target_ue_yaw_deg"]+180)%360-180)
            if error > .3 or z_error > .08 or yaw_error > 5:
                raise RuntimeError(f"{name} verification failed: position error {error:.3f} m, "
                                   f"vertical error {z_error:.3f} m, yaw error {yaw_error:.1f} deg")
            observations.append({"vehicle_id": name, "ue_position_m": [round(v, 3) for v in actual_ue],
                                 "position_error_m": round(error, 3), "vertical_error_m": round(z_error, 3),
                                 "yaw_error_deg": round(yaw_error, 3)})

        client.simPause(was_paused)
        if not was_paused:
            time.sleep(.6)
            for item in plan["targets"]:
                name = item["vehicle_id"]
                p = client.simGetObjectPose(name).position
                target = item["target_ue_position_m"]
                actual_ue = [p.x_val+origin[0], p.y_val+origin[1], origin[2]-p.z_val]
                error = math.hypot(actual_ue[0]-target[0], actual_ue[1]-target[1])
                z_error = abs(actual_ue[2]-target[2])
                if error > .3 or z_error > .08:
                    raise RuntimeError(f"{name} moved {error:.3f} m horizontally, {z_error:.3f} m vertically after physics resumed")

        if SETTINGS_PATH.read_bytes() != settings_bytes:
            raise RuntimeError("AirSim settings changed during placement")
        for item in plan["targets"]:
            target = item["target_ue_position_m"]
            vehicle = settings["Vehicles"][item["vehicle_id"]]
            vehicle["X"] = round(target[0]-origin[0], 4)
            vehicle["Y"] = round(target[1]-origin[1], 4)
            vehicle["Z"] = round(origin[2]-target[2], 4)
            vehicle["Yaw"] = item["target_ue_yaw_deg"]
        updated = (json.dumps(settings, indent=2, ensure_ascii=False)+"\n").encode("utf-8")
        with tempfile.NamedTemporaryFile(dir=SETTINGS_PATH.parent, prefix=".settings-", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(updated)
        os.replace(temporary, SETTINGS_PATH)
        result = {"status": "LIVE_PLACEMENT_PASS", "vehicles": len(observations),
                  "world_origin_cm": scene["world_origin_cm"],
                  "settings_path": str(SETTINGS_PATH), "settings_backup": str(backup),
                  "settings_before_sha256": hashlib.sha256(settings_bytes).hexdigest(),
                  "settings_after_sha256": hashlib.sha256(updated).hexdigest(),
                  "observations": observations}
        OUTPUT.write_text(json.dumps(result, indent=2)+"\n")
        print(f"LIVE_PLACEMENT_PASS: {len(observations)} trucks on surveyed lanes")
        return 0
    except Exception:
        client.simPause(True)
        for name in reversed(moved):
            old = before[name]
            client.simSetVehiclePose(make_pose(old["local_position"], old["local_quaternion"]),
                                     True, vehicle_name=name)
        client.simPause(was_paused)
        if SETTINGS_PATH.read_bytes() != settings_bytes:
            SETTINGS_PATH.write_bytes(settings_bytes)
        raise


if __name__ == "__main__":
    raise SystemExit(main())
