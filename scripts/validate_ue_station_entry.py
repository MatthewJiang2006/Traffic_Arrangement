#!/usr/bin/env python3
"""Slow single-truck entry trial for a surveyed warehouse or waiting berth."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import airsim

from ue_dynamic_crossing_check import absolute_ue, euclidean_xy, make_pose, xyz
from ue_dynamic_turn_check import project, at_station, WHEELBASE_M
from ue_short_cargo_driving_check import LIVE, stop_monitor
from ue_trailer_driving_check import ue_script

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from logistics_park.runtime.ground_contact import supported_ramp_contact
OUT = ROOT / "runs/ue_scene/station_siting"
NAME = "Car_0_0"


def drive(client, origin, path, report, spawn_y_offset_m=0, launch_throttle=.28,
          teleport_start=True, reverse=False, vehicle_name=NAME, start_monitor=False,
          stop_margin_m=.65, finish_tolerance_m=.85, sample_hook=None,
          max_duration_s=None, max_speed_m_s=1.0,turn_speed_m_s=1.5):
    name = vehicle_name
    start = path[0]
    if teleport_start:
        old_local = xyz(client.simGetVehiclePose(vehicle_name=name))
        old_world = xyz(client.simGetObjectPose(name))
        target = [start[0]-origin[0], start[1]+spawn_y_offset_m-origin[1], origin[2]-.894]
        local = [old_local[i]+target[i]-old_world[i] for i in range(3)]
        q = airsim.to_quaternion(0, 0, math.radians(start[2]))
        client.simPause(True)
        try:
            client.setCarControls(airsim.CarControls(brake=1), vehicle_name=name)
            client.simSetVehiclePose(make_pose(local, [q.x_val, q.y_val, q.z_val, q.w_val]),
                                     True, vehicle_name=name)
        finally:
            client.simPause(False)
    time.sleep(.5)
    observed, _ = absolute_ue(client, name, origin)
    report["actual_start_ue_m"] = observed
    if teleport_start or start_monitor:
        LIVE.unlink(missing_ok=True)
        client.simRunConsoleCommand("py import builtins; builtins.short_cargo_monitor_request = "
                                    + repr({"vehicle": name}))
        ue_script(client, "monitor_short_cargo.py", LIVE)
    # Some AirSim resets leave the pawn in neutral (gear 0, zero RPM). Select
    # first gear while braked; subsequent automatic controls retain drive.
    client.setCarControls(airsim.CarControls(
        brake=1, is_manual_gear=True, manual_gear=-1 if reverse else 1,
        gear_immediate=True),
        vehicle_name=name)
    baseline = client.simGetCollisionInfo(vehicle_name=name).time_stamp
    polyline = [p[:2] for p in path]
    total = sum(math.dist(a, b) for a, b in zip(polyline, polyline[1:]))
    best = 0
    last_progress = begin = time.monotonic()
    duration = max_duration_s if max_duration_s is not None else (150 if reverse else 80)
    while time.monotonic()-begin < duration:
        pos, pose = absolute_ue(client, name, origin)
        speed = abs(client.getCarState(vehicle_name=name).speed)
        yaw = airsim.to_eularian_angles(pose.orientation)[2]
        motion_yaw = (yaw+math.pi) if reverse else yaw
        follower = (pos[:2] if reverse else
                    [pos[0]-.5*WHEELBASE_M*math.cos(yaw),
                     pos[1]-.5*WHEELBASE_M*math.sin(yaw)])
        error, _ = project(pos[:2], polyline)
        _, station = project(follower, polyline)
        target = at_station(polyline, min(total, station+(1.6 if reverse else 2.5)))
        alpha = (math.atan2(target[1]-follower[1], target[0]-follower[0])-motion_yaw+math.pi)%(2*math.pi)-math.pi
        delta = math.atan2(2*WHEELBASE_M*math.sin(alpha), max(.5, euclidean_xy(target, follower)))
        steer_limit = 1.0 if reverse else .6
        steer = max(-steer_limit, min(steer_limit,
                                    (-1 if reverse else 1)*delta/math.radians(40)))
        collision = client.simGetCollisionInfo(vehicle_name=name)
        try:
            audit = json.loads(LIVE.read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            audit = {}
        row = {"t_s": round(time.monotonic()-begin, 2), "ue_m": pos,
               "speed_m_s": speed, "gear": client.getCarState(vehicle_name=name).gear,
               "path_error_m": error,
               "station_m": station, "yaw_deg": math.degrees(yaw),
               "collision": collision.object_name if collision.has_collided and collision.time_stamp != baseline else None,
               "collision_normal": [collision.normal.x_val, collision.normal.y_val,
                                    collision.normal.z_val],
               "collision_impact_ned_m": [collision.impact_point.x_val,
                                           collision.impact_point.y_val, collision.impact_point.z_val],
               "collision_penetration_m": collision.penetration_depth,
               "wheel_surfaces": [w.get("road_actor") for w in audit.get("wheels", [])],
               "wheel_clearances_m": [w.get("clearance_m") for w in audit.get("wheels", [])],
               "body_hits": [b.get("static_sweep_hit") for b in audit.get("bodies", [])],
               "parked_vehicle_overlaps": [b.get("parked_vehicle_overlap")
                                            for b in audit.get("bodies", [])]}
        report["samples"].append(row)
        failures = []
        # Tire support on surveyed roads/ramps can trigger AirSim's collision
        # flag. Require vertical contact, wheel support and clear body sweeps.
        support_surfaces = {
            "TrafficW1RoadToCurb", "TrafficW1CurbToHardstand",
            "TrafficW2RoadToCurb", "TrafficW2CurbToHardstand",
            "TrafficW1LaneTaperLowY", "TrafficW1LaneTaperHighY",
            "TrafficW2LaneTaperLowY", "TrafficW2LaneTaperHighY",
            "TrafficW5RoadToCurb", "TrafficW5CurbToHardstand",
            "TrafficW6RoadToCurb", "TrafficW6CurbToHardstand",
            "TrafficW1UnloadingRoadToCurb", "TrafficW1UnloadingCurbToHardstand",
            "TrafficW2UnloadingRoadToCurb", "TrafficW2UnloadingCurbToHardstand",
            "Traffic_SeamBridge_Middle", "Traffic_SeamBridge_Wide",
            "TrafficStationQ3CurbRamp", "TrafficStationQ4Q5CurbRamp",
            "TrafficStationQ6CurbRamp", "TrafficStationQ7Q8CurbRamp",
            "TrafficWaitingQ9Q10RoadToCurb", "TrafficWaitingQ9Q10CurbToHardstand",
            "TrafficWaitingQ11Q12RoadToCurb", "TrafficWaitingQ11Q12CurbToHardstand",
            "SM_Road_62", "SM_Road_63", "SM_Road_41", "SM_Road_51"}
        # Only road/floor actors actually observed beneath the tires qualify;
        # vertical normal and clear body sweeps remain mandatory.
        support_surfaces.update(s for s in row["wheel_surfaces"]
                                if s and s.startswith(("SM_Road_", "SM_Floor_")))
        ramp_wheel_contact = supported_ramp_contact(row, support_surfaces, clearance_m=None)
        if ramp_wheel_contact:
            row["collision_classification"] = "ground_surface_contact"
            baseline = collision.time_stamp
        if row["collision"] and not ramp_wheel_contact:
            failures.append("native collision: "+row["collision"])
        if error > 1.1:
            failures.append("path deviation > 1.1 m")
        if not .5 < pos[2] < 1.55:
            failures.append("chassis height outside surveyed hardstand")
        if audit.get("error") or not audit or time.monotonic()-audit.get("monotonic_s", 0) > 3:
            failures.append("body monitor stale or failed")
        if any(row["body_hits"]):
            failures.append("body sweep hit: "+str(row["body_hits"]))
        if any(row["parked_vehicle_overlaps"]):
            failures.append("vehicle body overlap: "+str(row["parked_vehicle_overlaps"]))
        row["wheel_contact_within_12cm"] = bool(row["wheel_clearances_m"]) and all(
            c is not None and abs(c) <= .12 for c in row["wheel_clearances_m"])
        if len(row["body_hits"]) != 3:
            failures.append("incomplete body monitor")
        if station > best+.2:
            best, last_progress = station, time.monotonic()
        progress_timeout = 20 if best < .2 else 8
        if time.monotonic()-last_progress > progress_timeout:
            failures.append("no progress %s s" % progress_timeout)
        if failures:
            report["failure"] = failures
            return
        if sample_hook:
            sample_hook(row, audit)
        remaining = euclidean_xy(pos, polyline[-1])
        if remaining < finish_tolerance_m and speed < .08:
            report["status"] = "PASS"
            return
        brake = remaining < stop_margin_m + speed*speed/2.0
        speed_limit=min(max_speed_m_s,turn_speed_m_s) if abs(steer)>.12 else max_speed_m_s
        client.setCarControls(airsim.CarControls(
            throttle=0 if brake or speed >= speed_limit else launch_throttle,
            steering=steer, brake=1 if brake else .1 if speed >= speed_limit else 0,
            is_manual_gear=True, manual_gear=-1 if reverse else 1,
            gear_immediate=True),
            vehicle_name=name)
        time.sleep(.1)
    report["failure"] = ["timeout"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("site")
    parser.add_argument("--route-file", required=True)
    parser.add_argument("--spawn-y-offset-m", type=float, default=0)
    parser.add_argument("--launch-throttle", type=float, default=.28)
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--follow-route-file")
    parser.add_argument("--follow-site")
    args = parser.parse_args()
    route = next(x for x in json.loads(Path(args.route_file).read_text())["results"]
                 if x["id"] == args.site)
    accepted = "REVERSE_PATH_CANDIDATE" if args.reverse else "FORWARD_PATH_CANDIDATE"
    if route["status"] != accepted:
        raise RuntimeError("No route candidate for site")
    client = airsim.CarClient(timeout_value=5)
    if not client.ping() or client.simIsPause():
        raise RuntimeError("UE PIE must be running and unpaused")
    if any(abs(client.getCarState(vehicle_name=n).speed) > .1 for n in client.listVehicles()):
        raise RuntimeError("Fleet must be stationary")
    scene = ue_script(client, "export_scene.py", ROOT/"runs/ue_scene/scene.json")
    origin = [v/100 for v in scene["world_origin_cm"]]
    client.simRunConsoleCommand("py "+str(ROOT/"scripts/ue/prepare_dynamic_validation.py"))
    old_api = client.isApiControlEnabled(vehicle_name=NAME)
    report = {"site": args.site, "route_file": args.route_file,
              "status": "FAIL", "failure": [], "samples": [],
              "maneuver": "reverse" if args.reverse else "forward",
              "persistent_map_changed": False}
    try:
        client.reset()
        client.enableApiControl(True, vehicle_name=NAME)
        drive(client, origin, route["path_xy_heading"], report,
              args.spawn_y_offset_m, args.launch_throttle, reverse=args.reverse)
        if report["status"] == "PASS" and args.follow_route_file:
            if not args.follow_site:
                raise RuntimeError("--follow-site required with follow route")
            follow = next(x for x in json.loads(Path(args.follow_route_file).read_text())["results"]
                          if x["id"] == args.follow_site)
            report["follow"] = {"site": args.follow_site, "status": "FAIL",
                                "failure": [], "samples": []}
            drive(client, origin, follow["path_xy_heading"], report["follow"],
                  launch_throttle=args.launch_throttle, teleport_start=False)
    finally:
        client.setCarControls(airsim.CarControls(brake=1), vehicle_name=NAME)
        stop_monitor(client)
        client.reset()
        client.enableApiControl(old_api, vehicle_name=NAME)
        report["vehicle_restored"] = True
        (OUT/("dynamic_entry_"+args.site+".json")).write_text(json.dumps(report, indent=2)+"\n")
    print(args.site, report["status"], report["failure"],
          "follow", report.get("follow", {}).get("status"),
          report.get("follow", {}).get("failure"), flush=True)
    if report["status"] != "PASS" or report.get("follow", {}).get("status", "PASS") != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
