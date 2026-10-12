#!/usr/bin/env python3
"""Check right-side-lane driving, rigid-body clearance and braking in UE."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import time
import airsim

from ue_dynamic_crossing_check import absolute_ue, euclidean_xy, make_pose, xyz
from ue_dynamic_turn_check import project, at_station, WHEELBASE_M
from ue_trailer_driving_check import clear_nearby, ue_script

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/"runs/ue_scene/short_cargo_driving"
LIVE = ROOT/"runs/ue_scene/short_cargo_live.json"
HISTORY = ROOT/"runs/ue_scene/short_cargo_history.jsonl"
NAME = "Car_0_0"
SETTINGS = Path.home()/"Documents/AirSim/settings.json"
WATCH_SECONDS = 0


def stop_monitor(client):
    client.simRunConsoleCommand("py "+str(ROOT/"scripts/ue/stop_short_cargo_monitor.py"))


def audit_live(path=LIVE):
    row = json.loads(path.read_text())
    if row.get("error"):
        return row, ["monitor error: "+row["error"]]
    errors = []
    if time.monotonic()-row["monotonic_s"] > 2:
        errors.append("stale body audit")
    for body in row["bodies"]:
        if body["static_sweep_hit"]:
            errors.append(body["name"]+" swept into "+body["static_sweep_hit"])
        if body["parked_vehicle_overlap"]:
            errors.append(body["name"]+" overlaps "+body["parked_vehicle_overlap"])
    for wheel in row["wheels"]:
        road = wheel["road_actor"] or ""
        if not road.startswith(("SM_Road_", "Traffic_SeamBridge")):
            errors.append(wheel["name"]+" off road: "+road)
        if wheel["clearance_m"] is None or abs(wheel["clearance_m"]) > .08:
            errors.append(wheel["name"]+" tire clearance outside 8 cm")
    if abs(row["pitch_deg"]) > 8 or abs(row["roll_deg"]) > 8:
        errors.append("abnormal chassis attitude")
    return row, errors


def run_case(client, label, path, origin):
    result = {"case": label, "path_ue_m": path, "samples": [], "failure": [],
              "path_mode": "surveyed right-side lanes; normal RoadGraph turn",
              "lane_side": "RIGHT" if label in ("right", "lane_change") or label.endswith("_right") else "SINGLE" if label == "cross_middle" else "LEFT"}
    if label == "lane_change":
        result["initial_lane"] = "LEFT"
        result["target_lane"] = "RIGHT"
        result["path_mode"] = "diagnostic 25 m smooth lane change on west two-lane road; no production lane-change edge"
    reached = False
    try:
        client.reset()
        result["cleared_nearby_vehicles"], _ = clear_nearby(client, origin)
        client.enableApiControl(True, vehicle_name=NAME)
        old_local = xyz(client.simGetVehiclePose(vehicle_name=NAME))
        old_world = xyz(client.simGetObjectPose(NAME))
        start = [path[0][0], 153 if label.startswith("cross") or label == "lane_change" else 239 if label == "right" else 198, .894]
        target = [start[0]-origin[0], start[1]-origin[1], origin[2]-start[2]]
        local = [old_local[i]+target[i]-old_world[i] for i in range(3)]
        q = airsim.to_quaternion(0, 0, -math.pi/2 if label == "right" else math.pi/2)
        client.simPause(True)
        try:
            client.setCarControls(airsim.CarControls(brake=1), vehicle_name=NAME)
            client.simSetVehiclePose(make_pose(local, [q.x_val, q.y_val, q.z_val, q.w_val]), True, vehicle_name=NAME)
        finally:
            client.simPause(False)
        time.sleep(.4)
        HISTORY.unlink(missing_ok=True)
        ue_script(client, "monitor_short_cargo.py", LIVE)
        baseline = client.simGetCollisionInfo(vehicle_name=NAME).time_stamp
        if WATCH_SECONDS:
            print(f"{label}: camera ready; starting in {WATCH_SECONDS:g} seconds", flush=True)
            time.sleep(WATCH_SECONDS)
        total = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        begin = time.monotonic()
        last_print = begin
        last_progress = begin
        best_station = 0
        while time.monotonic()-begin < 150:
            pos, pose = absolute_ue(client, NAME, origin)
            state = client.getCarState(vehicle_name=NAME)
            yaw = airsim.to_eularian_angles(pose.orientation)[2]
            rear = [pos[0]-.5*WHEELBASE_M*math.cos(yaw), pos[1]-.5*WHEELBASE_M*math.sin(yaw)]
            error, _ = project(pos[:2], path)
            _, station = project(rear, path)
            lookahead = max(2.7, abs(state.speed)*1.3)
            target = at_station(path, station+lookahead)
            alpha = (math.atan2(target[1]-rear[1], target[0]-rear[0])-yaw+math.pi)%(2*math.pi)-math.pi
            delta = math.atan2(2*WHEELBASE_M*math.sin(alpha), max(.5, euclidean_xy(target, rear)))
            steering = max(-1, min(1, delta/math.radians(40)))
            collision = client.simGetCollisionInfo(vehicle_name=NAME)
            fresh = bool(collision.has_collided and collision.time_stamp != baseline)
            audit, failures = audit_live()
            now = time.monotonic()
            result["samples"].append({"t_s": now-begin, "ue_position_m": pos, "speed_m_s": state.speed,
                                      "path_error_m": error, "yaw_deg": math.degrees(yaw),
                                      "steering": steering, "body_audit": audit})
            if fresh:
                failures.append("native chassis collision: "+collision.object_name)
            if error > .75 or not .7 < pos[2] < 1.4:
                failures.append("lane or chassis height deviation")
            if station > best_station+.5:
                best_station, last_progress = station, now
            if now-last_progress > 15:
                failures.append("no progress for 15 seconds")
            if failures:
                result["failure"] = failures
                break
            if now-last_print >= 5:
                print(f"{label}: x={pos[0]:.1f}, y={pos[1]:.1f}, speed={state.speed:.2f}, error={error:.2f}", flush=True)
                last_print = now
            if station >= total-1.2 and euclidean_xy(pos, path[-1]) < 2:
                reached = True
                break
            target_speed = 1.5 if abs(steering) > .15 else 2.5
            below = abs(state.speed) < target_speed
            client.setCarControls(airsim.CarControls(throttle=.45 if below else 0, steering=steering,
                                                   brake=0 if below else .12), vehicle_name=NAME)
            time.sleep(.1)
        else:
            result["failure"].append("timeout")
        if reached:
            brake_start = time.monotonic()
            client.setCarControls(airsim.CarControls(brake=1), vehicle_name=NAME)
            stopping_audits = []
            while time.monotonic()-brake_start < 3:
                audit, failures = audit_live()
                stopping_audits.append(audit)
                result["failure"].extend(failures)
                if abs(client.getCarState(vehicle_name=NAME).speed) < .05:
                    break
                time.sleep(.1)
            stopped, _ = absolute_ue(client, NAME, origin)
            time.sleep(.6)
            held, _ = absolute_ue(client, NAME, origin)
            final_audit, failures = audit_live()
            result["failure"].extend(failures)
            result["stopping"] = {"seconds_to_stop": time.monotonic()-brake_start-.6,
                "final_speed_m_s": client.getCarState(vehicle_name=NAME).speed,
                "hold_drift_m": euclidean_xy(stopped, held), "audits": stopping_audits+[final_audit]}
            if abs(result["stopping"]["final_speed_m_s"]) > .05 or result["stopping"]["hold_drift_m"] > .05:
                result["failure"].append("brake/hold acceptance failed")
            if WATCH_SECONDS:
                print(f"{label}: stopped at goal; holding view for {WATCH_SECONDS:g} seconds", flush=True)
                time.sleep(WATCH_SECONDS)
    except Exception as e:
        result["failure"].append(repr(e))
    finally:
        result["reached_goal"] = reached
        result["max_path_error_m"] = max((s["path_error_m"] for s in result["samples"]), default=None)
        if result["samples"]:
            start_yaw = result["samples"][0]["yaw_deg"]
            end_yaw = result["samples"][-1]["yaw_deg"]
            change = (end_yaw-start_yaw+180)%360-180
            result["heading_change_deg"] = change
            expected = 90 if label == "right" else -90 if label == "left" else 0
            if abs(change-expected) > 10:
                result["failure"].append("heading change disagrees with vehicle-relative maneuver")
        result["status"] = "PASS" if reached and not result["failure"] else "FAIL"
        (OUT/(label+".json")).write_text(json.dumps(result, indent=2)+"\n")
        stop_monitor(client)
        if HISTORY.exists():
            shutil.copy2(HISTORY, OUT/(label+"_telemetry.jsonl"))
        client.setCarControls(airsim.CarControls(brake=1), vehicle_name=NAME)
        client.reset()
        time.sleep(.5)
    print(label, result["status"], result["failure"], flush=True)
    return result


def main():
    global OUT, WATCH_SECONDS
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=("all", "cross_left", "cross_middle", "cross_left_right", "right", "left", "lane_change"), default="all")
    parser.add_argument("--output", type=Path, default=OUT)
    parser.add_argument("--watch-seconds", type=float, default=0)
    args = parser.parse_args()
    OUT = args.output.resolve()
    WATCH_SECONDS = max(0, min(30, args.watch_seconds))
    OUT.mkdir(parents=True, exist_ok=True)
    client = airsim.CarClient(timeout_value=5)
    if not client.ping() or client.simIsPause():
        raise RuntimeError("PIE must be running and unpaused")
    if any(abs(client.getCarState(vehicle_name=n).speed) > .1 for n in client.listVehicles()):
        raise RuntimeError("fleet must be stationary")
    scene = ue_script(client, "export_scene.py", ROOT/"runs/ue_scene/scene.json")
    origin = [v/100 for v in scene["world_origin_cm"]]
    client.simRunConsoleCommand("py "+str(ROOT/"scripts/ue/prepare_dynamic_validation.py"))
    graph = json.loads((ROOT/"config/topology/industrialarea_ue.json").read_text())
    edges = {e["edge_id"]: e for e in graph["edges"]}
    survey = json.loads((ROOT/"config/topology/industrialarea_ue_survey.json").read_text())
    lane = survey["lane_offset_m"]
    lane_x = [survey["junctions"][f"j_2_{col}"]["center_m"][0]-lane for col in (0, 1)]
    paths = {"cross_left": [[lane_x[0], 150], [lane_x[0], 233]],
             "cross_middle": [[lane_x[1], 150], [lane_x[1], 233]],
             "cross_left_right": [[lane_x[0]-2*lane, 150], [lane_x[0]-2*lane, 233]]}
    # Diagnostic lane change on the actual two-lane west road, away from
    # junctions. This is not a production routing lane-change edge.
    x0 = lane_x[0]
    transition = []
    for i in range(31):
        t = i/30
        shift = 10*t**3-15*t**4+6*t**5
        transition.append([x0-2*lane*shift, 160+25*t])
    paths["lane_change"] = [[x0, 150]]+transition+[[x0-2*lane, 198]]
    # UE positive yaw is RIGHT. Here -Y -> +X is RIGHT; +Y -> +X is LEFT.
    for label, outgoing, endx in (("right", "W", -55), ("left", "E", 25)):
        junction = "j_2_0"
        side = label
        arm = "N" if label == "right" else "S"
        outgoing = "E" if label == "right" else outgoing
        connector = edges[f"turn_{junction}_{arm}_{outgoing}_{side}_single"]["centerline"]
        if label == "right":
            paths[label] = [[connector[0][0], 245]]+connector+[[-55, connector[-1][1]]]
            continue
        paths[label] = [[connector[0][0], 190]]+connector+[[-55, connector[-1][1]]]
    if args.case != "all":
        paths = {args.case: paths[args.case]}
    original_api = client.isApiControlEnabled(vehicle_name=NAME)
    settings_hash = hashlib.sha256(SETTINGS.read_bytes()).hexdigest()
    home = {n: absolute_ue(client, n, origin)[0] for n in client.listVehicles()}
    report = {"vehicle": NAME, "cargo": "SM_Truck_Trailer_1", "cases": [],
              "scope": "selected single-vehicle straight/turn/lane-change diagnostics with braking",
              "requested_cases": list(paths),
              "collision_audit": "cab/chassis/cargo conservative boxes, static swept queries and parked-body SAT",
              "right_hand_traffic": True, "ground_truth_diagnostic": True,
              "coordinate_convention": "UE +X forward, +Y right; positive yaw RIGHT; arrows point opposite decal right vector"}
    try:
        for label, path in paths.items():
            report["cases"].append(run_case(client, label, path, origin))
    finally:
        stop_monitor(client)
        client.reset()
        time.sleep(.5)
        client.enableApiControl(original_api, vehicle_name=NAME)
        report["settings_unchanged"] = hashlib.sha256(SETTINGS.read_bytes()).hexdigest() == settings_hash
        report["fleet_restored"] = all(euclidean_xy(absolute_ue(client, n, origin)[0], p)<.1 for n, p in home.items())
        report["status"] = "PASS" if len(report["cases"])==len(paths) and all(r["status"]=="PASS" for r in report["cases"]) and report["settings_unchanged"] and report["fleet_restored"] else "FAIL"
        (OUT/"acceptance.json").write_text(json.dumps(report, indent=2)+"\n")
    print("Short cargo driving", report["status"], OUT/"acceptance.json", flush=True)
    return 0 if report["status"]=="PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
