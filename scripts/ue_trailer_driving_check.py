#!/usr/bin/env python3
"""Exercise a physical BP_Trailer3_Child on one AirSim truck and restore the fleet."""
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

ROOT = Path(__file__).resolve().parents[1]
OUTDIR = ROOT/"runs/ue_scene/trailer_trial"
LIVE = ROOT/"runs/ue_scene/trailer_trial_live.json"
HISTORY = ROOT/"runs/ue_scene/trailer_trial_history.jsonl"
NAME = "Car_0_0"
TRAILER = "Traffic_Trailer_Probe"
SETTINGS = Path.home()/"Documents/AirSim/settings.json"


def ue_script(client, script, output):
    old = output.stat().st_mtime_ns if output.exists() else 0
    if not client.simRunConsoleCommand("py "+str(ROOT/"scripts/ue"/script)):
        raise RuntimeError("UE rejected "+script)
    for _ in range(100):
        if output.exists() and output.stat().st_mtime_ns > old:
            result = json.loads(output.read_text())
            if isinstance(result, dict) and result.get("error"):
                raise RuntimeError(result["error"])
            return result
        time.sleep(.1)
    raise RuntimeError(script+" did not produce a fresh report")


def footprint_overlap(a, b):
    # SAT for oriented rectangles, including the stock trailer physics envelope.
    axes = [(math.cos(t), math.sin(t)) for t in (a[2], a[2]+math.pi/2, b[2], b[2]+math.pi/2)]
    for ax, ay in axes:
        d = abs((a[0]-b[0])*ax+(a[1]-b[1])*ay)
        radii = []
        for box in (a, b):
            c, s = math.cos(box[2]), math.sin(box[2])
            radii.append(box[3]*abs(c*ax+s*ay)+box[4]*abs(-s*ax+c*ay))
        if d >= sum(radii):
            return False
    return True


def clear_nearby(client, origin):
    """Temporarily park the twelve upper-area trucks on empty lower east roads."""
    plans = json.loads((ROOT/"config/traffic/industrialarea_ue_spawn_plan.json").read_text())["targets"]
    cleared = []
    counters = {0: 0, 180: 0}
    was_paused = client.simIsPause()
    client.simPause(True)
    try:
        for item in plans:
            if item["target_ue_position_m"][1] < 200:
                continue
            name, yaw = item["vehicle_id"], item["target_ue_yaw_deg"]
            index = counters[yaw]
            counters[yaw] += 1
            x = (5, 16, 27)[index%3]
            y = ((91.1024, 138.9024) if yaw == 0 else (87.3024, 135.1024))[index//3]
            old_local = xyz(client.simGetVehiclePose(vehicle_name=name))
            old_world = xyz(client.simGetObjectPose(name))
            target = [x-origin[0], y-origin[1], origin[2]-.894]
            local = [old_local[i]+target[i]-old_world[i] for i in range(3)]
            q = airsim.to_quaternion(0, 0, math.radians(yaw))
            client.simSetVehiclePose(make_pose(local, [q.x_val, q.y_val, q.z_val, q.w_val]), True, vehicle_name=name)
            cleared.append({"vehicle_id": name, "temporary_ue_m": [x, y, .894], "yaw_deg": yaw})
    finally:
        client.simPause(was_paused)
    time.sleep(.5)
    parked = {}
    for name in client.listVehicles():
        if name != NAME:
            pos, pose = absolute_ue(client, name, origin)
            parked[name] = [*pos[:2], airsim.to_eularian_angles(pose.orientation)[2], 4.172, 1.918]
    return cleared, parked


def run_case(client, case, path, origin, parked):
    global TRAILER
    TRAILER = "Traffic_Trailer_Probe_"+case+"_"+str(time.monotonic_ns())
    (ROOT/"runs/ue_scene/trailer_trial_request.json").write_text(json.dumps({"trailer_name": TRAILER}))
    result = {"case": case, "path_ue_m": path, "samples": [], "controller": "ground-truth Pure Pursuit diagnostic"}
    client.reset()
    result["cleared_nearby_vehicles"], parked = clear_nearby(client, origin)
    client.enableApiControl(True, vehicle_name=NAME)
    original_local = xyz(client.simGetVehiclePose(vehicle_name=NAME))
    original_world = xyz(client.simGetObjectPose(NAME))
    x = path[0][0]
    start_y = 153 if case.startswith("cross") else 195
    target_ned = [x-origin[0], start_y-origin[1], origin[2]-.994]
    local = [original_local[i]+target_ned[i]-original_world[i] for i in range(3)]
    heading = airsim.to_quaternion(0, 0, math.pi/2)
    client.setCarControls(airsim.CarControls(brake=1), vehicle_name=NAME)
    client.simSetVehiclePose(make_pose(local, [heading.x_val, heading.y_val, heading.z_val, heading.w_val]), True, vehicle_name=NAME)
    try:
        client.simSpawnObject(TRAILER, "BP_Trailer3_Child", airsim.Pose(airsim.Vector3r(-200, 20, -8)),
                              airsim.Vector3r(1, 1, 1), False, True)
        result["setup"] = ue_script(client, "setup_trailer_trial.py", ROOT/"runs/ue_scene/trailer_trial_setup.json")
        if HISTORY.exists():
            HISTORY.unlink()
        ue_script(client, "monitor_trailer_trial.py", LIVE)
        time.sleep(1)
        initial_collision = client.simGetCollisionInfo(vehicle_name=NAME).time_stamp
        total = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        begin = last_print = last_progress = time.monotonic()
        progress_station = 0
        contact_deviation_since = None
        while time.monotonic()-begin < 240:
            live = json.loads(LIVE.read_text())
            if live.get("error") or time.monotonic()-live["monotonic_s"] > 3:
                raise RuntimeError("trailer telemetry stopped: "+str(live.get("error")))
            pos, pose = absolute_ue(client, NAME, origin)
            state = client.getCarState(vehicle_name=NAME)
            yaw = airsim.to_eularian_angles(pose.orientation)[2]
            rear = [pos[0]-.5*WHEELBASE_M*math.cos(yaw), pos[1]-.5*WHEELBASE_M*math.sin(yaw)]
            error, _ = project(pos[:2], path)
            _, station = project(rear, path)
            if station > progress_station+.5:
                progress_station = station
                last_progress = time.monotonic()
            target = at_station(path, station+max(2.7, abs(state.speed)*1.3))
            alpha = (math.atan2(target[1]-rear[1], target[0]-rear[0])-yaw+math.pi)%(2*math.pi)-math.pi
            delta = math.atan2(2*WHEELBASE_M*math.sin(alpha), max(.5, euclidean_xy(target, rear)))
            steering = max(-1.0, min(1.0, delta/math.radians(40)))
            collision = client.simGetCollisionInfo(vehicle_name=NAME)
            fresh_collision = collision.has_collided and collision.time_stamp != initial_collision
            envelopes = [[*b["centre_ue_m"][:2], math.radians(b["yaw_deg"]), *b["half_xy_m"]] for b in live["body_boxes"]]
            overlap = next((name for name, box in parked.items() if any(footprint_overlap(envelope, box) for envelope in envelopes)), None)
            wheel_offroad = [w["name"] for w in live["wheels"] if not w["road_actor"] or not (
                w["road_actor"].startswith("SM_Road_") or w["road_actor"].startswith("Traffic_SeamBridge_"))]
            bad_contact = [w["name"] for w in live["wheels"] if w["clearance_m"] is None or abs(w["clearance_m"]) > .12]
            if bad_contact and contact_deviation_since is None:
                contact_deviation_since = time.monotonic()
            if not bad_contact:
                contact_deviation_since = None
            sample = {"t_s": time.monotonic()-begin, "ue_m": pos, "speed_m_s": state.speed,
                      "path_error_m": error, "steering": steering, "trailer": live,
                      "fresh_collision": bool(fresh_collision), "collision_object": collision.object_name if fresh_collision else None,
                      "parked_vehicle_overlap": overlap, "offroad_wheels": wheel_offroad}
            if fresh_collision:
                sample["collision_impact_ned_m"] = [collision.impact_point.x_val, collision.impact_point.y_val, collision.impact_point.z_val]
                sample["collision_normal_ned"] = [collision.normal.x_val, collision.normal.y_val, collision.normal.z_val]
                sample["collision_penetration_depth_m"] = collision.penetration_depth
            result["samples"].append(sample)
            reasons = []
            if fresh_collision: reasons.append("truck collision: "+str(collision.object_name))
            if overlap: reasons.append("trailer envelope overlaps parked "+overlap)
            if wheel_offroad: reasons.append("trailer wheels leave road: "+", ".join(wheel_offroad))
            if contact_deviation_since is not None and time.monotonic()-contact_deviation_since > 1:
                reasons.append("sustained trailer tire contact deviation: "+", ".join(bad_contact))
            if live["constraint_broken"] or live["hitch_error_m"] > .08: reasons.append("trailer hitch separation")
            if not live["physics"]: reasons.append("trailer physics inactive")
            if live["cargo_sweep_hit"]: reasons.append("cargo swept collision: "+live["cargo_sweep_hit"])
            if live.get("physics_sweep_hit"): reasons.append("trailer physics envelope collision: "+live["physics_sweep_hit"])
            if time.monotonic()-last_progress > 20: reasons.append("tractor/trailer stalled for 20 seconds")
            if abs(live["articulation_deg"]) > 85: reasons.append("trailer jackknife")
            if abs(live["trailer_roll_deg"]) > 10 or abs(live["trailer_pitch_deg"]) > 15: reasons.append("trailer excessive tilt")
            if error > 1.5 or pos[2] < .7 or pos[2] > 1.4: reasons.append("tractor path/height deviation")
            if reasons:
                result["failure"] = reasons
                break
            if station >= total-1.2 and euclidean_xy(pos, path[-1]) < 2:
                result["reached_goal"] = True
                break
            speed_target = .9 if abs(steering) > .12 else 1.5
            throttle = .6 if abs(state.speed) < .2 and time.monotonic()-begin > 5 else .45
            client.setCarControls(airsim.CarControls(throttle=throttle if abs(state.speed) < speed_target else 0,
                steering=steering, brake=0 if abs(state.speed) < speed_target else .12), vehicle_name=NAME)
            if time.monotonic()-last_print > 5:
                print(f"{case}: x={pos[0]:.1f}, y={pos[1]:.1f}, speed={state.speed:.2f}, "
                      f"hitch={live['hitch_error_m']:.3f}, articulation={live['articulation_deg']:.1f}", flush=True)
                last_print = time.monotonic()
            time.sleep(.1)
        else:
            result["failure"] = ["timeout"]
    except Exception as e:
        result["failure"] = [repr(e)]
    finally:
        # Preserve evidence even if the RPC server disappears during cleanup.
        (OUTDIR/(case+".json")).write_text(json.dumps(result, indent=2)+"\n")
        client.setCarControls(airsim.CarControls(brake=1), vehicle_name=NAME)
        if HISTORY.exists():
            shutil.copyfile(HISTORY, OUTDIR/(case+"_telemetry.jsonl"))
        client.simDestroyObject(TRAILER)
        restored = ue_script(client, "restore_trailer_trial.py", ROOT/"runs/ue_scene/trailer_trial_restored.json")
        if not restored["restored"]:
            raise RuntimeError("Could not restore normal truck physics")
        client.reset()
        time.sleep(.5)
    rows = result["samples"]
    result["status"] = "PASS" if result.get("reached_goal") and not result.get("failure") else "FAIL"
    result["max_path_error_m"] = max((s["path_error_m"] for s in rows), default=None)
    result["max_hitch_error_m"] = max((s["trailer"]["hitch_error_m"] for s in rows), default=None)
    result["max_articulation_deg"] = max((abs(s["trailer"]["articulation_deg"]) for s in rows), default=None)
    print(case, result["status"], result.get("failure", ""), flush=True)
    (OUTDIR/(case+".json")).write_text(json.dumps(result, indent=2)+"\n")
    return result


def main():
    global OUTDIR
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=("cross_left", "cross_middle", "right", "left", "turns", "all"), default="all")
    parser.add_argument("--output", type=Path, default=OUTDIR)
    args = parser.parse_args()
    OUTDIR = args.output.resolve()
    OUTDIR.mkdir(parents=True, exist_ok=True)
    client = airsim.CarClient(timeout_value=5)
    if not client.ping() or client.simIsPause():
        raise RuntimeError("AirSim must be running and unpaused")
    if any(abs(client.getCarState(vehicle_name=n).speed) > .1 for n in client.listVehicles()):
        raise RuntimeError("all vehicles must be stationary before the trial")
    # Remove any previous isolated setup probe before starting reproducible cases.
    previous_trials = client.simListSceneObjects("Traffic_Trailer_Probe.*")
    if previous_trials:
        for previous_trial in previous_trials:
            client.simDestroyObject(previous_trial)
        client.reset()
    scene = ue_script(client, "export_scene.py", ROOT/"runs/ue_scene/scene.json")
    origin = [v/100 for v in scene["world_origin_cm"]]
    client.simRunConsoleCommand("py "+str(ROOT/"scripts/ue/prepare_dynamic_validation.py"))
    original_api = client.isApiControlEnabled(vehicle_name=NAME)
    settings_hash = hashlib.sha256(SETTINGS.read_bytes()).hexdigest()
    parked = {}
    home = {name: absolute_ue(client, name, origin)[0] for name in client.listVehicles()}
    for name in client.listVehicles():
        if name != NAME:
            pos, pose = absolute_ue(client, name, origin)
            parked[name] = [*pos[:2], airsim.to_eularian_angles(pose.orientation)[2], 4.172, 1.918]
    graph = json.loads((ROOT/"config/topology/industrialarea_ue.json").read_text())
    edges = {e["edge_id"]: e for e in graph["edges"]}
    paths = {"cross_left": [[-90.2936, 150], [-90.2936, 244]],
             "cross_middle": [[-8.304, 150], [-8.304, 244]]}
    # Vehicle-specific trial paths: keep the surveyed road geometry unchanged.
    # Left: approach on the right of the northbound lane and exit toward the
    # outer edge, then return to the surveyed westbound lane after the trailer.
    normal_x = edges["turn_j_2_1_S_W"]["centerline"][0][0]
    approach = [[normal_x, 190], [normal_x, 195]]
    for i in range(1, 21):
        t = i/20
        approach.append([normal_x+(-6.5-normal_x)*(3*t*t-2*t*t*t), 195+8*t])
    paths["left"] = approach+[[ -6.5, 205.8]]
    for i in range(1, 61):
        a = math.pi/2*i/60
        paths["left"].append([-21.5+15*math.cos(a), 205.8+15*math.sin(a)])
    paths["left"] += [[-50, 220.8], [-65, 219.1229], [-70, 219.1229]]
    # Right: a 12 m arc starts before the junction and finishes beyond its
    # eastern boundary, reducing trailer articulation at the road-tile join.
    normal_right = edges["turn_j_2_1_S_E"]["centerline"][0]
    right_x, outgoing_y = normal_right[0], edges["turn_j_2_1_S_E"]["centerline"][-1][1]
    paths["right"] = [[right_x, 175], [right_x, outgoing_y-12]]
    for i in range(1, 61):
        a = math.pi-math.pi/2*i/60
        paths["right"].append([right_x+12+12*math.cos(a), outgoing_y-12+12*math.sin(a)])
    paths["right"].append([30, outgoing_y])
    cases = list(paths) if args.case == "all" else ["left", "right"] if args.case == "turns" else [args.case]
    report = {"blueprint": "BP_Trailer3_Child", "vehicle": NAME, "world_origin_cm": scene["world_origin_cm"],
              "load": "stock unloaded Blueprint physics mass", "path_mode": "large truck: left approach right, radii left 15 m/right 12 m", "cases": []}
    try:
        for case in cases:
            report["cases"].append(run_case(client, case, paths[case], origin, parked))
    finally:
        client.enableApiControl(original_api, vehicle_name=NAME)
        report["settings_unchanged"] = hashlib.sha256(SETTINGS.read_bytes()).hexdigest() == settings_hash
        report["fleet_restored"] = all(euclidean_xy(absolute_ue(client, name, origin)[0], pos) < .1
                                         for name, pos in home.items())
        report["status"] = "PASS" if len(report["cases"]) == len(cases) and all(r["status"] == "PASS" for r in report["cases"]) and report["settings_unchanged"] and report["fleet_restored"] else "FAIL"
        (OUTDIR/"acceptance.json").write_text(json.dumps(report, indent=2)+"\n")
    print("Trailer trial:", report["status"], "report", OUTDIR/"acceptance.json", flush=True)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
