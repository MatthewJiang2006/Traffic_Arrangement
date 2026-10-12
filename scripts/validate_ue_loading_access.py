#!/usr/bin/env python3
"""Continuous legal-road approach, reverse berth, exit and exclusive-zone trial."""
import argparse
import json
import hashlib
import math
from pathlib import Path
import sys
import time

import airsim

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from logistics_park.safety.collision import OBB, overlaps
from logistics_park.scheduling.maneuver_reservation import ManeuverReservations
from ue_dynamic_crossing_check import absolute_ue
from ue_junction_check import teleport
from ue_short_cargo_driving_check import stop_monitor
from ue_trailer_driving_check import ue_script
from validate_ue_station_entry import drive, WHEELBASE_M

OUT = ROOT/"runs/ue_scene/station_siting"
OWNER, BLOCKER, FOLLOWER = "Car_0_0", "Car_0_1", "Car_0_2"
MAP_DIRECTORY = Path("/home/song03/Traffic_ue4/Content/IndustrialArea/Map")


def map_fingerprints():
    return {name: hashlib.sha256((MAP_DIRECTORY/name).read_bytes()).hexdigest()
            for name in ("IndustrialArea（复件）.umap","IndustrialArea_Copy.umap")}


def with_headings(points, final=None):
    result = []
    for i, p in enumerate(points):
        a, b = (p, points[i+1]) if i+1 < len(points) else (points[i-1], p)
        result.append([*p[:2], math.degrees(math.atan2(b[1]-a[1], b[0]-a[0])) % 360])
    if final is not None:
        result[-1][2] = final
    return result


def routes(site):
    validated = ROOT/"config/planning/ue_loading_access_routes.json"
    if validated.exists():
        saved = json.loads(validated.read_text())
        current = hashlib.sha256((ROOT/"config/topology/industrialarea_ue.json").read_bytes()).hexdigest()
        if saved["topology_sha256"] != current:
            raise RuntimeError("Road topology changed; regenerate loading access routes")
        if saved["map_sha256"] != map_fingerprints():
            raise RuntimeError("Saved UE map changed; revalidate loading access routes")
        return saved["sites"][site]
    offset = 0 if site == "W1" else 128
    graph = json.loads((ROOT/"config/topology/industrialarea_ue.json").read_text())
    junction = "j_1_2" if site == "W1" else "j_3_2"
    edge = next(e for e in graph["edges"]
                if e["edge_id"] == f"turn_{junction}_W_S_single_single")
    clear_edge = next(e for e in graph["edges"]
                      if e["edge_id"] == ("road_j_1_2_j_1_1" if site == "W1"
                                            else "road_j_3_2_j_3_1"))
    west_y = clear_edge["centerline"][0][1]
    clear_path = with_headings([[30,west_y],[15,west_y]])
    # Stop aligned with the legal -Y lane. Back along a radius-5 m quarter
    # circle, then straight into the berth. Heading is continuous at shifts.
    lane_x, start_y = 44.898, 119+offset
    reverse = []
    for i in range(81):
        angle = math.pi/2*i/80
        reverse.append([lane_x-5+5*math.cos(angle), start_y+5*math.sin(angle),
                        (270+math.degrees(angle)) % 360])
    reverse += [[39.898-(39.898-28)*i/60,124+offset,0] for i in range(1,61)]
    # Keep the already physically validated departure: reversing the entry
    # curve alone would send the forward cab too close to the opposite curb.
    exit_path = json.loads((OUT/f"{site}_loading_exit_candidate.json").read_text())["results"][0]["path_xy_heading"]
    approach = with_headings([[15, edge["centerline"][0][1]]]+edge["centerline"]
                             +[[lane_x,start_y]], 270)
    recovery = with_headings([exit_path[-1][:2], [44.898, 107+offset]])
    return dict(approach=approach, reverse=reverse, exit=exit_path, recovery=recovery,
                clear_blocker=clear_path, clear_connector=clear_edge["edge_id"],
                source_edge=f"road_{'j_1_1_j_1_2' if site == 'W1' else 'j_3_1_j_3_2'}",
                connector=edge["edge_id"], offset=offset,
                zone_bounds_m=[22, 113+offset, 51, 147+offset])


def fleet_snapshot(c, origin):
    rows, boxes = {}, {}
    for name in c.listVehicles():
        pos, pose = absolute_ue(c, name, origin)
        yaw = airsim.to_eularian_angles(pose.orientation)[2]
        boxes[name] = OBB(*pos[:2], yaw, 9, 3.3)
        rows[name] = dict(xy_m=pos[:2], yaw_deg=math.degrees(yaw),
                          speed_m_s=abs(c.getCarState(vehicle_name=name).speed))
    return rows, boxes


def phase(c, origin, name, path, label, report, **kwargs):
    record = dict(status="FAIL", failure=[], samples=[], path_xy_heading=path)
    report["phases"][label] = record
    throttle = kwargs.pop("launch_throttle", .3)
    record["launch_throttle"] = throttle
    record["speed_limit_m_s"] = kwargs.get("max_speed_m_s",1.0)
    drive(c, origin, path, record, vehicle_name=name, launch_throttle=throttle,
          max_duration_s=180, **kwargs)
    c.setCarControls(airsim.CarControls(brake=1), vehicle_name=name)
    print(report["site"], label, record["status"], record["failure"], flush=True)
    if record["status"] != "PASS":
        raise RuntimeError(label+": "+str(record["failure"]))


def run(c, origin, site, case):
    r = routes(site)
    offset = r["offset"]
    report = dict(site=site, case=case, status="FAIL", failure=[], phases={}, routes=r,
                  persistent_map_changed=False, camera_changed=False, events=[], traffic_samples=[],
                  map_sha256=map_fingerprints())
    report["acceptance_criteria"] = ["complete_route", "no_body_obstacle_contact",
                                      "no_vehicle_collision", "exclusive_zone_and_tail_clear_release"]
    report["wheel_contact_is_diagnostic"] = True
    names = (OWNER, BLOCKER, FOLLOWER) if case == "reservation" else (OWNER,)
    old_api = {n: c.isApiControlEnabled(vehicle_name=n) for n in names}
    begin = time.monotonic()
    reservation = None
    try:
        c.reset()
        for name in names:
            c.enableApiControl(True, vehicle_name=name)
        if case.startswith("straight"):
            north = case == "straight_north"
            x = 41.098 if north else 44.898
            path = with_headings([[x, (104 if north else 126)+offset],
                                  [x, (126 if north else 104)+offset]])
            phase(c, origin, OWNER, path, case, report,
                  launch_throttle=.28 if north else .45,
                  max_speed_m_s=.5 if north else 1.0)
        else:
            reservation = None
            held = False
            waited = 0
            if case == "reservation":
                teleport(c, OWNER, r["approach"][0][:2], 0, origin)
                teleport(c, BLOCKER, r["clear_blocker"][0][:2], 180, origin)
                teleport(c, FOLLOWER, [0, r["approach"][0][1]], 0, origin)
                time.sleep(.5)
                x0,y0,x1,y1 = r["zone_bounds_m"]
                reservation = ManeuverReservations({site: OBB((x0+x1)/2,(y0+y1)/2,0,x1-x0,y1-y0)})
                _, boxes = fleet_snapshot(c, origin)
                if reservation.request(site, OWNER, boxes, 0):
                    raise RuntimeError("Occupied reversing zone was incorrectly granted")
                # Clear the occupied westbound lane in its legal direction;
                # keep the eastbound follower outside the intersection.
                phase(c, origin, BLOCKER, r["clear_blocker"],
                      "clear_blocker", report, teleport_start=False, start_monitor=True,
                      launch_throttle=.45)
                _, boxes = fleet_snapshot(c, origin)
                if not reservation.request(site, OWNER, boxes, time.monotonic()-begin):
                    raise RuntimeError("Zone still occupied after forward clearance")
                held = True

            def hook(row, audit):
                nonlocal waited
                if reservation is None:
                    return
                observations, boxes = fleet_snapshot(c, origin)
                if held and reservation.intruders(site, boxes):
                    raise RuntimeError("Unauthorized zone entry: "+str(reservation.intruders(site, boxes)))
                for a, b in ((OWNER,FOLLOWER),(OWNER,BLOCKER),(FOLLOWER,BLOCKER)):
                    if overlaps(boxes[a], boxes[b], margin=.15):
                        raise RuntimeError("Vehicle clearance envelope overlap: "+a+"/"+b)
                f = observations[FOLLOWER]
                o = observations[OWNER]
                remaining = 15-f["xy_m"][0]
                # Keep the follower on the upstream road, before the junction.
                may_advance = o["xy_m"][0] > 30 or o["xy_m"][1] < 130+offset
                brake = not may_advance or remaining < .65+f["speed_m_s"]**2/2
                yaw = math.radians(f["yaw_deg"])
                aim_angle = math.atan2(r["approach"][0][1]-f["xy_m"][1], 3)
                alpha = (aim_angle-yaw+math.pi) % (2*math.pi)-math.pi
                steer = max(-.4,min(.4,math.atan2(2*WHEELBASE_M*math.sin(alpha),3)/math.radians(40)))
                c.setCarControls(airsim.CarControls(
                    throttle=.3 if not brake and f["speed_m_s"] < .7 else 0,
                    brake=1 if brake else .1 if f["speed_m_s"] >= .7 else 0,
                    steering=steer,
                    is_manual_gear=True, manual_gear=1, gear_immediate=True),vehicle_name=FOLLOWER)
                if remaining < 1 and f["speed_m_s"] < .08:
                    waited += 1
                hit = c.simGetCollisionInfo(vehicle_name=FOLLOWER)
                if hit.has_collided and hit.time_stamp != follower_baseline:
                    raise RuntimeError("Follower collision: "+hit.object_name)
                report["traffic_samples"].append(dict(t_s=time.monotonic()-begin,
                                                       owner=reservation.owners.get(site), vehicles=observations))

            follower_baseline = c.simGetCollisionInfo(vehicle_name=FOLLOWER).time_stamp
            phase(c, origin, OWNER, r["approach"], "approach", report,
                  teleport_start=case != "reservation", start_monitor=case == "reservation",
                  stop_margin_m=.03, finish_tolerance_m=.15, sample_hook=hook,
                  launch_throttle=.45)
            if reservation is not None:
                _, boxes = fleet_snapshot(c, origin)
                if reservation.release(site, OWNER, boxes, time.monotonic()-begin):
                    raise RuntimeError("Reservation released at the reverse starting pose")
            phase(c, origin, OWNER, r["reverse"], "reverse_entry", report,
                  teleport_start=False, reverse=True, sample_hook=hook)
            phase(c, origin, OWNER, r["exit"], "forward_exit", report,
                  teleport_start=False, sample_hook=hook,
                  launch_throttle=.28, max_speed_m_s=.5)
            phase(c, origin, OWNER, r["recovery"], "lane_recovery", report,
                  teleport_start=False, sample_hook=hook)
            if reservation is not None:
                _, boxes = fleet_snapshot(c, origin)
                if not reservation.release(site, OWNER, boxes, time.monotonic()-begin):
                    raise RuntimeError("Truck tail has not cleared the reserved zone")
                held = False
                if waited < 10:
                    raise RuntimeError("Follower stationary waiting evidence missing")
                report["follower_stationary_wait_samples"] = waited
                # Release the follower after the owner's complete departure.
                follow = r["approach"][:-1]+[[44.898,125+offset,270]]
                phase(c, origin, FOLLOWER, follow, "follower_released", report,
                      teleport_start=False, start_monitor=True)
                report["events"] = reservation.events
        report["status"] = "PASS"
    except Exception as exc:
        report["failure"].append(str(exc))
        print(site,case,"FAIL",exc,flush=True)
    finally:
        if reservation is not None:
            report["events"] = reservation.events
        for name in names:
            c.setCarControls(airsim.CarControls(brake=1), vehicle_name=name)
        stop_monitor(c)
        c.reset()
        for name, enabled in old_api.items():
            c.enableApiControl(enabled, vehicle_name=name)
        c.simRunConsoleCommand("py import builtins; builtins.short_cargo_monitor_request = {}")
        report["vehicle_restored"] = True
        if report["map_sha256"] != map_fingerprints():
            report["status"] = "FAIL"
            report["failure"].append("Saved map changed during trial")
        report["elapsed_s"] = time.monotonic()-begin
        (OUT/f"loading_access_{site}_{case}.json").write_text(json.dumps(report,indent=2)+"\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site", choices=("W1","W2","both"), default="both")
    parser.add_argument("--case", choices=("approach","reservation","straight_south","straight_north","lanes","all"), default="all")
    args = parser.parse_args()
    c = airsim.CarClient(timeout_value=5)
    if not c.ping() or c.simIsPause() or any(abs(c.getCarState(vehicle_name=n).speed)>.1 for n in c.listVehicles()):
        raise RuntimeError("PIE must run with a stationary fleet")
    scene = ue_script(c,"export_scene.py",ROOT/"runs/ue_scene/scene.json")
    origin = [v/100 for v in scene["world_origin_cm"]]
    c.simRunConsoleCommand("py "+str(ROOT/"scripts/ue/prepare_dynamic_validation.py"))
    cases = (("approach","reservation","straight_south","straight_north") if args.case == "all"
             else ("straight_south","straight_north") if args.case == "lanes" else (args.case,))
    for site in (("W1","W2") if args.site == "both" else (args.site,)):
        for case in cases:
            if run(c,origin,site,case)["status"] != "PASS":
                raise SystemExit(1)


if __name__ == "__main__":
    main()
