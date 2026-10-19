#!/usr/bin/env python3
"""Continuous single-truck access for the six remaining warehouse service bays."""
import argparse
import json
import math
import time
from pathlib import Path

import airsim
from ue_dynamic_crossing_check import absolute_ue
from ue_junction_check import teleport
from ue_trailer_driving_check import ue_script
from ue_short_cargo_driving_check import stop_monitor
from validate_ue_station_entry import drive
from validate_ue_loading_access import map_fingerprints

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/"runs/ue_scene/station_siting"
NAME = "Car_0_0"
SITES = ("W1-unloading", "W2-unloading", "W5-loading", "W5-unloading",
         "W6-loading", "W6-unloading")


def route_phases(site):
    if site in ("MIDDLE-north","MIDDLE-south"):
        x,start,end,yaw=(-12.104,160,201,90) if site.endswith("north") else (-8.304,201,160,270)
        return [("straight",[[x,start,yaw],[x,end,yaw]],False)]
    candidates = json.loads((OUT/"remaining_service_analytic_paths.json").read_text())
    if site=="W5-from-Q1":
        exit_path=list(reversed(candidates["W5-unloading"]))
        staging=exit_path[-1]
        return [("forward_to_loading",[[-64,173,0],[-40,173,0]],False),
                ("forward_to_unloading",[[-40,173,0],[-28,173,0]],False),
                ("forward_exit",exit_path,False),
                ("lane_recovery",[staging,[staging[0],staging[1]+8,90]],False)]
    path = candidates[site]
    if site.startswith(("W1", "W2")):
        return [("entry", path, False), ("reverse_exit", list(reversed(path)), True),
                ("lane_recovery", [path[0], [21,path[0][1],180]], False)]
    staging = path[0]
    approach = [[staging[0],staging[1]-8,90], staging]
    exit_path = list(reversed(path))
    recovery = [staging, [staging[0],staging[1]+8,90]]
    return [("approach", approach, False), ("reverse_entry", path, True),
            ("forward_exit", exit_path, False), ("lane_recovery", recovery, False)]


def clear_near_route(client, origin, phases):
    points = [p for _,path,_ in phases for p in path]
    names = []
    for name in client.listVehicles():
        if name == NAME:
            continue
        pos,_ = absolute_ue(client,name,origin)
        if min(math.dist(pos[:2],p[:2]) for p in points) < 15:
            names.append(name)
    # Established, unused wide-road lanes; only the test setup relocates cars.
    slots = [(x,y,yaw) for x,yaw in ((-97.8936,90),(-86.4936,270))
             for y in (60,75,105,120,153,171,189,235,250,285,300)]
    slots += [(-94.0936,285,90)]
    # Never stage a cleared truck on the very route being tested.
    fixed=[absolute_ue(client,n,origin)[0][:2] for n in client.listVehicles() if n not in names and n!=NAME]
    slots=[p for p in slots if min(math.dist(p[:2],q[:2]) for q in points)>15
           and all(math.dist(p[:2],q)>10 for q in fixed)]
    if len(names)>len(slots):
        raise RuntimeError("Not enough clearance staging positions")
    cleared=[]
    for name,(x,y,yaw) in zip(names,slots):
        teleport(client,name,[x,y],yaw,origin)
        cleared.append(dict(vehicle=name,temporary_xy_m=[x,y],yaw_deg=yaw))
    return cleared


def run(client,origin,site,phases=None,parked=None,output_prefix="service_access_",hold_labels=(),max_speed_m_s=1):
    phases = route_phases(site) if phases is None else phases
    api = {n:client.isApiControlEnabled(vehicle_name=n) for n in client.listVehicles()}
    original_xy = {n:absolute_ue(client,n,origin)[0][:2] for n in api}
    report = dict(site=site,status="FAIL",phases={},failure=[],map_sha256=map_fingerprints(),
                  camera_changed=False,persistent_map_changed=False,
                  wheel_contact_is_diagnostic=True,scope="single-truck continuous low-speed access",
                  max_speed_m_s=max_speed_m_s)
    begin=time.monotonic()
    try:
        client.reset()
        report["cleared_vehicles"]=clear_near_route(client,origin,phases)
        report["stationary_waiting_vehicles"]=parked or []
        for item in parked or []:
            teleport(client,item["vehicle"],item["xy_m"],item["yaw_deg"],origin)
        start=phases[0][1][0]
        teleport(client,NAME,start[:2],start[2],origin)
        for i,(label,path,reverse) in enumerate(phases):
            record=dict(status="FAIL",failure=[],samples=[],path_xy_heading=path,reverse=reverse)
            report["phases"][label]=record
            drive(client,origin,path,record,teleport_start=False,start_monitor=(i==0),
                  reverse=reverse,launch_throttle=.5,max_speed_m_s=max_speed_m_s,
                  max_duration_s=180,stop_margin_m=.35,finish_tolerance_m=.55)
            client.setCarControls(airsim.CarControls(brake=1),vehicle_name=NAME)
            print(site,label,record["status"],record["failure"],flush=True)
            if record["status"]!="PASS":
                raise RuntimeError(label+": "+str(record["failure"]))
            if label in hold_labels:
                record["stationary_hold_s"]=3
                time.sleep(3)
                if abs(client.getCarState(vehicle_name=NAME).speed)>.08:
                    raise RuntimeError("Waiting vehicle did not remain stopped")
        report["status"]="PASS"
    except Exception as exc:
        report["failure"].append(str(exc))
    finally:
        client.setCarControls(airsim.CarControls(brake=1),vehicle_name=NAME)
        stop_monitor(client)
        client.reset()
        time.sleep(.3)
        for name,enabled in api.items():
            client.enableApiControl(enabled,vehicle_name=name)
        client.simRunConsoleCommand("py import builtins; builtins.short_cargo_monitor_request = {}")
        report["fleet_restore_max_xy_error_m"]=max(
            math.dist(original_xy[n],absolute_ue(client,n,origin)[0][:2]) for n in api)
        report["fleet_restore_max_speed_m_s"]=max(
            abs(client.getCarState(vehicle_name=n).speed) for n in api)
        report["vehicle_restored"]=(report["fleet_restore_max_xy_error_m"]<.3
                                     and report["fleet_restore_max_speed_m_s"]<.1)
        if not report["vehicle_restored"]:
            report["status"]="FAIL"
            report["failure"].append("Fleet restoration verification failed")
        report["elapsed_s"]=time.monotonic()-begin
        if report["map_sha256"]!=map_fingerprints():
            report["status"]="FAIL"
            report["failure"].append("Map changed during trial")
        (OUT/f"{output_prefix}{site}.json").write_text(json.dumps(report,indent=2)+"\n")
        print(site,report["status"],report["failure"],flush=True)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site",choices=(*SITES,"all","lanes","W5-from-Q1"),default="all")
    parser.add_argument("--resume",action="store_true",help="Skip passing cases with identical map and phase paths")
    args=parser.parse_args()
    client=airsim.CarClient(timeout_value=5)
    if not client.ping() or client.simIsPause():
        raise RuntimeError("PIE must be running and unpaused")
    if any(abs(client.getCarState(vehicle_name=n).speed)>.1 for n in client.listVehicles()):
        raise RuntimeError("Fleet must be stationary")
    scene=ue_script(client,"export_scene.py",ROOT/"runs/ue_scene/scene.json")
    origin=[v/100 for v in scene["world_origin_cm"]]
    client.simRunConsoleCommand("py "+str(ROOT/"scripts/ue/prepare_dynamic_validation.py"))
    results=[]
    selected=SITES if args.site=="all" else ("MIDDLE-north","MIDDLE-south") if args.site=="lanes" else (args.site,)
    for site in selected:
        previous=OUT/f"service_access_{site}.json"
        if args.resume and previous.exists():
            record=json.loads(previous.read_text())
            same_paths=all(record.get("phases",{}).get(label,{}).get("path_xy_heading")==path
                           for label,path,_ in route_phases(site))
            if record.get("status")=="PASS" and record.get("vehicle_restored") and same_paths and record.get("map_sha256")==map_fingerprints():
                print(site,"PASS (existing identical-map evidence)",flush=True)
                results.append(record)
                continue
        results.append(run(client,origin,site))
    if any(r["status"]!="PASS" for r in results):
        raise SystemExit(1)


if __name__=="__main__":
    main()
