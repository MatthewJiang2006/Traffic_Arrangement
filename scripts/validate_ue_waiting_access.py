#!/usr/bin/env python3
"""Verify remaining approved waiting berths and their continuous road access."""
import argparse
import json
import math
from pathlib import Path
import airsim
from ue_trailer_driving_check import ue_script
from validate_ue_remaining_service import run
from validate_ue_loading_access import map_fingerprints
from ue_access_evidence import unchanged_access_region

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/"runs/ue_scene/station_siting"
SITES=("Q1","Q2","Q6","Q7","Q8","Q9","Q10","Q11","Q12")
LANES=("LOWER-north","LOWER-south","UPPER-north","UPPER-south")


def phases(site):
    if site in LANES:
        offset=0 if site.startswith("LOWER") else 128
        north=site.endswith("north")
        x,start,end,yaw=(-12.104,94,151,90) if north else (-8.304,151,94,270)
        return [("straight",[[x,start+offset,yaw],[x,end+offset,yaw]],False)]
    middle=json.loads((OUT/"remaining_service_analytic_paths.json").read_text())
    if site in ("Q1","Q2","W5-waiting-clearance"):
        path=middle["W5-loading"]
        staging=path[0]
        approach=[[staging[0],staging[1]-8,90],staging]
        if site=="W5-waiting-clearance":
            entry=path
        else:
            target=-64 if site=="Q1" else -52
            entry=path+[[path[-1][0]+(target-path[-1][0])*i/50,173,0] for i in range(1,51)]
        exit_path=list(reversed(middle["W5-unloading"]))
        result=[("approach",approach,False),("waiting_entry",entry,True)]
        if site!="W5-waiting-clearance":
            result.append(("forward_to_loading",[entry[-1],[-40,173,0]],False))
        return result + [
                ("forward_to_unloading",[[-40,173,0],[-28,173,0]],False),
                ("forward_exit",exit_path,False),
                ("lane_recovery",[staging,[staging[0],staging[1]+8,90]],False)]
    candidates=json.loads((OUT/"waiting_analytic_candidates.json").read_text())
    return [(p["label"],p["path"],p["reverse"]) for p in candidates[site]]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site",choices=(*SITES,"all","lanes","W5-waiting-clearance"),default="all")
    parser.add_argument("--resume",action="store_true")
    args=parser.parse_args()
    client=airsim.CarClient(timeout_value=5)
    if not client.ping() or client.simIsPause() or any(abs(client.getCarState(vehicle_name=n).speed)>.1 for n in client.listVehicles()):
        raise RuntimeError("PIE must run with stationary vehicles")
    scene=ue_script(client,"export_scene.py",ROOT/"runs/ue_scene/scene.json")
    origin=[v/100 for v in scene["world_origin_cm"]]
    client.simRunConsoleCommand("py "+str(ROOT/"scripts/ue/prepare_dynamic_validation.py"))
    results=[]
    selected=SITES if args.site=="all" else LANES if args.site=="lanes" else (args.site,)
    for site in selected:
        previous=OUT/f"waiting_access_{site}.json"
        if args.resume and previous.exists():
            record=json.loads(previous.read_text())
            identical=all(record.get("phases",{}).get(label,{}).get("path_xy_heading")==path for label,path,_ in phases(site))
            if record.get("status")=="PASS" and record.get("vehicle_restored") and identical and (
                    record.get("map_sha256")==map_fingerprints() or unchanged_access_region(record,previous.stem,map_fingerprints())):
                print(site,"PASS (preserved unchanged-route evidence)",flush=True)
                results.append(record)
                continue
        parked=[dict(vehicle="Car_0_1",xy_m=[-64,173],yaw_deg=0),
                dict(vehicle="Car_0_2",xy_m=[-52,173],yaw_deg=0)] if site=="W5-waiting-clearance" else []
        results.append(run(client,origin,site,phases=phases(site),parked=parked,
                           output_prefix="waiting_access_",hold_labels=("waiting_entry",),
                           max_speed_m_s=3 if site in LANES else 2))
    if any(r["status"]!="PASS" for r in results):
        raise SystemExit(1)


if __name__=="__main__":
    main()
