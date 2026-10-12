"""Read-only full-body straight-corridor probe for approved warehouse berths."""
import json
import math
from pathlib import Path

import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/ue_scene/station_siting/access_corridors.json"
plan = json.loads((ROOT / "config/planning/ue_station_proposal.json").read_text(encoding="utf-8"))
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
origin = unreal.GameplayStatics.get_world_origin_location(world)
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
ignore = [a for a in actors if a.get_class().get_name() in
          ("TrafficTruckPawn", "BP_Truck_C", "CameraActor")]


def ground(x, y):
    start = unreal.Vector(x*100-origin.x, y*100-origin.y, 1200-origin.z)
    end = unreal.Vector(start.x, start.y, -100-origin.z)
    hit = unreal.SystemLibrary.line_trace_single(
        world, start, end, unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,
        False, ignore, unreal.DrawDebugTrace.NONE, True)
    row = hit.to_tuple() if hit else None
    return ((row[5].z+origin.z)/100, row[9].get_name() if row[9] else "") if row and row[0] else (None, "")


def check(x, y, yaw):
    z, surface = ground(x, y)
    if z is None or not .7 < z < 1.35:
        return {"clear": False, "reason": "no low ground", "surface_actor": surface}
    co, si = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    for dx, dy in ((-4.5, -1.65), (-4.5, 1.65), (4.5, -1.65), (4.5, 1.65)):
        h, _ = ground(x+dx*co-dy*si, y+dx*si+dy*co)
        if h is None or abs(h-z) > .08:
            return {"clear": False, "reason": "footprint ground discontinuity",
                    "surface_actor": surface}
    center = unreal.Vector(x*100-origin.x, y*100-origin.y, (z+2.3)*100-origin.z)
    hit = unreal.SystemLibrary.box_trace_single(
        world, center, center, unreal.Vector(450, 165, 195),
        unreal.Rotator(pitch=0, yaw=yaw, roll=0),
        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, False, ignore,
        unreal.DrawDebugTrace.NONE, True)
    row = hit.to_tuple() if hit else None
    if row and row[0]:
        return {"clear": False, "reason": "static overlap",
                "obstacle_actor": row[9].get_name() if row[9] else ""}
    return {"clear": True, "surface_actor": surface, "ground_z_m": z}


request_path = OUT.parent / "access_request.json"
request = json.loads(request_path.read_text()) if request_path.exists() else {}
points = [(s["id"]+"-"+kind, s[kind]) for s in plan["stations"]
          for kind in ("loading", "unloading", "waiting") if s[kind]]
points += [(p["id"], p) for p in plan["shared_waiting"]]
if "points" in request:
    points = [(p["id"], p) for p in request["points"]]
rows = []
for label, point in points:
    x, y = point["xy_m"]
    yaw = point["heading_deg"]
    co, si = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    samples = []
    for offset in range(-18, 19, 2):
        result = check(x+offset*co, y+offset*si, yaw)
        samples.append({"offset_m": offset, **result})
    backward = next((abs(s["offset_m"])-2 for s in reversed(samples[:9]) if not s["clear"]), 18)
    forward = next((s["offset_m"]-2 for s in samples[10:] if not s["clear"]), 18)
    rows.append({"id": label, "xy_m": [x, y], "heading_deg": yaw,
                 "samples": samples, "backward_clear_m": max(0, backward),
                 "forward_clear_m": max(0, forward)})
OUT.parent.mkdir(parents=True, exist_ok=True)
target = OUT.parent / request.get("output", OUT.name)
target.write_text(json.dumps({"read_only": True, "box_m": [9, 3.3, 3.9],
                              "step_m": 2, "points": rows}, indent=2) + "\n")
