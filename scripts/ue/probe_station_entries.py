"""Read-only ground and obstacle profiles across selected station entrances."""
import json
import math
from pathlib import Path

import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/ue_scene/station_siting"
request = json.loads((OUT / "entry_profile_request.json").read_text())
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
origin = unreal.GameplayStatics.get_world_origin_location(world)
ignore = [a for a in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
          if a.get_class().get_name() in ("TrafficTruckPawn", "BP_Truck_C", "CameraActor")]


def ground(x, y):
    a = unreal.Vector(x*100-origin.x, y*100-origin.y, 1200-origin.z)
    b = unreal.Vector(a.x, a.y, -100-origin.z)
    hit = unreal.SystemLibrary.line_trace_single(
        world, a, b, unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,
        False, ignore, unreal.DrawDebugTrace.NONE, True)
    row = hit.to_tuple() if hit else None
    return ((row[5].z+origin.z)/100, row[9].get_name() if row[9] else "") if row and row[0] else (None, "")


profiles = []
for entry in request["profiles"]:
    start, end = entry["from_xy_m"], entry["to_xy_m"]
    length = math.hypot(end[0]-start[0], end[1]-start[1])
    count = max(1, int(math.ceil(length/.5)))
    rows = []
    for i in range(count+1):
        u = i/count
        x, y = start[0]+u*(end[0]-start[0]), start[1]+u*(end[1]-start[1])
        z, actor = ground(x, y)
        rows.append({"distance_m": round(length*u, 2),
                     "xy_m": [round(x, 3), round(y, 3)],
                     "ground_z_m": z, "surface_actor": actor})
    profiles.append({"id": entry["id"], "from_xy_m": start, "to_xy_m": end,
                     "samples": rows})
(OUT / request.get("output", "entry_profiles.json")).write_text(
    json.dumps({"read_only": True, "profiles": profiles}, indent=2)+"\n")
