"""Read-only fine survey of station curb ramps against the turn lanes they cross."""
import json
import traceback
from pathlib import Path

import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/ue_node_pairs/ramp_lane_geometry.json"


def report():
    world = next(w for w in unreal.ObjectIterator(unreal.World)
                 if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
    origin = unreal.GameplayStatics.get_world_origin_location(world)
    actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
    ignore = [a for a in actors if a.get_class().get_name() in ("TrafficTruckPawn", "BP_Truck_C")]

    ramps = []
    for a in actors:
        name = a.get_name()
        if "CurbRamp" not in name:
            continue
        loc = a.get_actor_location()
        box = unreal.SystemLibrary.get_actor_bounds(a)
        ramps.append(dict(
            name=name,
            outer=a.get_outermost().get_name(),
            location_m=[(loc.x + origin.x) / 100, (loc.y + origin.y) / 100, (loc.z + origin.z) / 100],
            bounds_origin_m=[(box[0].x + origin.x) / 100, (box[0].y + origin.y) / 100, (box[0].z + origin.z) / 100],
            bounds_extent_m=[box[1].x / 100, box[1].y / 100, box[1].z / 100],
            pitch_deg=a.get_actor_rotation().pitch,
        ))

    def trace(x, y):
        hit = unreal.SystemLibrary.line_trace_single(
            world,
            unreal.Vector(x * 100 - origin.x, y * 100 - origin.y, 400 - origin.z),
            unreal.Vector(x * 100 - origin.x, y * 100 - origin.y, -200 - origin.z),
            unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, False, ignore, unreal.DrawDebugTrace.NONE, True)
        v = hit.to_tuple() if hit else None
        return dict(x=x, y=y, z=(v[5].z + origin.z) / 100 if v and v[0] else None,
                    actor=v[9].get_name() if v and v[0] and v[9] else None)

    rows = []
    bands = ([212, 213, 214, 214.5, 215, 215.25, 215.5, 215.75, 216, 216.25, 216.5, 217, 218, 219, 220]
             + [135, 136, 136.5, 137, 137.5, 138, 138.5, 139, 140]
             + [264, 265, 265.5, 266, 266.5, 266.93, 267, 267.5, 268, 269, 270, 271])
    for y in bands:
        rows.append(dict(y=y, hits=[trace(-88 + i * .25, y) for i in range(41)]))
    grid = [dict(y=y, hits=[trace(-90 + i * .5, y) for i in range(29)]) for y in range(180, 276)]
    grid_low = [dict(y=y, hits=[trace(-90 + i * .5, y) for i in range(29)]) for y in range(50, 150)]
    return dict(ramps=ramps, rows=rows, grid=grid, grid_low=grid_low, map_changed=False, camera_changed=False,
                pie_world=world.get_path_name())


try:
    OUT.write_text(json.dumps(report(), indent=2) + "\n")
except BaseException as exc:
    OUT.write_text(json.dumps(dict(error=repr(exc), traceback=traceback.format_exc()), indent=2) + "\n")
    raise
