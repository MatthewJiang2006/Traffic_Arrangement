"""Read-only PIE collision traces under the proposed 24 vehicle positions."""

import json
from pathlib import Path
from statistics import mean

import unreal


ROOT = Path(__file__).resolve().parents[2]
PLAN = json.loads((ROOT/"config/traffic/industrialarea_ue_spawn_plan.json").read_text(encoding="utf-8"))
worlds = [world for world in unreal.ObjectIterator(unreal.World)
          if "UEDPIE_" in world.get_path_name() and "IndustrialArea（复件）" in world.get_path_name()]
if len(worlds) != 1:
    raise RuntimeError("Expected one active IndustrialArea PIE world")
world = worlds[0]
origin = unreal.GameplayStatics.get_world_origin_location(world)
half_length = PLAN["truck_length_m_from_pawn_bounds"]/2
vehicles_to_ignore = [actor for actor in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
                      if actor.get_class().get_name() in ("TrafficTruckPawn", "BP_Truck_C")]
vehicles = {actor.get_name(): actor for actor in vehicles_to_ignore
            if actor.get_class().get_name() == "TrafficTruckPawn"}
visuals = {actor.get_attach_parent_actor().get_name(): actor for actor in vehicles_to_ignore
           if actor.get_class().get_name() == "BP_Truck_C" and actor.get_attach_parent_actor()}


def ground_at(x, y):
    xx, yy = x*100-origin.x, y*100-origin.y
    hit = unreal.SystemLibrary.line_trace_single(
        world, unreal.Vector(xx, yy, 500), unreal.Vector(xx, yy, -1500),
        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, False, vehicles_to_ignore, unreal.DrawDebugTrace.NONE, True)
    if hit is None:
        return {"blocking_hit": False}
    values = hit.to_tuple()
    if not values[0]:
        return {"blocking_hit": False}
    actor = values[9]
    return {"blocking_hit": True, "actor": actor.get_name() if actor else None,
            "ue_surface_z_m": round((values[5].z+origin.z)/100, 3)}


results = []
for item in PLAN["targets"]:
    x, y, _ = item["target_ue_position_m"]
    points = [{"part": part, "ue_xy_m": [round(x+offset, 3), y],
               **ground_at(x+offset, y)}
              for part, offset in (("rear", -half_length), ("centre", 0), ("front", half_length))]
    vehicle = vehicles[item["vehicle_id"]]
    wheel_bottom_offsets = []
    for component in visuals[item["vehicle_id"]].get_components_by_class(unreal.StaticMeshComponent):
        if component.get_name().startswith("Wheel_"):
            mesh = component.get_editor_property("static_mesh")
            wheel_bottom_offsets.append((component.get_world_location().z-mesh.get_bounds().box_extent.z
                                         -vehicle.get_actor_location().z)/100)
    if len(wheel_bottom_offsets) != 6:
        raise RuntimeError("Expected six visible truck tires for height calibration")
    tire_bottom_relative_m = mean(wheel_bottom_offsets)
    centre_road_z_m = points[1].get("ue_surface_z_m")
    target_actor_z_m = (centre_road_z_m - tire_bottom_relative_m
                        if centre_road_z_m is not None else None)
    results.append({"vehicle_id": item["vehicle_id"], "samples": points,
                    "mean_visible_tire_bottom_relative_actor_m": round(tire_bottom_relative_m, 4),
                    "target_actor_z_m_for_visual_tire_contact": round(target_actor_z_m, 4) if target_actor_z_m else None,
                    "road_surface_pass": all(sample["blocking_hit"]
                                             and (sample["actor"] or "").startswith("SM_Road_")
                                             and abs(sample["ue_surface_z_m"]-1.1) < .2
                                             for sample in points)})

OUTPUT = ROOT/"runs/ue_scene/spawn_surface_validation.json"
OUTPUT.parent.mkdir(parents=True, exist_ok=True)
OUTPUT.write_text(json.dumps({"map_path": world.get_path_name(),
                              "world_origin_cm": [origin.x, origin.y, origin.z],
                              "vehicle_count": len(results), "all_road_surfaces": all(row["road_surface_pass"] for row in results),
                              "vehicles": results}, indent=2), encoding="utf-8")
unreal.log("UE spawn surface validation written to " + str(OUTPUT))
