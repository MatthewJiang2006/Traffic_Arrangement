"""Check that all visible truck tires rest close to traced UE road surfaces."""

import json
from pathlib import Path

import unreal


ROOT = Path(__file__).resolve().parents[2]
worlds = [w for w in unreal.ObjectIterator(unreal.World)
          if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name()]
if len(worlds) != 1:
    raise RuntimeError("Expected one PIE world")
world = worlds[0]
origin = unreal.GameplayStatics.get_world_origin_location(world)
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
vehicles = [a for a in actors if a.get_class().get_name() == "TrafficTruckPawn"]
visuals = [a for a in actors if a.get_class().get_name() == "BP_Truck_C"]
if len(vehicles) != 24 or len(visuals) != 24:
    raise RuntimeError("Expected 24 chassis and 24 visual trucks")
ignore = vehicles+visuals
rows = []
for visual in visuals:
    parent = visual.get_attach_parent_actor()
    wheels = []
    for component in visual.get_components_by_class(unreal.StaticMeshComponent):
        if not component.get_name().startswith("Wheel_"):
            continue
        p = component.get_world_location()
        mesh = component.get_editor_property("static_mesh")
        bottom = p.z-mesh.get_bounds().box_extent.z
        hit = unreal.SystemLibrary.line_trace_single(
            world, unreal.Vector(p.x, p.y, p.z+300), unreal.Vector(p.x, p.y, p.z-1000),
            unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, False, ignore, unreal.DrawDebugTrace.NONE, True)
        values = hit.to_tuple() if hit is not None else None
        road = values[9].get_name() if values and values[0] and values[9] else None
        clearance = (bottom-values[5].z)/100 if values and values[0] else None
        wheels.append({"name": component.get_name(), "road_actor": road,
                       "clearance_m": round(clearance, 4) if clearance is not None else None})
    rows.append({"vehicle_id": parent.get_name() if parent else None,
                 "visual": visual.get_name(), "wheels": sorted(wheels, key=lambda x: x["name"])})
all_wheels = [wheel for row in rows for wheel in row["wheels"]]
clearances = [w["clearance_m"] for w in all_wheels if w["clearance_m"] is not None]
result = {"vehicle_count": len(rows), "wheel_count": len(all_wheels),
          "min_clearance_m": min(clearances) if clearances else None,
          "max_clearance_m": max(clearances) if clearances else None,
          "pass": len(all_wheels) == 144 and all(w["road_actor"] and w["road_actor"].startswith("SM_Road_")
                      and -.03 <= w["clearance_m"] <= .03 for w in all_wheels),
          "vehicles": sorted(rows, key=lambda x: x["vehicle_id"])}
output = ROOT/"runs/ue_scene/truck_contact_validation.json"
output.write_text(json.dumps(result, indent=2), encoding="utf-8")
unreal.log("Truck contact validation written to " + str(output))
