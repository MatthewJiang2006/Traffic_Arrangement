"""Measure fixed short cargo against cab/chassis in the active PIE fleet."""
import json
import math
from pathlib import Path
import unreal

ROOT = Path(__file__).resolve().parents[2]
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())


def bounds(component, actor):
    b = component.get_editor_property("static_mesh").get_bounds()
    points = []
    for x in (-1, 1):
        for y in (-1, 1):
            for z in (-1, 1):
                p = unreal.Vector(b.origin.x+x*b.box_extent.x,
                                  b.origin.y+y*b.box_extent.y,
                                  b.origin.z+z*b.box_extent.z)
                p = unreal.MathLibrary.transform_location(component.get_world_transform(), p)
                p = unreal.MathLibrary.inverse_transform_location(actor.get_actor_transform(), p)
                points.append([p.x, p.y, p.z])
    return {"min_cm": [min(p[i] for p in points) for i in range(3)],
            "max_cm": [max(p[i] for p in points) for i in range(3)]}


rows = []
for actor in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor):
    if actor.get_class().get_name() != "BP_Truck_C":
        continue
    components = {c.get_name(): c for c in actor.get_components_by_class(unreal.StaticMeshComponent)}
    parent = actor.get_attach_parent_actor()
    row = {"vehicle_id": parent.get_name() if parent else None, "cargo_present": "TrafficShortCargo" in components}
    if row["cargo_present"]:
        cargo = components["TrafficShortCargo"]
        c, cab, frame = [bounds(components[n], actor) for n in ("TrafficShortCargo", "Truck_Body_01", "Truck_Body_00")]
        suspension = actor.get_component_by_class(unreal.SkeletalMeshComponent)
        hitch = unreal.MathLibrary.inverse_transform_location(actor.get_actor_transform(), suspension.get_socket_location("TruckAtt"))
        wheel_top = max(bounds(comp, actor)["max_cm"][2] for name, comp in components.items() if name.startswith("Wheel_"))
        # The exported mesh's broad underside plane lies at local z=-0.128 cm;
        # its support feet extend lower and should sit at the hitch height.
        floor_z = cargo.get_editor_property("relative_location").z - .128
        row.update(cargo_bounds=c, cab_gap_cm=cab["min_cm"][0]-c["max_cm"][0],
                   frame_gap_cm=c["min_cm"][2]-frame["max_cm"][2],
                   hitch_gap_cm=c["min_cm"][2]-hitch.z,
                   floor_to_wheel_top_cm=floor_z-wheel_top,
                   rear_support_margin_cm=c["min_cm"][0]-frame["min_cm"][0],
                   cargo_mesh=cargo.get_editor_property("static_mesh").get_path_name(),
                   cargo_yaw_deg=cargo.get_editor_property("relative_rotation").yaw,
                   opening_faces_cab=math.cos(math.radians(cargo.get_editor_property("relative_rotation").yaw)) < -.99,
                   cargo_physics=cargo.is_simulating_physics(), collision=str(cargo.get_collision_enabled()))
        row["pass"] = (abs(row["cab_gap_cm"]-20)<1 and abs(row["hitch_gap_cm"])<1
                       and row["floor_to_wheel_top_cm"]>0
                       and row["rear_support_margin_cm"]>=0 and not row["cargo_physics"]
                       and row["opening_faces_cab"])
    else:
        row["pass"] = False
    rows.append(row)
result = {"vehicle_count": len(rows), "pass": len(rows)==24 and all(r["pass"] for r in rows),
          "vehicles": sorted(rows, key=lambda r: r["vehicle_id"] or "")}
(ROOT/"runs/ue_scene/short_cargo_validation.json").write_text(json.dumps(result, indent=2)+"\n")
