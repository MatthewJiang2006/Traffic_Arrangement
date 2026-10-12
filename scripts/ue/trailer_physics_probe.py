"""Read Blueprint hitch settings and trailer collision bodies."""
import json
from pathlib import Path
import unreal

world = next(w for w in unreal.ObjectIterator(unreal.World) if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
actor = next(a for a in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor) if a.get_name() == "Traffic_Trailer_Probe")
mesh = actor.get_component_by_class(unreal.SkeletalMeshComponent)
result = {"properties": {}, "bodies": [], "methods": [n for n in dir(actor) if any(s in n.lower() for s in ("trailer", "attach", "call_method"))]}
for prop in ("TruckRef", "AttachToName", "bSimulate", "TrailerSK", "DetachConstraint"):
    try: result["properties"][prop] = str(actor.get_editor_property(prop))
    except Exception as e: result["properties"][prop] = str(e)
asset = mesh.get_editor_property("skeletal_mesh").get_editor_property("physics_asset")
result["physics_asset"] = asset.get_path_name()
try:
    bodies = asset.get_editor_property("SkeletalBodySetups")
except Exception:
    bodies = [o for o in unreal.ObjectIterator(unreal.Object)
              if isinstance(o, unreal.BodySetup)
              and o.get_path_name().startswith(asset.get_path_name())]
for body in bodies:
    row = {"bone": str(body.get_editor_property("bone_name"))}
    geom = body.get_editor_property("agg_geom")
    for shape in ("sphere_elems", "box_elems", "sphyl_elems", "convex_elems"):
        row[shape] = []
        for s in geom.get_editor_property(shape):
            data = {}
            for key in ("center", "radius", "x", "y", "z", "rotation"):
                try: data[key] = str(s.get_editor_property(key))
                except Exception: pass
            row[shape].append(data)
    result["bodies"].append(row)
truck = next(a for a in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor) if a.get_name() == "Car_0_0")
truck_mesh = truck.get_component_by_class(unreal.SkeletalMeshComponent)
truck_asset = truck_mesh.get_editor_property("skeletal_mesh").get_editor_property("physics_asset")
result["truck_bodies"] = []
for body in unreal.ObjectIterator(unreal.Object):
    if isinstance(body, unreal.BodySetup) and body.get_path_name().startswith(truck_asset.get_path_name()):
        bone = str(body.get_editor_property("bone_name"))
        result["truck_bodies"].append({"bone": bone, "simulating": truck_mesh.is_simulating_physics(bone)})
constraint = actor.get_component_by_class(unreal.PhysicsConstraintComponent)
result["constraint_component_names"] = {}
for key in ("component_name1", "component_name2"):
    try: result["constraint_component_names"][key] = str(constraint.get_editor_property(key))
    except Exception as e: result["constraint_component_names"][key] = str(e)
result["trailer_root_simulating"] = mesh.is_simulating_physics("Root")
path = Path(__file__).resolve().parents[2]/"runs/ue_scene/trailer_physics_inspection.json"
path.write_text(json.dumps(result, indent=2), encoding="utf-8")
