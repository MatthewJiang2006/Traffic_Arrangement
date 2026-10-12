"""Read-only AirSim pawn wheel and road collision diagnostics."""

import json
from pathlib import Path

import unreal


ROOT = Path(__file__).resolve().parents[2]
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
origin = unreal.GameplayStatics.get_world_origin_location(world)
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
car = next(a for a in actors if a.get_name() == "Car_0_0")
mesh = next(c for c in car.get_components_by_class(unreal.SkeletalMeshComponent)
            if c.get_name() == "VehicleMesh")
road = next(a for a in actors if a.get_name() == "SM_Road_2")
road_mesh = road.get_components_by_class(unreal.StaticMeshComponent)[0]
result = {"car_position_cm": [car.get_actor_location().x+origin.x,
                               car.get_actor_location().y+origin.y,
                               car.get_actor_location().z+origin.z],
          "pawn_mesh_simulating_physics": mesh.is_simulating_physics(),
          "pawn_mesh_collision_enabled": str(mesh.get_collision_enabled()),
          "road_collision_enabled": str(road_mesh.get_collision_enabled()),
          "road_collision_profile": str(road_mesh.get_collision_profile_name()),
          "skeletal_asset": mesh.get_editor_property("skeletal_mesh").get_path_name(),
          "bone_count": mesh.get_num_bones(),
          "bone_names": [str(mesh.get_bone_name(i)) for i in range(mesh.get_num_bones())],
          "wheel_bones": {}, "components": []}
for name in ("WheelFL", "WheelFR", "WheelBL", "WheelBR"):
    p = mesh.get_socket_location(name)
    result["wheel_bones"][name] = {"exists": mesh.does_socket_exist(name),
                                   "location_cm": [p.x+origin.x, p.y+origin.y, p.z+origin.z]}
for component in car.get_components_by_class(unreal.ActorComponent):
    if "Movement" in component.get_class().get_name():
        result["components"].append({"name": component.get_name(), "class": component.get_class().get_name(),
                                     "methods": [n for n in dir(component) if any(s in n.lower() for s in ("wheel", "engine", "throttle"))]})
path = ROOT/"runs/ue_scene/drive_physics_inspection.json"
path.write_text(json.dumps(result, indent=2), encoding="utf-8")
unreal.log("Drive physics inspection written to " + str(path))
