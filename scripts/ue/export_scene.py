"""Read the loaded UE level into a JSON snapshot. Run inside UE's Python console.

The script only reads actors and writes runs/ue_scene/scene.json in this repo.
"""

import json
import os
from pathlib import Path

import unreal


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "runs" / "ue_scene" / "scene.json"
MAP_PATH = "/Game/IndustrialArea/Map/map.map"


def xyz(vector):
    return [float(vector.x), float(vector.y), float(vector.z)]


editor_world = unreal.load_object(None, MAP_PATH)
if editor_world is None or editor_world.get_path_name() != MAP_PATH:
    raise RuntimeError("map level is not loaded")

pie_world = unreal.EditorLevelLibrary.get_game_world()
if (pie_world is None or "UEDPIE_" not in pie_world.get_path_name()
        or "UEDPIE_0_map" not in pie_world.get_path_name()):
    raise RuntimeError("Expected one active IndustrialArea PIE world")
origin = unreal.GameplayStatics.get_world_origin_location(pie_world)

actors = []
for actor in unreal.GameplayStatics.get_all_actors_of_class(editor_world, unreal.Actor):
    location = actor.get_actor_location()
    rotation = actor.get_actor_rotation()
    bounds_origin, bounds_extent = actor.get_actor_bounds(False)
    meshes = []
    for component in actor.get_components_by_class(unreal.StaticMeshComponent):
        mesh = component.get_editor_property("static_mesh")
        if mesh is not None:
            meshes.append(mesh.get_path_name())
    decals = []
    for component in actor.get_components_by_class(unreal.DecalComponent):
        material = component.get_editor_property("decal_material")
        decals.append({
            "material": material.get_path_name() if material is not None else None,
            "size_cm": xyz(component.get_editor_property("decal_size")),
        })
    actors.append({
        "path": actor.get_path_name(),
        "name": actor.get_name(),
        "label": actor.get_actor_label(),
        "class": actor.get_class().get_name(),
        "location_cm": xyz(location),
        "rotation_deg": [float(rotation.pitch), float(rotation.yaw), float(rotation.roll)],
        "bounds_origin_cm": xyz(bounds_origin),
        "bounds_extent_cm": xyz(bounds_extent),
        "meshes": meshes,
        "decals": decals,
        "decal_right_vector": xyz(actor.get_actor_right_vector()) if decals else None,
    })

pie_vehicles = []
for actor in unreal.GameplayStatics.get_all_actors_of_class(pie_world, unreal.Actor):
    if actor.get_name().startswith("Car_") and actor.get_class().get_name() == "TrafficTruckPawn":
        rotation = actor.get_actor_rotation()
        bounds_origin, bounds_extent = actor.get_actor_bounds(False)
        pie_vehicles.append({"name": actor.get_name(), "location_from_origin_cm": xyz(actor.get_actor_location()),
                             "rotation_deg": [float(rotation.pitch), float(rotation.yaw), float(rotation.roll)],
                             "bounds_extent_cm": xyz(bounds_extent)})

OUTPUT.parent.mkdir(parents=True, exist_ok=True)
OUTPUT.write_text(json.dumps({
    "process_id": os.getpid(),
    "map_path": MAP_PATH,
    "editor_world": editor_world.get_path_name(),
    "pie_world": pie_world.get_path_name(),
    "world_origin_cm": xyz(origin),
    "editor_actor_count": len(actors),
    "pie_vehicles": sorted(pie_vehicles, key=lambda row: row["name"]),
    "actors": sorted(actors, key=lambda row: row["path"]),
}, ensure_ascii=False, indent=2), encoding="utf-8")
unreal.log("UE road survey written to " + str(OUTPUT))
