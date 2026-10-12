"""Clear surveyed Q4/Q5 small props and add one shared east-shoulder ramp."""
import json
import math
from pathlib import Path

import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/ue_scene/station_siting"
BACKUP = OUT / "IndustrialArea_before_Q4_Q5_access.umap"
TAG = "TrafficStationQ4Q5CurbRamp"
PROPS = [
    ("SM_Block_Wall_70", -80.08, 109.01),
    ("SM_Pole_16", -83.02, 113.91),
    ("SM_Block_Wall_66", -80.08, 122.45),
    ("SM_Block_Wall_67", -80.08, 119.11),
]
handle = None


def apply(delta):
    global handle
    if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
        return
    if handle is not None:
        unreal.unregister_slate_post_tick_callback(handle)
        handle = None
    world = unreal.EditorLevelLibrary.get_editor_world()
    if world is None or world.get_name() != "IndustrialArea（复件）":
        raise RuntimeError("Wrong editor map")
    if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():
        raise RuntimeError("Unsaved map edits; abort Q4/Q5 patch")
    if not BACKUP.exists():
        raise RuntimeError("Current-map Q4/Q5 backup required")
    if not unreal.EditorLevelLibrary.set_current_level_by_name("IndustrialArea（复件）"):
        raise RuntimeError("Cannot select persistent map")
    origin = unreal.GameplayStatics.get_world_origin_location(world)
    actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
    if any(TAG in [str(t) for t in a.tags] for a in actors):
        raise RuntimeError("Q4/Q5 ramp already exists")
    matched = []
    for name, expected_x, expected_y in PROPS:
        hits = []
        for actor in actors:
            if actor.get_name() != name:
                continue
            loc = actor.get_actor_location()
            x, y = (loc.x+origin.x)/100, (loc.y+origin.y)/100
            if abs(x-expected_x) < .5 and abs(y-expected_y) < .5:
                hits.append(actor)
        if len(hits) != 1:
            raise RuntimeError("Expected one surveyed small prop: "+name)
        bounds = hits[0].get_actor_bounds(False)
        extent = bounds[1] if isinstance(bounds, tuple) else bounds.to_tuple()[1]
        width_m = 2*max(extent.x, extent.y)/100
        height_m = 2*extent.z/100
        if width_m > 4 or height_m > (10 if name.startswith("SM_Pole_") else 4):
            raise RuntimeError("Prop exceeds small-prop safety limit: "+name)
        matched.append(hits[0])
    removed = []
    for actor in matched:
        removed.append(actor.get_name())
        if not unreal.EditorLevelLibrary.destroy_actor(actor):
            raise RuntimeError("Failed to remove "+actor.get_name())
    cube = unreal.load_object(None, "/Engine/BasicShapes/Cube.Cube")
    x, y, z = -84.75, 116.0, 1.165
    pitch = math.degrees(math.atan2(.15, 2.7))
    loc = unreal.Vector(x*100-origin.x, y*100-origin.y, z*100-origin.z)
    ramp = unreal.EditorLevelLibrary.spawn_actor_from_class(
        unreal.StaticMeshActor, loc, unreal.Rotator(pitch=pitch, yaw=0, roll=0))
    if ramp is None:
        raise RuntimeError("Cannot spawn shared Q4/Q5 ramp")
    mesh = ramp.get_components_by_class(unreal.StaticMeshComponent)[0]
    mesh.set_mobility(unreal.ComponentMobility.MOVABLE)
    mesh.set_static_mesh(cube)
    ramp.set_actor_scale3d(unreal.Vector(2.7, 50.0, .04))
    mesh.set_collision_profile_name("BlockAll")
    mesh.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
    mesh.set_mobility(unreal.ComponentMobility.STATIC)
    ramp.set_actor_label(TAG)
    ramp.set_editor_property("tags", [unreal.Name(TAG)])
    mesh.set_editor_property("cast_shadow", False)
    road = next((a for a in actors if a.get_name() == "SM_Road_70"), None)
    if road is not None:
        road_mesh = road.get_components_by_class(unreal.StaticMeshComponent)
        if road_mesh:
            material = road_mesh[0].get_material(0)
            if material is not None:
                mesh.set_material(0, material)
    if not unreal.EditorLoadingAndSavingUtils.save_map(world, "/Game/IndustrialArea/Map/IndustrialArea（复件）"):
        raise RuntimeError("Q4/Q5 map save failed")
    (OUT / "q4_q5_access_patch.json").write_text(json.dumps({
        "saved": True, "removed_small_props": removed,
        "ramp": {"center_m": [x, y, z], "size_m": [2.7, 50.0, .04],
                 "pitch_deg": pitch, "actor": ramp.get_path_name()},
        "backup": str(BACKUP)}, indent=2)+"\n")


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle = unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
