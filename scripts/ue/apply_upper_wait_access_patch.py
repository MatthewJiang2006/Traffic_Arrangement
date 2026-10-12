"""Patch only the copied upper UE sublevel for Q6/Q7/Q8 access trials."""
import json
import math
from pathlib import Path

import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/ue_scene/station_siting"
BACKUP = OUT / "IndustrialArea_Copy_before_upper_wait_access.umap"
SUBLEVEL = "IndustrialArea_Copy"
PROPS = [
    ("SM_Block_Wall_58", -79.99, 198.96),
    ("SM_Block_Wall_59", -79.99, 202.28),
    ("SM_Block_Wall_60", -79.99, 205.59),
    ("SM_Box_752", -81.61, 205.32),
    ("SM_Block_Wall_70", -80.08, 237.03),
    ("SM_Block_Wall_69", -80.08, 240.41),
    ("SM_Pole_16", -83.02, 241.94),
    ("SM_Block_Wall_66", -80.08, 250.48),
    ("SM_Block_Wall_67", -80.08, 247.13),
]
RAMPS = [
    ("TrafficStationQ6CurbRamp", -84.75, 199.0, 32.0),
    ("TrafficStationQ7Q8CurbRamp", -84.75, 243.0, 54.0),
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
        raise RuntimeError("Wrong parent editor map")
    if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():
        raise RuntimeError("Unsaved map edits; abort upper access patch")
    if not BACKUP.exists():
        raise RuntimeError("Upper sublevel backup required")
    if not unreal.EditorLevelLibrary.set_current_level_by_name(SUBLEVEL):
        raise RuntimeError("Cannot select copied upper sublevel")
    origin = unreal.GameplayStatics.get_world_origin_location(world)
    prefix = "/Game/IndustrialArea/Map/IndustrialArea_Copy.IndustrialArea_Copy:PersistentLevel."
    actors = [a for a in unreal.ObjectIterator(unreal.Actor)
              if a.get_path_name().startswith(prefix)]
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
            raise RuntimeError("Expected one surveyed upper prop: "+name)
        bounds = hits[0].get_actor_bounds(False)
        extent = bounds[1] if isinstance(bounds, tuple) else bounds.to_tuple()[1]
        if max(extent.x, extent.y) > 250 or extent.z > (500 if name.startswith("SM_Pole_") else 250):
            raise RuntimeError("Upper prop exceeds small-prop safety limit: "+name)
        matched.append(hits[0])
    removed = []
    for actor in matched:
        removed.append(actor.get_name())
        if not unreal.EditorLevelLibrary.destroy_actor(actor):
            raise RuntimeError("Cannot remove upper prop "+actor.get_name())
    cube = unreal.load_object(None, "/Engine/BasicShapes/Cube.Cube")
    road = next((a for a in actors if a.get_name() == "SM_Road_70"), None)
    material = None
    if road is not None:
        road_mesh = road.get_components_by_class(unreal.StaticMeshComponent)
        if road_mesh:
            material = road_mesh[0].get_material(0)
    saved_ramps = []
    for tag, x, y, length in RAMPS:
        if any(tag in [str(t) for t in a.tags] for a in actors):
            raise RuntimeError("Upper ramp already exists: "+tag)
        loc = unreal.Vector(x*100-origin.x, y*100-origin.y, 1.165*100-origin.z)
        ramp = unreal.EditorLevelLibrary.spawn_actor_from_class(
            unreal.StaticMeshActor, loc,
            unreal.Rotator(pitch=math.degrees(math.atan2(.15, 2.7)), yaw=0, roll=0))
        if ramp is None:
            raise RuntimeError("Cannot spawn upper curb ramp: "+tag)
        mesh = ramp.get_components_by_class(unreal.StaticMeshComponent)[0]
        mesh.set_mobility(unreal.ComponentMobility.MOVABLE)
        mesh.set_static_mesh(cube)
        ramp.set_actor_scale3d(unreal.Vector(2.7, length, .04))
        mesh.set_collision_profile_name("BlockAll")
        mesh.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
        mesh.set_mobility(unreal.ComponentMobility.STATIC)
        ramp.set_actor_label(tag)
        ramp.set_editor_property("tags", [unreal.Name(tag)])
        mesh.set_editor_property("cast_shadow", False)
        if material is not None:
            mesh.set_material(0, material)
        saved_ramps.append({"tag": tag, "center_m": [x, y, 1.165],
                            "size_m": [2.7, length, .04], "path": ramp.get_path_name()})
    if not unreal.EditorLevelLibrary.save_current_level():
        raise RuntimeError("Upper sublevel save failed")
    unreal.EditorLevelLibrary.set_current_level_by_name("IndustrialArea（复件）")
    (OUT / "upper_wait_access_patch.json").write_text(json.dumps({
        "saved": True, "sublevel": SUBLEVEL, "removed_small_props": removed,
        "ramps": saved_ramps, "backup": str(BACKUP)}, indent=2)+"\n")


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle = unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
