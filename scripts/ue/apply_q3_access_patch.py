"""Save a local Q3 curb ramp and clear three short wall segments in the UE map.

Run from a live PIE console. The script ends PIE and edits the backed-up editor
map. Start a new PIE session after the report is written. No camera is moved.
"""
import json
import math
from pathlib import Path

import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/ue_scene/station_siting"
MAP_PATH = "/Game/IndustrialArea/Map/IndustrialArea（复件）"
TAG = "TrafficStationQ3CurbRamp"
WALLS = [(58, 70.94), (59, 74.25), (60, 77.57)]
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
        raise RuntimeError("Expected IndustrialArea（复件） editor map")
    if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():
        raise RuntimeError("Map has unsaved edits; Q3 patch aborted")
    if not (OUT / "IndustrialArea_before_Q3_access.umap").exists():
        raise RuntimeError("Q3 map backup is required")
    if not unreal.EditorLevelLibrary.set_current_level_by_name("IndustrialArea（复件）"):
        raise RuntimeError("Cannot select persistent level")
    origin = unreal.GameplayStatics.get_world_origin_location(world)
    actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
    matched = []
    for number, y in WALLS:
        hits = []
        for actor in actors:
            if actor.get_name() != "SM_Block_Wall_" + str(number):
                continue
            loc = actor.get_actor_location()
            x0, y0 = (loc.x+origin.x)/100, (loc.y+origin.y)/100
            if abs(x0+79.99) < .5 and abs(y0-y) < .5:
                hits.append(actor)
        if len(hits) != 1:
            raise RuntimeError("Expected one short Q3 wall segment "+str(number))
        matched.append(hits[0])
    ramps = [a for a in actors if TAG in [str(t) for t in a.tags]]
    if len(ramps) > 1:
        raise RuntimeError("Duplicate Q3 curb ramps")
    if ramps and matched:
        raise RuntimeError("Partial Q3 patch state; inspect map before retry")
    removed = []
    for wall in matched:
        loc = wall.get_actor_location()
        removed.append({"name": wall.get_name(),
                        "xy_m": [(loc.x+origin.x)/100, (loc.y+origin.y)/100]})
        if not unreal.EditorLevelLibrary.destroy_actor(wall):
            raise RuntimeError("Failed to clear "+wall.get_name())
    cube = unreal.load_object(None, "/Engine/BasicShapes/Cube.Cube")
    if cube is None:
        raise RuntimeError("Engine cube mesh unavailable")
    x, y, z = -84.25, 74.0, 1.175
    pitch = math.degrees(math.atan2(.15, 1.8))
    loc = unreal.Vector(x*100-origin.x, y*100-origin.y, z*100-origin.z)
    ramp = unreal.EditorLevelLibrary.spawn_actor_from_class(
        unreal.StaticMeshActor, loc, unreal.Rotator(pitch=pitch, yaw=0, roll=0))
    if ramp is None:
        raise RuntimeError("Cannot spawn Q3 ramp")
    component = ramp.get_components_by_class(unreal.StaticMeshComponent)[0]
    component.set_mobility(unreal.ComponentMobility.MOVABLE)
    component.set_static_mesh(cube)
    ramp.set_actor_scale3d(unreal.Vector(1.8, 15.0, .04))
    component.set_collision_profile_name("BlockAll")
    component.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
    component.set_mobility(unreal.ComponentMobility.STATIC)
    ramp.set_actor_label(TAG)
    ramp.set_editor_property("tags", [unreal.Name(TAG)])
    component.set_editor_property("cast_shadow", False)
    road = next((a for a in actors if a.get_name() == "SM_Road_70"), None)
    if road is not None:
        road_mesh = road.get_components_by_class(unreal.StaticMeshComponent)
        if road_mesh:
            material = road_mesh[0].get_material(0)
            if material is not None:
                component.set_material(0, material)
    if not unreal.EditorLoadingAndSavingUtils.save_map(world, MAP_PATH):
        raise RuntimeError("Q3 map save failed")
    (OUT / "q3_access_patch.json").write_text(json.dumps({
        "saved": True, "map": MAP_PATH, "removed_small_walls": removed,
        "ramp": {"center_m": [x, y, z], "size_m": [1.8, 15.0, .04],
                 "pitch_deg": pitch, "actor": ramp.get_path_name()},
        "backup": str(OUT / "IndustrialArea_before_Q3_access.umap")}, indent=2)+"\n")


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle = unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
