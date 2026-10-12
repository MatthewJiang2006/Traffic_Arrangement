"""Remove explicitly surveyed small props from the backed-up UE editor map.

Request file: runs/ue_scene/station_siting/small_prop_removal_request.json.
The script ends PIE, edits the persistent level, and leaves PIE stopped so the
next play session can be started after the save completes.
"""
import json
from pathlib import Path

import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/ue_scene/station_siting"
REQUEST = json.loads((OUT / "small_prop_removal_request.json").read_text())
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
        raise RuntimeError("Unsaved map edits: aborting small-prop removal")
    if not Path(REQUEST["backup"]).exists():
        raise RuntimeError("Current map backup is required")
    if not unreal.EditorLevelLibrary.set_current_level_by_name("IndustrialArea（复件）"):
        raise RuntimeError("Cannot select persistent level")
    origin = unreal.GameplayStatics.get_world_origin_location(world)
    actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
    matched = []
    for spec in REQUEST["props"]:
        if not spec["name"].startswith(("SM_Box_", "SM_Barrel_", "SM_Pole_", "SM_Fence_",
                                        "SM_Block_Wall_", "SM_Stop_Sign_", "SM_Leaf_")):
            raise RuntimeError("Actor is not an approved small-prop class")
        candidates = []
        for actor in actors:
            if actor.get_name() != spec["name"]:
                continue
            loc = actor.get_actor_location()
            x, y = (loc.x+origin.x)/100, (loc.y+origin.y)/100
            if (x-spec["xy_m"][0])**2+(y-spec["xy_m"][1])**2 < .25:
                candidates.append(actor)
        if len(candidates) != 1:
            raise RuntimeError("Expected one small prop: "+str(spec))
        actor = candidates[0]
        bounds = actor.get_actor_bounds(False)
        extent = bounds[1] if isinstance(bounds, tuple) else bounds.to_tuple()[1]
        dimensions_m = [2*extent.x/100, 2*extent.y/100, 2*extent.z/100]
        max_height = 10 if spec["name"].startswith("SM_Pole_") else 5
        if max(dimensions_m[:2]) > 5 or dimensions_m[2] > max_height:
            raise RuntimeError("Prop exceeds 5 m safety limit: "+str(spec))
        matched.append((actor, dimensions_m))
    removed = []
    for actor, dimensions in matched:
        removed.append({"name": actor.get_name(), "dimensions_m": dimensions})
        if not unreal.EditorLevelLibrary.destroy_actor(actor):
            raise RuntimeError("Could not remove "+actor.get_name())
    if not unreal.EditorLoadingAndSavingUtils.save_map(world, "/Game/IndustrialArea/Map/IndustrialArea（复件）"):
        raise RuntimeError("Small-prop map save failed")
    (OUT / REQUEST["output"]).write_text(json.dumps({
        "saved": True, "removed": removed, "backup": REQUEST["backup"]}, indent=2)+"\n")


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle = unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
