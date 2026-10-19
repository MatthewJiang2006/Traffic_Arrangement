"""Close the surveyed 20 cm wheel gap at W1 curb-to-hardstand ramp."""
import json
import math
from pathlib import Path
import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/"runs/ue_scene/station_siting"
BACKUP = OUT/"IndustrialArea_before_W1_curb_gap_bridge.umap"
TAG = "TrafficW1CurbToHardstand"
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
        raise RuntimeError("Unsaved map edits")
    if not BACKUP.exists():
        raise RuntimeError("W1 map backup required")
    if not unreal.EditorLevelLibrary.set_current_level_by_name("IndustrialArea（复件）"):
        raise RuntimeError("Cannot select persistent map")
    hits = [a for a in unreal.EditorLevelLibrary.get_all_level_actors()
            if TAG in [str(t) for t in a.tags]]
    if len(hits) != 1:
        raise RuntimeError("Expected one W1 hardstand ramp")
    origin = unreal.GameplayStatics.get_world_origin_location(world)
    ramp = hits[0]
    ramp.set_actor_location(unreal.Vector(33.0*100-origin.x,
                                         124.0*100-origin.y,
                                         1.162*100-origin.z), False, False)
    ramp.set_actor_rotation(unreal.Rotator(
        pitch=math.degrees(math.atan2(.176, 4.5)), yaw=0, roll=0), False)
    ramp.set_actor_scale3d(unreal.Vector(4.5, 8.0, .04))
    if not unreal.EditorLoadingAndSavingUtils.save_map(
            world, "/Game/IndustrialArea/Map/IndustrialArea（复件）"):
        raise RuntimeError("W1 map save failed")
    (OUT/"w1_curb_gap_bridge.json").write_text(json.dumps({
        "saved": True, "backup": str(BACKUP), "tag": TAG,
        "center_m": [33.0, 124.0, 1.162], "size_m": [4.5, 8.0, .04]}, indent=2)+"\n")


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle = unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
