"""Cover the full rear-axle swept width through the W1 reversing turn."""
import json
from pathlib import Path
import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/"runs/ue_scene/station_siting"
BACKUP = OUT/"IndustrialArea_before_W1_ramp_widening.umap"
TAGS = ("TrafficW1RoadToCurb", "TrafficW1CurbToHardstand")
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
    actors = unreal.EditorLevelLibrary.get_all_level_actors()
    changed = []
    for tag in TAGS:
        hits = [a for a in actors if tag in [str(t) for t in a.tags]]
        if len(hits) != 1:
            raise RuntimeError("Expected one ramp " + tag)
        hits[0].set_actor_scale3d(unreal.Vector(4.0, 8.0, .04))
        changed.append(tag)
    if not unreal.EditorLoadingAndSavingUtils.save_map(
            world, "/Game/IndustrialArea/Map/IndustrialArea（复件）"):
        raise RuntimeError("W1 map save failed")
    (OUT/"w1_reverse_ramp_widening.json").write_text(json.dumps({
        "saved": True, "backup": str(BACKUP), "ramps": changed,
        "size_m": [4.0, 8.0, .04]}, indent=2)+"\n")


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle = unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
