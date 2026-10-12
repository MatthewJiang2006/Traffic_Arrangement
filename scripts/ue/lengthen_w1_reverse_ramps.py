"""Lengthen W1 ramps to reduce chassis pitch and rear-wheel lift."""
import json
import math
from pathlib import Path
import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/"runs/ue_scene/station_siting"
BACKUP = OUT/"IndustrialArea_before_W1_ramp_lengthening.umap"
RAMPS = [
    ("TrafficW1RoadToCurb", 40.75, 124.0, 1.175, -math.degrees(math.atan2(.15, 4.0))),
    ("TrafficW1CurbToHardstand", 32.75, 124.0, 1.162, math.degrees(math.atan2(.176, 4.0))),
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
        raise RuntimeError("Unsaved map edits")
    if not BACKUP.exists():
        raise RuntimeError("W1 map backup required")
    if not unreal.EditorLevelLibrary.set_current_level_by_name("IndustrialArea（复件）"):
        raise RuntimeError("Cannot select persistent map")
    actors = unreal.EditorLevelLibrary.get_all_level_actors()
    origin = unreal.GameplayStatics.get_world_origin_location(world)
    saved = []
    for tag, x, y, z, pitch in RAMPS:
        hits = [a for a in actors if tag in [str(t) for t in a.tags]]
        if len(hits) != 1:
            raise RuntimeError("Expected exactly one ramp " + tag)
        ramp = hits[0]
        ramp.set_actor_location(unreal.Vector(x*100-origin.x, y*100-origin.y,
                                               z*100-origin.z), False, False)
        ramp.set_actor_rotation(unreal.Rotator(pitch=pitch, yaw=0, roll=0), False)
        ramp.set_actor_scale3d(unreal.Vector(4.0, 5.0, .04))
        saved.append({"tag": tag, "center_m": [x, y, z],
                      "size_m": [4.0, 5.0, .04], "pitch_deg": pitch})
    if not unreal.EditorLoadingAndSavingUtils.save_map(
            world, "/Game/IndustrialArea/Map/IndustrialArea（复件）"):
        raise RuntimeError("W1 map save failed")
    (OUT/"w1_reverse_ramp_lengthening.json").write_text(json.dumps({
        "saved": True, "backup": str(BACKUP), "ramps": saved}, indent=2)+"\n")


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle = unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
