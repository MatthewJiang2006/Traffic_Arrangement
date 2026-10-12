"""Widen the backed-up Q3 ramp after the first physical curb contact trial."""
import json
import math
from pathlib import Path
import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/ue_scene/station_siting"
TAG = "TrafficStationQ3CurbRamp"
handle = None


def apply(delta):
    global handle
    if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
        return
    if handle is not None:
        unreal.unregister_slate_post_tick_callback(handle)
        handle = None
    world = unreal.EditorLevelLibrary.get_editor_world()
    if world.get_name() != "IndustrialArea（复件）":
        raise RuntimeError("Wrong editor map")
    if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():
        raise RuntimeError("Unsaved map edits")
    origin = unreal.GameplayStatics.get_world_origin_location(world)
    ramps = [a for a in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
             if TAG in [str(t) for t in a.tags]]
    if len(ramps) != 1:
        raise RuntimeError("Expected one saved Q3 ramp")
    ramp = ramps[0]
    x, y, z = -84.75, 71.0, 1.165
    ramp.set_actor_location(unreal.Vector(x*100-origin.x, y*100-origin.y, z*100-origin.z), False, False)
    ramp.set_actor_rotation(unreal.Rotator(pitch=math.degrees(math.atan2(.15, 2.7)), yaw=0, roll=0), False)
    ramp.set_actor_scale3d(unreal.Vector(2.7, 32.0, .04))
    if not unreal.EditorLoadingAndSavingUtils.save_map(world, "/Game/IndustrialArea/Map/IndustrialArea（复件）"):
        raise RuntimeError("Q3 ramp adjustment save failed")
    (OUT / "q3_ramp_adjustment.json").write_text(json.dumps({
        "saved": True, "center_m": [x, y, z], "size_m": [2.7, 32.0, .04],
        "slope_m_per_m": .15/2.7}, indent=2)+"\n")


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle = unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
