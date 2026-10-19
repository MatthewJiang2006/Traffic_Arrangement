"""Trim the four station curb ramps out of the junction corridors they block.

Surveyed defect: each ramp is a 2.7 m tilted plate rising to +14.8 cm at its east
edge.  The waiting hardstand is already cut back to road level where the east-west
roads cross (j_2_0 y 213..221, j_3_0 y 261..269, j_1_0 y 85..93 / 133..141), but
the ramps were not, leaving a 15 cm cliff inside those traffic lanes.  This patch
only shortens the ramps along Y so their ends sit flush with the hardstand edges;
the slope across X, the plate thickness and the retained berth coverage are
unchanged.
"""
import json
import math
import traceback
from pathlib import Path

import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/ue_scene/station_siting"
RECORD = OUT / "station_ramp_corridor_trim.json"
BACKUPS = [OUT / "IndustrialArea_before_station_ramp_corridor_trim.umap",
           OUT / "IndustrialArea_Copy_before_station_ramp_corridor_trim.umap"]
PREFIX = {
    "IndustrialArea（复件）": "/Game/IndustrialArea/Map/IndustrialArea（复件）.IndustrialArea（复件）:PersistentLevel.",
    "IndustrialArea_Copy": "/Game/IndustrialArea/Map/IndustrialArea_Copy.IndustrialArea_Copy:PersistentLevel.",
}
# tag, level, surveyed center y, surveyed y length, new center y, new y length, corridor kept clear
MODS = [
    ("TrafficStationQ3CurbRamp", "IndustrialArea（复件）", 71.0, 32.0, 69.75, 29.5, "j_1_0 north y 85..93"),
    ("TrafficStationQ4Q5CurbRamp", "IndustrialArea（复件）", 116.0, 50.0, 113.0, 39.0, "j_1_0 y 85..93 and y 133..141"),
    ("TrafficStationQ6CurbRamp", "IndustrialArea_Copy", 199.0, 32.0, 197.75, 29.5, "j_2_0 y 213..221"),
    ("TrafficStationQ7Q8CurbRamp", "IndustrialArea_Copy", 243.0, 54.0, 241.0, 39.0, "j_2_0 y 213..221 and j_3_0 y 261..269"),
]
RAMPS_X, RAMPS_Z, RAMPS_LEN_X, RAMPS_THICKNESS = -84.75, 1.165, 2.7, .04
RAMPS_PITCH = math.degrees(math.atan2(.15, 2.7))
handle = None


def fail(message):
    RECORD.write_text(json.dumps(dict(error=message, traceback=traceback.format_exc()), indent=2) + "\n")
    raise RuntimeError(message)


def apply(delta):
    global handle
    if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
        return
    if handle is not None:
        unreal.unregister_slate_post_tick_callback(handle)
        handle = None
    world = unreal.EditorLevelLibrary.get_editor_world()
    if world is None or world.get_name() != "IndustrialArea（复件）":
        fail("Wrong parent editor map")
    if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():
        fail("Unsaved map edits; abort ramp trim")
    if not all(p.exists() for p in BACKUPS):
        fail("Both map backups required")
    origin = unreal.GameplayStatics.get_world_origin_location(world)
    actors = list(unreal.ObjectIterator(unreal.Actor))
    changed = []
    for tag, level, old_y, old_len, new_y, new_len, cleared in MODS:
        hits = [a for a in actors
                if a.get_name() == tag and a.get_path_name().startswith(PREFIX[level])]
        if len(hits) != 1:
            fail("Expected exactly one ramp %s in %s, found %d" % (tag, level, len(hits)))
        actor = hits[0]
        loc = actor.get_actor_location()
        scale = actor.get_actor_scale3d()
        rot = actor.get_actor_rotation()
        x, y, z = (loc.x + origin.x) / 100, (loc.y + origin.y) / 100, (loc.z + origin.z) / 100
        if max(abs(x - RAMPS_X), abs(y - old_y), abs(z - RAMPS_Z),
               abs(scale.x - RAMPS_LEN_X), abs(scale.y - old_len), abs(scale.z - RAMPS_THICKNESS),
               abs(rot.pitch - RAMPS_PITCH), abs(rot.yaw), abs(rot.roll)) > .01:
            fail("Surveyed ramp %s changed: loc=%s scale=%s pitch=%.4f"
                 % (tag, (x, y, z), (scale.x, scale.y, scale.z), rot.pitch))
        actor.set_actor_location(
            unreal.Vector(RAMPS_X * 100 - origin.x, new_y * 100 - origin.y, RAMPS_Z * 100 - origin.z), False, False)
        actor.set_actor_scale3d(unreal.Vector(RAMPS_LEN_X, new_len, RAMPS_THICKNESS))
        changed.append(dict(tag=tag, level=level, new_center_m=[RAMPS_X, new_y, RAMPS_Z],
                            new_size_m=[RAMPS_LEN_X, new_len, RAMPS_THICKNESS],
                            old_y_range_m=[old_y - old_len / 2, old_y + old_len / 2],
                            new_y_range_m=[new_y - new_len / 2, new_y + new_len / 2],
                            corridor_kept_clear=cleared))
    for level in ("IndustrialArea（复件）", "IndustrialArea_Copy"):
        if not unreal.EditorLevelLibrary.set_current_level_by_name(level):
            fail("Cannot select " + level)
        if not unreal.EditorLevelLibrary.save_current_level():
            fail("Save failed " + level)
    unreal.EditorLevelLibrary.set_current_level_by_name("IndustrialArea（复件）")
    RECORD.write_text(json.dumps(dict(saved=True, modified=changed, removed_actors=[],
                                      backups=[str(p) for p in BACKUPS],
                                      slope_m_per_m=.15 / 2.7), indent=2) + "\n")
    handle = unreal.register_slate_post_tick_callback(resume)


def resume(delta):
    """Restart PIE once the edited editor world has been saved."""
    global handle
    if handle is not None:
        unreal.unregister_slate_post_tick_callback(handle)
        handle = None
    unreal.EditorLevelLibrary.editor_play_simulate()


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle = unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    try:
        apply(0)
    except BaseException:
        raise
