"""Stop PIE and save a fixed PlayerStart for default-start play sessions."""

import json
from pathlib import Path

import unreal


ROOT = Path(__file__).resolve().parents[2]


def configure(delta):
    if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
        return
    if handle is not None:
        unreal.unregister_slate_post_tick_callback(handle)
    world = unreal.EditorLevelLibrary.get_editor_world()
    starts = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.PlayerStart)
    if any(s.get_actor_label() != "Traffic_FixedPlayerStart" for s in starts):
        raise RuntimeError("Inspect existing PlayerStart before changing the spawn origin")
    start = starts[0] if starts else unreal.EditorLevelLibrary.spawn_actor_from_class(
        unreal.PlayerStart, unreal.Vector(5681, 8814, 805), unreal.Rotator(0, 0, 0))
    start.set_actor_label("Traffic_FixedPlayerStart")
    if not unreal.EditorLevelLibrary.save_current_level():
        raise RuntimeError("Fixed PlayerStart map save failed")
    (ROOT/"runs/ue_scene/fixed_play_origin.json").write_text(json.dumps({
        "actor": start.get_path_name(), "ue_position_cm": [5681, 8814, 805],
        "play_location": "DefaultPlayerStart", "saved": True}, indent=2), encoding="utf-8")


handle = None
if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle = unreal.register_slate_post_tick_callback(configure)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    configure(0)
