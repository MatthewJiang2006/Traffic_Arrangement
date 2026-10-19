"""Restart PIE so the node-pair fixture runs against a pristine vehicle fleet."""
import json
from pathlib import Path

import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/ue_node_pairs/pie_restart.json"
handle = None


def resume(delta):
    global handle
    if handle is not None:
        unreal.unregister_slate_post_tick_callback(handle)
        handle = None
    unreal.EditorLevelLibrary.editor_play_simulate()


def restart(delta):
    global handle
    if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
        return
    if handle is not None:
        unreal.unregister_slate_post_tick_callback(handle)
    world = unreal.EditorLevelLibrary.get_editor_world()
    OUT.write_text(json.dumps(dict(editor_world=world.get_name() if world else None,
                                   restart=True), indent=2) + "\n")
    handle = unreal.register_slate_post_tick_callback(resume)


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle = unreal.register_slate_post_tick_callback(restart)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    OUT.write_text(json.dumps(dict(editor_world=None, restart=True), indent=2) + "\n")
    handle = unreal.register_slate_post_tick_callback(resume)
