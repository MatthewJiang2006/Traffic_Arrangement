"""Read-only enquiry of the editor play helpers available in this UE build."""
import json
import traceback
from pathlib import Path

import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/ue_node_pairs/editor_play_api.json"
out = {}


def grab(key, fn):
    try:
        out[key] = fn()
    except BaseException as exc:
        out[key] = dict(error=repr(exc), traceback=traceback.format_exc())


grab("play_names", lambda: [n for n in dir(unreal.EditorLevelLibrary)
                            if "play" in n.lower() or "pie" in n.lower()])
grab("simulate_names", lambda: [n for n in dir(unreal.EditorLevelLibrary) if "simulate" in n.lower()])
grab("worlds", lambda: [w.get_path_name() for w in unreal.ObjectIterator(unreal.World)])
grab("dirty_packages", lambda: [p.get_name() for p in unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages()])
grab("editor_world", lambda: unreal.EditorLevelLibrary.get_editor_world().get_name())
grab("level_names", lambda: [l.get_name() for l in unreal.EditorLevelLibrary.get_all_level_actors()
                             if False] or [n for n in dir(unreal.EditorLevelLibrary) if "level" in n.lower()])
OUT.write_text(json.dumps(out, indent=2) + "\n")
