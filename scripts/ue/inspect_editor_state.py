"""Inspect current UE editor save and play state without changing the scene."""

import json
from pathlib import Path

import unreal


ROOT = Path(__file__).resolve().parents[2]
apis = [name for name in dir(unreal) if any(word in name.lower() for word in ("editorlevel", "level_editor", "editorloading", "pie"))]
worlds = [{"path": w.get_path_name(), "name": w.get_name()}
          for w in unreal.ObjectIterator(unreal.World) if "IndustrialArea" in w.get_path_name()]
result = {"apis": apis, "worlds": worlds}
if hasattr(unreal, "EditorLoadingAndSavingUtils"):
    util = unreal.EditorLoadingAndSavingUtils
    result["saving_methods"] = [name for name in dir(util) if any(word in name.lower() for word in ("dirty", "save"))]
    result["dirty_maps"] = [p.get_name() for p in util.get_dirty_map_packages()]
    result["dirty_content"] = [p.get_name() for p in util.get_dirty_content_packages()]
output = ROOT/"runs/ue_scene/editor_state.json"
output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
unreal.log("UE editor state written to " + str(output))
