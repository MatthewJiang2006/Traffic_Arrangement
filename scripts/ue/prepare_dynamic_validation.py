"""Keep PIE physics running when the user switches away from the UE editor."""

import json
from pathlib import Path

import unreal


ROOT = Path(__file__).resolve().parents[2]
settings_class = unreal.load_class(None, "/Script/UnrealEd.EditorPerformanceSettings")
settings = unreal.get_default_object(settings_class)
before = settings.get_editor_property("bThrottleCPUWhenNotForeground")
settings.set_editor_property("bThrottleCPUWhenNotForeground", False)
path = ROOT/"runs/ue_scene/dynamic_editor_settings.json"
path.write_text(json.dumps({"previous_throttle_cpu_when_not_foreground": before,
                           "current_throttle_cpu_when_not_foreground": False}, indent=2), encoding="utf-8")
unreal.log("Disabled background editor throttling for dynamic validation")
