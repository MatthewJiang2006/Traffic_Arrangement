"""Restart the test world after a stale AirSim local-frame restoration."""
import json
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
(ROOT/'runs/ue_scene/parking_expansion/recovery_restart.json').write_text(json.dumps(dict(reason='Completed access test; stale pre-reset local poses shifted active trucks',camera_locked=False))+'\n')
unreal.EditorLevelLibrary.editor_end_play()
