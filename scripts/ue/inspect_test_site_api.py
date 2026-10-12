import json
from pathlib import Path
import unreal
r=Path(__file__).resolve().parents[2]
d={'gameplay_spawn':[n for n in dir(unreal.GameplayStatics) if 'spawn' in n], 'text_actor':hasattr(unreal,'TextRenderActor'),'text_component':hasattr(unreal,'TextRenderComponent'), 'draw_string':hasattr(unreal.SystemLibrary,'draw_debug_string')}
(r/'runs/ue_scene/test_site_api.json').write_text(json.dumps(d,indent=2))
