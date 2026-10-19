"""Read-only check of scene time and PhysX settings after a test fixture."""
import builtins
import json
from pathlib import Path
import unreal

ROOT=Path(__file__).resolve().parents[2]
world=next(w for w in unreal.ObjectIterator(unreal.World)
           if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
physics,expected=getattr(builtins,'node_pair_physics_settings')
actual={k:physics.get_editor_property(k) for k in expected}
time_scale=unreal.GameplayStatics.get_global_time_dilation(world)
expected_scale=getattr(builtins,'node_pair_original_time_dilation')
result=dict(physics_settings_restored=actual==expected,
            time_dilation_restored=abs(time_scale-expected_scale)<1e-6,
            time_dilation=time_scale,expected_time_dilation=expected_scale,
            physics_settings=actual,expected_physics_settings=expected,
            fixture_cleared=getattr(builtins,'node_pair_fleet_fixture',None) is None,
            camera_changed=False,map_changed=False)
result['status']='PASS' if all(result[k] for k in
    ('physics_settings_restored','time_dilation_restored','fixture_cleared')) else 'FAIL'
(ROOT/'runs/ue_node_pairs/restoration_audit.json').write_text(json.dumps(result,indent=2)+'\n')
