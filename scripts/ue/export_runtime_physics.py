"""Read the active PIE clock and physics frame limits without changing them."""
import json
from pathlib import Path
import unreal
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
physics=unreal.get_default_object(unreal.load_class(None,'/Script/Engine.PhysicsSettings'))
fields=('MaxPhysicsDeltaTime','bSubstepping','MaxSubstepDeltaTime','MaxSubsteps')
result=dict(world=world.get_path_name(),game_time_s=unreal.GameplayStatics.get_time_seconds(world),time_dilation=unreal.GameplayStatics.get_global_time_dilation(world),physics={k:physics.get_editor_property(k) for k in fields})
(Path(__file__).resolve().parents[2]/'runs/ue_scene/runtime_physics_before_round41_v5.json').write_text(json.dumps(result,indent=2)+'\n')
