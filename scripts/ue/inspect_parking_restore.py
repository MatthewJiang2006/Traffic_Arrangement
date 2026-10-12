import json
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
rows=[]
cache=globals().get('state',{})
for a in unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor):
 if a.get_name() not in ('Car_0_0','Car_0_1','Car_0_2'):continue
 mesh=a.get_component_by_class(unreal.SkeletalMeshComponent)
 row=dict(name=a.get_name(),actor=a.get_actor_location().to_tuple(),mesh=mesh.get_world_location().to_tuple(),relative=mesh.get_editor_property('relative_location').to_tuple(),physics=mesh.is_simulating_physics())
 if isinstance(cache,dict) and a.get_name() in cache:row['cached_pose']=cache[a.get_name()]['pose'].translation.to_tuple()
 rows.append(row)
(ROOT/'runs/ue_scene/parking_expansion/restore_inspection.json').write_text(json.dumps(rows,indent=2)+'\n')
