"""Zero only requested test chassis velocities before reversible fixture teleports."""
import json
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
request=json.loads((ROOT/'runs/ue_scene/test_sites/motion_stop_request.json').read_text())
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
rows=[]
for actor in unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor):
 if actor.get_name() not in request['vehicles']:continue
 mesh=actor.get_component_by_class(unreal.SkeletalMeshComponent)
 mesh.set_all_physics_linear_velocity(unreal.Vector(0,0,0),False)
 mesh.set_all_physics_angular_velocity_in_degrees(unreal.Vector(0,0,0),False)
 rows.append(actor.get_name())
(ROOT/'runs/ue_scene/test_sites/motion_stop.json').write_text(json.dumps({'vehicles':rows})+'\n')
