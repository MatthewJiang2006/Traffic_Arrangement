"""Temporarily hide idle PIE trucks and restore their original visibility."""
import builtins
import json
from pathlib import Path
import unreal

ROOT=Path(__file__).resolve().parents[2]
folder=ROOT/'runs/ue_scene/test_sites'
request=json.loads((folder/'idle_visibility_request.json').read_text())
state=getattr(builtins,'traffic_idle_visibility',None)
if request.get('action')=='RESTORE':
    count=0
    for actor,hidden in state or []:
        actor.set_actor_hidden_in_game(hidden);count+=1
    builtins.traffic_idle_visibility=None
    result=dict(status='RESTORED',actors=count)
else:
    if state is not None:raise RuntimeError('Idle visibility fixture already active')
    world=next(w for w in unreal.ObjectIterator(unreal.World)
               if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
    actors=unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor)
    idle={a.get_name() for a in actors if a.get_class().get_name()=='TrafficTruckPawn'
          and a.get_name() not in request['active_vehicles']}
    saved=[]
    for actor in actors:
        parent=actor.get_attach_parent_actor()
        if actor.get_name() in idle or (parent and parent.get_name() in idle):
            saved.append((actor,actor.get_editor_property('hidden')));actor.set_actor_hidden_in_game(True)
    builtins.traffic_idle_visibility=saved
    result=dict(status='HIDDEN',idle_vehicles=sorted(idle),actors=len(saved),camera_changed=False,map_saved=False)
(folder/'idle_visibility_status.json').write_text(json.dumps(result)+'\n')
