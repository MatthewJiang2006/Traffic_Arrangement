"""Temporarily test opening the internal north fence; restore every actor."""
import json,runpy
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
origin=unreal.GameplayStatics.get_world_origin_location(world);changed=[]
try:
    for a in unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor):
        if not a.get_name().startswith('SM_Fence_'):continue
        c,e=a.get_actor_bounds(False);x,y=(c.x+origin.x)/100,(c.y+origin.y)/100
        if -85<x<0 and 303<y<310:
            changed.append((a,a.get_actor_enable_collision()));a.set_actor_enable_collision(False)
    runpy.run_path(str(ROOT/'scripts/ue/plan_warehouse_access.py'))
finally:
    for a,enabled in changed:a.set_actor_enable_collision(enabled)
(ROOT/'runs/ue_scene/station_siting/internal_north_fence_trial.json').write_text(json.dumps(dict(disabled=[a.get_name() for a,_ in changed],restored=True,map_changed=False))+'\n')
