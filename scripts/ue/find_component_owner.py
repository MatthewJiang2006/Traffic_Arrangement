"""Locate the PIE actor owning a named collision component."""
import json
from pathlib import Path
import unreal
root=Path(__file__).resolve().parents[2];out=root/'runs/ue_scene/station_siting/component_owner.json'
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
origin=unreal.GameplayStatics.get_world_origin_location(world);rows=[]
for a in unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor):
 for c in a.get_components_by_class(unreal.PrimitiveComponent):
  if 'SM_Leaf_26' in c.get_name():
   l=a.get_actor_location();rows.append({'actor':a.get_name(),'class':a.get_class().get_name(),'component':c.get_name(),'xy_m':[(l.x+origin.x)/100,(l.y+origin.y)/100]})
out.write_text(json.dumps(rows,indent=2)+'\n')
