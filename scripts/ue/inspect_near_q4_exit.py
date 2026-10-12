"""Read-only inventory of foliage and props by the Q4 exit."""
import json
from pathlib import Path
import unreal
root=Path(__file__).resolve().parents[2];out=root/'runs/ue_scene/station_siting/Q4_exit_nearby_actors.json'
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
origin=unreal.GameplayStatics.get_world_origin_location(world);rows=[]
for a in unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor):
 l=a.get_actor_location();x,y=(l.x+origin.x)/100,(l.y+origin.y)/100
 if -91<x<-75 and 90<y<108:
  comps=[c.get_name() for c in a.get_components_by_class(unreal.PrimitiveComponent) if 'Leaf' in c.get_name() or 'Tree' in c.get_name()]
  rows.append({'name':a.get_name(),'xy_m':[round(x,2),round(y,2)],'class':a.get_class().get_name(),'leaf_components':comps})
out.write_text(json.dumps(rows,indent=2)+'\n')
