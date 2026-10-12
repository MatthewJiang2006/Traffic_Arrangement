"""Read-only inventory of tiny decorative leaf actors near the Q4 merge."""
import json
from pathlib import Path
import unreal
root=Path(__file__).resolve().parents[2];out=root/'runs/ue_scene/station_siting/Q4_exit_leaves.json';rows=[]
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
origin=unreal.GameplayStatics.get_world_origin_location(world)
for a in unreal.ObjectIterator(unreal.Actor):
 if not a.get_path_name().startswith(world.get_path_name()+':') or not a.get_name().startswith('SM_Leaf_'):continue
 l=a.get_actor_location();x,y=(l.x+origin.x)/100,(l.y+origin.y)/100
 if -90<x<-78 and 90<y<110:
  b=a.get_actor_bounds(False);e=b[1] if isinstance(b,tuple) else b.to_tuple()[1]
  rows.append({'name':a.get_name(),'xy_m':[round(x,2),round(y,2)],'size_m':[round(2*e.x/100,2),round(2*e.y/100,2),round(2*e.z/100,2)]})
out.write_text(json.dumps(rows,indent=2)+'\n')
