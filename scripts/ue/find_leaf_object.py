"""Search loaded actors for the native SM_Leaf_26 collision source."""
import json
from pathlib import Path
import unreal
root=Path(__file__).resolve().parents[2];out=root/'runs/ue_scene/station_siting/leaf_object.json';rows=[]
for obj in unreal.ObjectIterator(unreal.Actor):
 if obj.get_name()!='SM_Leaf_26':continue
 try:
  l=obj.get_actor_location();b=obj.get_actor_bounds(False);e=b[1] if isinstance(b,tuple) else b.to_tuple()[1]
  rows.append({'path':obj.get_path_name(),'location_cm':[l.x,l.y,l.z],'extent_m':[e.x/100,e.y/100,e.z/100],'collision':obj.get_actor_enable_collision()})
 except Exception as exc:rows.append({'path':obj.get_path_name(),'error':str(exc)})
out.write_text(json.dumps(rows,indent=2)+'\n')
