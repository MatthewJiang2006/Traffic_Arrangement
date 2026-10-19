"""Locate foliage mesh assets behind native collision names."""
import json
from pathlib import Path
import unreal
root=Path(__file__).resolve().parents[2];out=root/'runs/ue_scene/station_siting/leaf_owner.json';rows=[]
for c in unreal.ObjectIterator(unreal.StaticMeshComponent):
 try:
  m=c.get_static_mesh()
  if m and 'SM_Leaf_26' in m.get_name():
   a=c.get_owner()
   rows.append({'component':c.get_path_name(),'owner':a.get_path_name() if a else None,'class':c.get_class().get_name()})
 except Exception:pass
out.write_text(json.dumps(rows,indent=2)+'\n')
