"""Place PIE debug labels and markers without camera control or map mutation."""
import json
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
(ROOT/'runs/ue_scene/test_sites').mkdir(parents=True,exist_ok=True)
config=json.loads((ROOT/'config/traffic/industrialarea_ue_test_sites.json').read_text(encoding='utf-8'))
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
origin=unreal.GameplayStatics.get_world_origin_location(world)
ignore=[a for a in unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor) if a.get_class().get_name() in ('TrafficTruckPawn','BP_Truck_C')]
rows=[]
for label,site in config['sites'].items():
 for role in ('start','finish'):
  p=site[role];x,y=p['xy_m'];loc=unreal.Vector(x*100-origin.x,y*100-origin.y,500-origin.z)
  hit=unreal.SystemLibrary.line_trace_single(world,loc,unreal.Vector(loc.x,loc.y,-1000-origin.z),unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,False,ignore,unreal.DrawDebugTrace.NONE,True)
  values=hit.to_tuple() if hit else None
  if not values or not values[0] or not values[9].get_name().startswith(('SM_Road_','Traffic_SeamBridge')):raise RuntimeError(label+' '+role+' not on road')
  z=(values[5].z+origin.z)/100
  name='Traffic_TestSite_'+label+'_'+role
  text=label+' '+role.upper()
  color=unreal.LinearColor(0.1,1,0.1,1) if role=='start' else unreal.LinearColor(1,.3,.05,1)
  point=unreal.Vector(x*100-origin.x,y*100-origin.y,(z+.12)*100-origin.z)
  unreal.SystemLibrary.draw_debug_cylinder(world,point,point+unreal.Vector(0,0,50),70,24,color,86400,5)
  unreal.SystemLibrary.draw_debug_string(world,point+unreal.Vector(0,0,160),text,None,color,86400)
  rows.append({'site':label,'role':role,'xy_m':[x,y],'surface_z_m':z,'road_actor':values[9].get_name(),'marker_label':name})
(ROOT/'runs/ue_scene/test_sites/placement.json').write_text(json.dumps({'status':'PASS','markers':rows,'map_saved':False,'camera_changed':False},indent=2)+'\n')
