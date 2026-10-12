"""Read-only full-body static clearance and wheel-road checks for candidate paths."""
import json,math
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'runs/ue_scene/test_sites'
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
origin=unreal.GameplayStatics.get_world_origin_location(world)
actors=unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor)
ignore=[a for a in actors if a.get_class().get_name() in ('TrafficTruckPawn','BP_Truck_C')]
truck=next(a for a in ignore if a.get_name()=='Car_0_0')
visual=next(a for a in ignore if a.get_class().get_name()=='BP_Truck_C' and a.get_attach_parent_actor()==truck)
transform=truck.get_actor_transform();bodies=[];wheels=[]
for c in visual.get_components_by_class(unreal.StaticMeshComponent):
 if c.get_name() in ('Truck_Body_00','Truck_Body_01','TrafficShortCargo'):
  bounds=c.get_editor_property('static_mesh').get_bounds();center=transform.inverse_transform_location(c.get_world_transform().transform_location(bounds.origin));scale=c.get_world_scale();half=bounds.box_extent
  bodies.append((c.get_name(),center,unreal.Vector(half.x*abs(scale.x),half.y*abs(scale.y),half.z*abs(scale.z))))
 elif c.get_name().startswith('Wheel_'):wheels.append(transform.inverse_transform_location(c.get_world_location()))

def pos(p,local,yaw):
 co,si=math.cos(yaw),math.sin(yaw)
 # Path points reference rear axle; Actor is half a wheelbase forward.
 xx=p[0]+1.16425*co;yy=p[1]+1.16425*si
 return unreal.Vector(xx*100+local.x*co-local.y*si-origin.x,yy*100+local.x*si+local.y*co-origin.y,89.4+local.z-origin.z)

result={}
for site,candidates in json.loads((OUT/'path_candidates.json').read_text()).items():
 results=[]
 for number,candidate in enumerate(candidates):
  path=candidate['rear_path'];bad=[]
  for i in range(0,len(path)-1,3):
   p,q=path[i],path[i+1];yaw=math.atan2(q[1]-p[1],q[0]-p[0])
   for local in [unreal.Vector(w.x+dx,w.y+dy,w.z) for w in wheels for dx,dy in [(0,0),(.0,25),(.0,-25),(25,0),(-25,0)]]:
    v=pos(p,local,yaw);hit=unreal.SystemLibrary.line_trace_single(world,v+unreal.Vector(0,0,300),v-unreal.Vector(0,0,1000),unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,False,ignore,unreal.DrawDebugTrace.NONE,True)
    a=hit.to_tuple() if hit else None;road=a[9].get_name() if a and a[0] and a[9] else ''
    if not road.startswith(('SM_Road_','Traffic_SeamBridge')) or abs((a[5].z+origin.z)/100-1.1)>.03:bad.append({'index':i,'reason':'wheel outside asphalt','road':road});break
   if bad:break
   for name,local,half in bodies:
    v=pos(p,local,yaw);hit=unreal.SystemLibrary.box_trace_single(world,v,v,half,unreal.Rotator(pitch=0,yaw=math.degrees(yaw),roll=0),unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,False,ignore,unreal.DrawDebugTrace.NONE,True)
    a=hit.to_tuple() if hit else None
    if a and a[0]:bad.append({'index':i,'reason':'static body overlap '+name,'actor':a[9].get_name() if a[9] else ''});break
   if bad:break
  results.append({'candidate':number,'pass':not bad,'failure':bad})
 result[site]=results
(OUT/'path_probe.json').write_text(json.dumps(result,indent=2)+'\n')
