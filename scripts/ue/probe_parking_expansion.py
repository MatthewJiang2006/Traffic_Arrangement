"""Read-only full-body and terrain checks, plus non-colliding parking outlines."""
import json,math,time,hashlib
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
plan=json.loads((ROOT/'config/planning/ue_parking_expansion.json').read_text())
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
origin=unreal.GameplayStatics.get_world_origin_location(world)
actors=unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor)
ignore=[a for a in actors if a.get_class().get_name() in ('TrafficTruckPawn','BP_Truck_C','CameraActor')]

def ground(x,y):
 a=unreal.Vector(x*100-origin.x,y*100-origin.y,1200-origin.z)
 hit=unreal.SystemLibrary.line_trace_single(world,a,unreal.Vector(a.x,a.y,-100-origin.z),unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,False,ignore,unreal.DrawDebugTrace.NONE,True)
 r=hit.to_tuple() if hit else None
 return ((r[5].z+origin.z)/100,r[9].get_name() if r[9] else '') if r and r[0] else (None,'')

rows=[];begin=time.monotonic()
for site in plan['sites']:
 phases={}
 for mode in ('entry_rear_path','exit_rear_path'):
  failures=[];surfaces=set();steps=[];prev=None
  path=site[mode]
  for i,(p,q) in enumerate(zip(path,path[1:])):
   h=math.atan2(q[1]-p[1],q[0]-p[0]);co,si=math.cos(h),math.sin(h)
   x,y=p[0]+1.91425*co,p[1]+1.91425*si
   z,surface=ground(x,y);surfaces.add(surface)
   if z is None or not .7<z<1.35:
    failures.append(dict(index=i,reason='unsupported terrain',xy_m=[x,y],z=z));continue
   if prev is not None and abs(z-prev)>.035:steps.append(dict(index=i,xy_m=[x,y],rise_m=z-prev))
   prev=z
   center=unreal.Vector(x*100-origin.x,y*100-origin.y,(z+2.35)*100-origin.z)
   hit=unreal.SystemLibrary.box_trace_single(world,center,center,unreal.Vector(452.2,165,195),unreal.Rotator(pitch=0,yaw=math.degrees(h),roll=0),unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,False,ignore,unreal.DrawDebugTrace.NONE,True)
   r=hit.to_tuple() if hit else None
   if r and r[0]:failures.append(dict(index=i,reason='static body envelope overlap',actor=r[9].get_name() if r[9] else '',xy_m=[x,y]))
   for dx,dy in ((-4.172,-1.3),(-4.172,1.3),(4.172,-1.3),(4.172,1.3)):
    gz,_=ground(x+dx*co-dy*si,y+dx*si+dy*co)
    if gz is None or abs(gz-z)>.15:
     failures.append(dict(index=i,reason='uneven footprint ground',xy_m=[x,y],center_z=z,corner_z=gz));break
  phases[mode]=dict(status='PASS' if not failures and not steps else 'FAIL',failures=failures[:20],failure_count=len(failures),terrain_steps=steps[:20],terrain_step_count=len(steps),ground_surfaces=sorted(surfaces))
 x,y=site['xy_m'];z,surface=ground(x,y);yaw=math.radians(site['heading_deg']);co,si=math.cos(yaw),math.sin(yaw)
 color=unreal.LinearColor(.1,.8,1.,1.)
 corners=[unreal.Vector((x+dx*co-dy*si)*100-origin.x,(y+dx*si+dy*co)*100-origin.y,(z+.05)*100-origin.z) for dx,dy in ((-4.5,-1.65),(4.5,-1.65),(4.5,1.65),(-4.5,1.65))]
 for a,b in zip(corners,corners[1:]+corners[:1]):unreal.SystemLibrary.draw_debug_line(world,a,b,color,86400,4)
 unreal.SystemLibrary.draw_debug_string(world,unreal.Vector(x*100-origin.x,y*100-origin.y,(z+1.0)*100-origin.z),site['id'],None,color,86400)
 rows.append(dict(id=site['id'],xy_m=site['xy_m'],ground_z_m=z,surface_actor=surface,phases=phases,status='PASS' if all(r['status']=='PASS' for r in phases.values()) else 'FAIL'))
map_sha256={name:hashlib.sha256((Path('/home/song03/Traffic_ue4/Content/IndustrialArea/Map')/name).read_bytes()).hexdigest() for name in plan['map_sha256']}
result=dict(map_sha256=map_sha256,sites_sha256=hashlib.sha256(json.dumps(plan['sites'],sort_keys=True).encode()).hexdigest(),status='PASS' if all(r['status']=='PASS' for r in rows) else 'FAIL',rows=rows,elapsed_s=time.monotonic()-begin,read_only=True,map_saved=False,camera_changed=False)
(ROOT/'runs/ue_scene/station_siting/parking_expansion_probe.json').write_text(json.dumps(result,indent=2)+'\n')
