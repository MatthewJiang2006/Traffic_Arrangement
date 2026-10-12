#!/usr/bin/env python3
"""Validate terminal markers and actual U-turns; preserve the user's camera."""
import argparse,hashlib,json,math,sys,time
from pathlib import Path
import airsim
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.adapters.airsim import AirSimAdapter,AirSimConfig
from logistics_park.common.messages import ControlCommand
from logistics_park.map.map_loader import load_map
from logistics_park.scheduling.road_capacity import RoadCapacity
from logistics_park.scheduling.fcfs import FCFSScheduler,EntranceRequest
from logistics_park.runtime.body_observer import FileBodyObserver
from logistics_park.runtime.speed_limits import SpeedLimits,bend_stations
from ue_junction_check import teleport
from ue_dynamic_crossing_check import absolute_ue
from ue_trailer_driving_check import ue_script
from ue_uturn_paths import WHEELBASE,line,make_paths
OUT=ROOT/'runs/ue_scene/test_sites';NAMES=('Car_0_0','Car_0_1')
RESULTS=OUT
LIMITS=SpeedLimits(**json.loads((ROOT/'config/runtime/ue_speed_limits.json').read_text(encoding='utf-8')))

def cumulative(path):
 stations=[0.]
 for a,b in zip(path,path[1:]):stations.append(stations[-1]+math.dist(a,b))
 return stations

def point_at(path,ss,s):
 for i in range(len(ss)-1):
  if ss[i+1]>=s:
   t=max(0,min(1,(s-ss[i])/(ss[i+1]-ss[i])));a,b=path[i],path[i+1];return [a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])]
 return path[-1]

def project_local(p,path,ss,previous):
 best=(float('inf'),previous)
 for i,(a,b) in enumerate(zip(path,path[1:])):
  if ss[i+1]<previous-.6 or ss[i]>previous+3:continue
  dx,dy=b[0]-a[0],b[1]-a[1];length2=dx*dx+dy*dy
  if length2<1e-12:continue
  f=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length2));x,y=a[0]+f*dx,a[1]+f*dy
  candidate=(math.hypot(p[0]-x,p[1]-y),ss[i]+f*(ss[i+1]-ss[i]))
  if candidate[0]<best[0]:best=candidate
 return best

def intersects(a,junction):
 cx,cy=junction['center_m'];hx,hy=junction['half_extent_m']
 for b in a['bodies']:
  x,y=b['centre_ue_m'][:2];ex,ey=b['half_size_m'][:2];yaw=math.radians(b['yaw_deg'])
  for angle in (0,math.pi/2,yaw,yaw+math.pi/2):
   ux,uy=math.cos(angle),math.sin(angle)
   r=hx*abs(ux)+hy*abs(uy)+ex*abs(math.cos(yaw)*ux+math.sin(yaw)*uy)+ey*abs(-math.sin(yaw)*ux+math.cos(yaw)*uy)
   if abs((x-cx)*ux+(y-cy)*uy)>=r:break
  else:return True
 return False

def zero_motion(c,names):
 (OUT/'motion_stop_request.json').write_text(json.dumps({'vehicles':list(names)})+'\n')
 ue_script(c,'stop_test_vehicle_motion.py',OUT/'motion_stop.json')


def run_case(c,site,mode,plan,origin,config,survey):
 names=NAMES if mode=='two' else NAMES[:1];label=site+'_'+mode
 result={'case':label,'status':'FAIL','failure':[],'samples':[],'path':plan,'finished':[],
         'speed_limits':vars(LIMITS)}
 paths={n:[list(p) for p in plan['rear_path']] for n in names}
 if mode=='two':
  p=paths[names[0]];ss=cumulative(p);start=point_at(p,ss,12)
  paths[names[0]]=[start]+[q for q,s in zip(p,ss) if s>12+.01]
  p=paths[names[0]];a,b=p[-2:];yaw=math.atan2(b[1]-a[1],b[0]-a[0]);end=[b[0]+12*math.cos(yaw),b[1]+12*math.sin(yaw)]
  paths[names[0]]=p[:-1]+line(b,end)
 ss={n:cumulative(paths[n]) for n in names};progress={n:0. for n in names};sequence={n:0 for n in names}
 # Detect bends from the actual path, so faster straight travel brakes before
 # a U-turn rather than waiting for the steering command to grow.
 bends={n:bend_stations(paths[n],ss[n]) for n in names}
 turn_start={}
 for n in names:
  points=paths[n]
  first=next((i for i,p in enumerate(points) if abs(p[1]-points[0][1])>1e-6),len(points)-1)
  turn_start[n]=ss[n][max(0,first-1)] if site in ('B','D') else 0
 zero_motion(c,names)
 for n in names:
  a,b=paths[n][:2];yaw=math.atan2(b[1]-a[1],b[0]-a[0]);xy=[a[0]+WHEELBASE/2*math.cos(yaw),a[1]+WHEELBASE/2*math.sin(yaw)]
  teleport(c,n,xy,math.degrees(yaw),origin);c.enableApiControl(False,vehicle_name=n)
 time.sleep(.4);c.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/monitor_following.py'));time.sleep(.4)
 observer=FileBodyObserver(ROOT/'runs/ue_scene',prefix='following',max_age_s=1.5)
 adapter=AirSimAdapter(AirSimConfig(tuple(names),origin),RESULTS/label/'adapter.jsonl')
 junction=plan['junction'];box=survey['junctions'][junction];connector='uturn_'+site
 incoming=config['sites'][site]['start']['edge_id'];outgoing=config['sites'][site]['finish']['edge_id']
 # B/D arrive on the site's finish lane and leave on its start lane.
 if site in ('B','D'):incoming,outgoing=outgoing,incoming
 elif site=='A':incoming='road_gate_south_0_in_j_0_0'
 else:incoming='road_gate_north_0_in_j_3_0'
 capacity=RoadCapacity(load_map(ROOT/'config/topology/industrialarea_ue.json'));capacity.capacities[connector]=1
 scheduler=FCFSScheduler((junction,),capacity)
 entered=set();cleared=set();finished=set();wait_stationary=0.;max_inside=0;max_error=0.;last_station={n:(0.,time.monotonic()) for n in names}
 started=time.monotonic();last_print=started;last_tick=started
 try:
  adapter.open();baseline={n:adapter.telemetry[n]['collision']['time_stamp'] for n in names}
  for i,n in enumerate(names):scheduler.request(EntranceRequest(n,junction,incoming,connector,outgoing,0,12 if i==0 and mode=='two' else 0))
  while time.monotonic()-started<210:
   now=time.monotonic();t=now-started;dt=min(.3,now-last_tick);last_tick=now
   states=adapter.read_states();audits={};errors=[];inside=[]
   for n in names:
    a,e=observer.read(n);audits[n]=a;errors.extend(n+': '+v for v in e)
    if intersects(a,box):inside.append(n)
    hit=adapter.telemetry[n]['collision']
    if hit.get('has_collided') and hit.get('time_stamp')!=baseline[n]:errors.append(n+' chassis collision '+str(hit.get('object_name')))
   def outlet_clear(edge,vehicle):
    return all(not intersects(audits[n],box) for n in names if n!=vehicle and n in entered) and all(math.dist([states[vehicle].x,states[vehicle].y],[states[n].x,states[n].y])>12 for n in names if n!=vehicle and n in cleared)
   scheduler.advance(t,can_enter=outlet_clear)
   for n in names:
    if n in inside:
     if not scheduler.allows(n,junction,connector):errors.append(n+' unauthorized junction entry')
     elif n not in entered:scheduler.mark_entered(junction,n,t);entered.add(n)
    if n in entered and n not in cleared and n not in inside:
     scheduler.complete(junction,n,t);cleared.add(n)
   max_inside=max(max_inside,len(inside))
   if len(inside)>1:errors.append('multiple trucks in terminal junction')
   frame={}
   for n in names:
    state=states[n];rear=[state.x-WHEELBASE/2*math.cos(state.heading),state.y-WHEELBASE/2*math.sin(state.heading)]
    error,station=project_local(rear,paths[n],ss[n],progress[n]);progress[n]=max(progress[n],station)
    max_error=max(max_error,error)
    if error>.7:errors.append(n+' rear-path deviation '+str(round(error,3)))
    if station>last_station[n][0]+.2:last_station[n]=(station,now)
    allowed=n in cleared or scheduler.allows(n,junction,connector)
    target=point_at(paths[n],ss[n],station+1.65)
    alpha=(math.atan2(target[1]-rear[1],target[0]-rear[0])-state.heading+math.pi)%(2*math.pi)-math.pi
    delta=math.atan2(2*WHEELBASE*math.sin(alpha),max(.25,math.dist(target,rear)));delta=max(-math.radians(40),min(math.radians(40),delta))
    if site in ('B','D') and station<turn_start[n]-.02:delta=0
    remaining=ss[n][-1]-station
    speed=LIMITS.straight_m_s if abs(delta)<.15 else LIMITS.uturn_m_s
    upcoming=next((s for s in bends[n] if s>=station-.3),None)
    if upcoming is not None:speed=min(speed,LIMITS.approach(LIMITS.uturn_m_s,upcoming-station))
    speed=min(speed,math.sqrt(max(0,1.2*remaining)),remaining*.8)
    if not allowed:
     last_station[n]=(station,now)  # Waiting is not an execution stall.
     # Hold well outside the empty-junction boundary, including cab overhang and a 7.2 m maneuver buffer.
     cx,cy=box['center_m'];hx,hy=box['half_extent_m']
     if site=='A':distance=cy-hy-state.y-10.5
     elif site=='C':distance=state.y-(cy+hy)-10.5
     else:distance=cx-hx-state.x-10.5
     speed=min(speed,max(0,distance)*.6)
     if abs(state.speed)<.03:wait_stationary+=dt
    if n in finished:speed=0
    elif remaining<.22 and abs(state.speed)<.08:
     speed=0;finished.add(n)
    if allowed and n not in finished and now-last_station[n][1]>18:errors.append(n+' no progress')
    omega=speed*math.tan(delta)/WHEELBASE if speed>.05 else 0
    frame[n]={'station':station,'error':error,'speed':state.speed,'xy':[state.x,state.y],'heading_deg':math.degrees(state.heading),'target_speed':speed,'inside':n in inside,'requested_steering_deg':math.degrees(delta),'turn_start_station':turn_start[n]}
    frame[n]['command']=(speed,omega)
   if errors:raise RuntimeError('; '.join(errors))
   for n in names:
    speed,omega=frame[n].pop('command');adapter.send(ControlCommand(n,sequence[n],adapter.clock(),speed,omega,ttl=.8));sequence[n]+=1
   result['samples'].append({'t_s':t,'vehicles':frame})
   if now-last_print>8:
    print(label, 'progress', {n:round(progress[n],1) for n in names},'inside',inside,'finished',list(finished),flush=True);last_print=now
   if len(finished)==len(names):break
   time.sleep(.1)
  else:raise RuntimeError('case timeout')
  if len(cleared)!=len(names):raise RuntimeError('not all bodies cleared the junction')
  if mode=='two' and wait_stationary<.5:raise RuntimeError('second truck did not demonstrate stationary waiting')
  result['status']='PASS'
 except BaseException as e:
  result['failure'].append(repr(e));adapter.fail_stop('terminal U-turn '+repr(e))
  if isinstance(e,(KeyboardInterrupt,SystemExit)):raise
 finally:
  adapter.close();c.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/stop_following.py'))
  result.update(finished=sorted(finished),cleared=sorted(cleared),maximum_simultaneous_junction_occupancy=max_inside,stationary_wait_s=wait_stationary,max_rear_path_error_m=max_error,events=scheduler.events,close_result=adapter.close_result)
  folder=RESULTS/label;folder.mkdir(parents=True,exist_ok=True);(folder/'acceptance.json').write_text(json.dumps(result,indent=2)+'\n')
 print(label,result['status'],result['failure'],flush=True)
 return result

def main():
 global RESULTS
 parser=argparse.ArgumentParser();parser.add_argument('--site',action='append',choices=list('ABCD'));parser.add_argument('--single-only',action='store_true');parser.add_argument('--two-only',action='store_true');parser.add_argument('--result-directory',type=Path,default=OUT);args=parser.parse_args()
 RESULTS=args.result_directory;RESULTS.mkdir(parents=True,exist_ok=True)
 OUT.mkdir(parents=True,exist_ok=True)
 c=airsim.CarClient(timeout_value=5);names=c.listVehicles()
 if c.simIsPause() or any(abs(c.getCarState(vehicle_name=n).speed)>.01 for n in names):raise RuntimeError('stationary unpaused PIE required')
 origin=tuple(v/100 for v in ue_script(c,'export_scene.py',ROOT/'runs/ue_scene/scene.json')['world_origin_cm'])
 homes={}
 for n in names:
  pos,pose=absolute_ue(c,n,origin);homes[n]={'position':pos,'yaw_deg':math.degrees(airsim.to_eularian_angles(pose.orientation)[2])}
 (OUT/'homes.json').write_text(json.dumps(homes,indent=2)+'\n')
 api={n:c.isApiControlEnabled(vehicle_name=n) for n in names}
 settings=Path.home()/'Documents/AirSim/settings.json';digest=hashlib.sha256(settings.read_bytes()).hexdigest()
 config=json.loads((ROOT/'config/traffic/industrialarea_ue_test_sites.json').read_text());survey=json.loads((ROOT/'config/topology/industrialarea_ue_survey.json').read_text())
 candidates=make_paths(survey)
 (OUT/'path_candidates.json').write_text(json.dumps(candidates,indent=2)+'\n')
 probe=ue_script(c,'probe_uturn_paths.py',OUT/'path_probe.json')
 summary={'status':'FAIL','cases':[]}
 try:
  c.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/prepare_dynamic_validation.py'))
  for site in args.site or list('ABCD'):
   passed=[r for r in probe[site] if r['pass']]
   if not passed:raise RuntimeError('no preflight-clear path for '+site)
   preferred=sorted(passed,key=lambda r:(abs(candidates[site][r['candidate']]['radius_m']-3.8),candidates[site][r['candidate']].get('length_m',0)))
   plan=candidates[site][preferred[0]['candidate']]
   modes=['single'] if args.single_only else ['two'] if args.two_only else ['single','two']
   for mode in modes:
    result=run_case(c,site,mode,plan,origin,config,survey)
    summary['cases'].append({k:v for k,v in result.items() if k not in ['samples','path']})
    (RESULTS/'latest_acceptance.json').write_text(json.dumps(summary,indent=2)+'\n')
    if result['status']!='PASS':raise RuntimeError(site+' '+mode+' failed')
  summary['status']='PASS'
 except Exception as e:summary['failure']=repr(e)
 finally:
  c.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/stop_following.py'))
  for n in NAMES:
   c.enableApiControl(True,vehicle_name=n);c.setCarControls(airsim.CarControls(brake=1),vehicle_name=n)
  zero_motion(c,NAMES)
  c.simPause(True)
  try:
   for n in NAMES:teleport(c,n,homes[n]['position'][:2],homes[n]['yaw_deg'],origin)
   zero_motion(c,NAMES)
  finally:c.simPause(False)
  time.sleep(.8)
  for n in names:c.enableApiControl(api[n],vehicle_name=n)
  summary['fleet_restored']=all(math.dist(absolute_ue(c,n,origin)[0][:2],home['position'][:2])<.1 for n,home in homes.items())
  summary['settings_unchanged']=hashlib.sha256(settings.read_bytes()).hexdigest()==digest
  summary['camera_changed']=False
  if not summary['fleet_restored'] or not summary['settings_unchanged']:summary['status']='FAIL'
  (RESULTS/'latest_acceptance.json').write_text(json.dumps(summary,indent=2)+'\n')
 print('Test sites acceptance',summary['status'],flush=True)
 return 0 if summary['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
