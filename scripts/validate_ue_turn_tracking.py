#!/usr/bin/env python3
"""Verify the repaired right corner with measured UE truck physics."""
import argparse,hashlib,json,math,statistics,sys,time
from pathlib import Path
import airsim
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from logistics_park.adapters.airsim import AirSimAdapter,AirSimConfig
from logistics_park.common.messages import ControlCommand
from logistics_park.runtime.transport_routes import TransportMap,RouteGeometry
from logistics_park.runtime.body_observer import FleetFileObserver
from validate_ue_node_pairs import rear
from validate_ue_loading_access import map_fingerprints
from ue_trailer_driving_check import ue_script
from ue_dynamic_crossing_check import absolute_ue
NAME='Car_5_1'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,default=ROOT/'runs/right_turn_tracking_native_run3')
parser.add_argument('--edge',default='turn_j_3_0_N_E_right_single')
args=parser.parse_args();OUT=args.output.resolve()
if (OUT/'acceptance.json').exists():raise RuntimeError('Choose a new output directory to preserve prior evidence')
OUT.mkdir(parents=True,exist_ok=True)
c=airsim.CarClient(timeout_value=5);names=c.listVehicles()
if c.simIsPause() or any(abs(c.getCarState(vehicle_name=n).speed)>.1 for n in names):raise RuntimeError('Stationary unpaused fleet required')
scene=ue_script(c,'export_scene.py',ROOT/'runs/ue_scene/scene.json');origin=[v/100 for v in scene['world_origin_cm']]
home={n:absolute_ue(c,n,origin)[0] for n in names};poses={n:c.simGetVehiclePose(vehicle_name=n) for n in names};apis={n:c.isApiControlEnabled(vehicle_name=n) for n in names};maps=map_fingerprints()
def fixture(action,vehicles=None):
 (ROOT/'runs/ue_node_pairs/fleet_request.json').write_text(json.dumps(dict(action=action,vehicles=vehicles or names,time_scale=1,output_dir=str(OUT))))
 return ue_script(c,'node_pair_fleet_fixture.py',OUT/'fleet_status.json')
def console(code):
 if not c.simRunConsoleCommand(code):raise RuntimeError('Rejected UE command')
report=dict(status='RUNNING',cases=[],camera_changed=False,map_changed=False,initial_fleet_ue_m=home,source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'src/logistics_park/runtime/transport_routes.py',ROOT/'config/topology/industrialarea_ue_internal_waiting_transport.json')});adapter=None
try:
 fixture('INIT')
 model=TransportMap(ROOT/'config/topology/industrialarea_ue_internal_waiting_transport.json')
 keys=sorted([k for k,e in model.edges.items() if e['capacity_alias']==args.edge],key=lambda k:int(k.rsplit('test',1)[1]))
 start=model.edges[keys[0]]['centerline'][0]
 model.edges['probe_approach']=dict(edge_id='probe_approach',kind='road',speed_limit=2.2,centerline=[[start[0],start[1]+5],start],capacity_alias='probe_approach')
 model.aliases['probe_approach']='probe_approach'
 end=model.edges[keys[-1]]['centerline'][-1]
 model.edges['probe_exit']=dict(edge_id='probe_exit',kind='road',speed_limit=2.2,centerline=[end,[end[0]+12,end[1]]],capacity_alias='probe_exit')
 model.aliases['probe_exit']='probe_exit'
 plan=RouteGeometry(model,['probe_approach']+keys+['probe_exit'],[],'probe',[])
 for case,target in enumerate((1.5,2.2,2.2)):
  if adapter:adapter.close();adapter=None
  fixture('FREEZE',[NAME]);c.simPause(True)
  x,y=plan.point(0);x+=(0,.15,-.15)[case];h=plan.heading(0)+(0,.05,-.05)[case]
  local=poses[NAME].position;world=home[NAME];px=x+1.16425*math.cos(h);py=y+1.16425*math.sin(h)
  pose=airsim.Pose(airsim.Vector3r(local.x_val+px-world[0],local.y_val+py-world[1],local.z_val-.894+world[2]),airsim.to_quaternion(0,0,h))
  fixture('ACTIVATE',[NAME]);c.enableApiControl(True,vehicle_name=NAME);c.simSetVehiclePose(pose,True,vehicle_name=NAME);c.setCarControls(airsim.CarControls(brake=1),vehicle_name=NAME)
  placed=absolute_ue(c,NAME,origin)[0]
  if math.dist(placed[:2],(px,py))>.1:raise RuntimeError('Incorrect calibration staged pose '+str(placed))
  stopdir=ROOT/'runs/ue_scene/test_sites';(stopdir/'motion_stop_request.json').write_text(json.dumps(dict(vehicles=[NAME])));ue_script(c,'stop_test_vehicle_motion.py',stopdir/'motion_stop.json');c.simPause(False);time.sleep(.2)
  console('py import builtins,runpy; builtins.fleet_monitor_request='+repr(dict(vehicles=[NAME],file='steering_calibration_live.json'))+'; runpy.run_path('+repr(str(ROOT/'scripts/ue/monitor_fleet.py'))+')')
  obs=FleetFileObserver(ROOT/'runs/ue_scene/steering_calibration_live.json')
  adapter=AirSimAdapter(AirSimConfig(vehicle_ids=(NAME,),origin_ue_m=tuple(origin),command_ttl_s=2,allow_reverse=True,speed_rise_m_s2=.6,speed_error_gain=.6),OUT/('adapter_'+str(case)+'.jsonl'),watchdog=False);adapter.open();adapter.snapshot_provider=obs.snapshot;adapter.last_source={};adapter.source_advanced={}
  initial=None;previous=None;samples=[];curvatures=[];travel=0;progress=0.;peak_error=0.;begin=time.monotonic();seq=1;baseline=c.simGetCollisionInfo(vehicle_name=NAME).time_stamp
  while time.monotonic()-begin<70:
   s=rear(adapter.read_states()[NAME]);audit,errors=obs.read(NAME)
   if errors:raise RuntimeError('Calibration body contact '+str(errors))
   hit=c.simGetCollisionInfo(vehicle_name=NAME)
   if hit.has_collided and hit.time_stamp!=baseline:raise RuntimeError('Calibration native contact '+hit.object_name)
   if initial is None:initial=s.heading
   if previous:
    ds=math.hypot(s.x-previous.x,s.y-previous.y);dh=(s.heading-previous.heading+math.pi)%(2*math.pi)-math.pi;travel+=ds
    if ds>.015 and dh>.001 and abs(s.speed)>.15 and time.monotonic()-begin>2:curvatures.append(dh/ds)
   error,station=plan.project((s.x,s.y),progress,ahead=3);progress=max(progress,station);peak_error=max(peak_error,error)
   limit=model.edges[plan.edges[plan.edge_index(progress)]].get('maximum_path_error_m',.7)
   if error>limit:raise RuntimeError('Right-turn path deviation '+str(error))
   delta=plan.steering((s.x,s.y),s.heading,progress,speed=s.speed)
   commanded=min(target,plan.speed_cap(progress))
   yaw=(s.heading-initial+math.pi)%(2*math.pi)-math.pi
   samples.append(dict(t_s=time.monotonic()-begin,x=s.x,y=s.y,heading=s.heading,speed=s.speed,yaw=yaw,progress_m=progress,path_error_m=error,steering_rad=delta,target_m_s=commanded))
   if progress>=plan.edge_ends[3]+7:break
   if seq==1:adapter.start_watchdog_after_staging()
   adapter.send(ControlCommand(NAME,seq,time.monotonic(),commanded,commanded*math.tan(delta)/2.3285,ttl=2));seq+=1
   previous=s;time.sleep(.05)
  adapter.send(ControlCommand(NAME,seq,time.monotonic(),0,0,ttl=2));adapter.close();adapter=None
  row=dict(case=case,target_m_s=target,peak_path_error_m=peak_error,progress_m=progress,completed_yaw_rad=yaw,travel_m=travel,median_radius_m=1/statistics.median(curvatures) if curvatures else None,samples=samples)
  if progress<plan.edge_ends[3]+7:raise RuntimeError('Right-turn verification timeout')
  report['cases'].append(row);print('RIGHT_TURN_VERIFIED',case,target,peak_error,flush=True)
  (OUT/'progress.json').write_text(json.dumps(report,indent=2)+'\n')
 report['status']='PASS'
except Exception as exc:report.update(status='FAIL',failure=repr(exc));print(report['failure'],flush=True)
finally:
 if adapter:adapter.close()
 c.enableApiControl(True,vehicle_name=NAME);c.setCarControls(airsim.CarControls(brake=1),vehicle_name=NAME)
 console("py import builtins,unreal; h=getattr(builtins,'flow_fleet_monitor',None); unreal.unregister_slate_post_tick_callback(h) if h else None; builtins.flow_fleet_monitor=None")
 c.simPause(True);fixture('FREEZE');c.reset();c.simPause(True)
 try:
  fixture('RESTORE')
  stopdir=ROOT/'runs/ue_scene/test_sites';(stopdir/'motion_stop_request.json').write_text(json.dumps(dict(vehicles=names)));ue_script(c,'stop_test_vehicle_motion.py',stopdir/'motion_stop.json')
 finally:c.simPause(False)
 for n,enabled in apis.items():c.enableApiControl(enabled,vehicle_name=n)
 time.sleep(.5)
 report['fleet_max_speed_m_s']=max(abs(c.getCarState(vehicle_name=n).speed) for n in names)
 report.update(fleet_restored=report['fleet_max_speed_m_s']<.1 and max(math.dist(home[n][:2],absolute_ue(c,n,origin)[0][:2]) for n in names)<.1,api_restored=all(c.isApiControlEnabled(vehicle_name=n)==apis[n] for n in names),saved_maps_unchanged=map_fingerprints()==maps)
 if not all(report[k] for k in ('fleet_restored','api_restored','saved_maps_unchanged')):report['status']='FAIL'
 (OUT/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(0 if report['status']=='PASS' else 1)
