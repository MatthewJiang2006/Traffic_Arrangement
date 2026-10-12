#!/usr/bin/env python3
"""Measure loaded-truck rear-axle curvature at a cleared known right junction."""
import json,math,statistics,sys,time
from pathlib import Path
import airsim
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from logistics_park.adapters.airsim import AirSimAdapter,AirSimConfig
from logistics_park.common.messages import ControlCommand
from logistics_park.runtime.body_observer import FleetFileObserver
from validate_ue_node_pairs import rear
from validate_ue_loading_access import map_fingerprints
from ue_trailer_driving_check import ue_script
from ue_dynamic_crossing_check import absolute_ue
NAME='Car_5_1';OUT=ROOT/'runs/right_turn_response_calibration_run4';OUT.mkdir(parents=True,exist_ok=True)
c=airsim.CarClient(timeout_value=5);names=c.listVehicles()
if c.simIsPause() or any(abs(c.getCarState(vehicle_name=n).speed)>.1 for n in names):raise RuntimeError('Stationary unpaused fleet required')
scene=ue_script(c,'export_scene.py',ROOT/'runs/ue_scene/scene.json');origin=[v/100 for v in scene['world_origin_cm']]
home={n:absolute_ue(c,n,origin)[0] for n in names};poses={n:c.simGetVehiclePose(vehicle_name=n) for n in names};apis={n:c.isApiControlEnabled(vehicle_name=n) for n in names};maps=map_fingerprints()
def fixture(action,vehicles=None):
 (ROOT/'runs/ue_node_pairs/fleet_request.json').write_text(json.dumps(dict(action=action,vehicles=vehicles or names,time_scale=1,output_dir=str(OUT))))
 return ue_script(c,'node_pair_fleet_fixture.py',OUT/'fleet_status.json')
def console(code):
 if not c.simRunConsoleCommand(code):raise RuntimeError('Rejected UE command')
report=dict(status='RUNNING',cases=[],camera_changed=False,map_changed=False,initial_fleet_ue_m=home);adapter=None
try:
 fixture('INIT')
 for target in (.5,1.,1.5):
  if adapter:adapter.close();adapter=None
  fixture('FREEZE',[NAME]);c.simPause(True)
  x,y=-86.4937,271.;h=-math.pi/2
  local=poses[NAME].position;world=home[NAME];px=x+1.16425*math.cos(h);py=y+1.16425*math.sin(h)
  pose=airsim.Pose(airsim.Vector3r(local.x_val+px-world[0],local.y_val+py-world[1],local.z_val-.894+world[2]),airsim.to_quaternion(0,0,h))
  fixture('ACTIVATE',[NAME]);c.enableApiControl(True,vehicle_name=NAME);c.simSetVehiclePose(pose,True,vehicle_name=NAME);c.setCarControls(airsim.CarControls(brake=1),vehicle_name=NAME)
  placed=absolute_ue(c,NAME,origin)[0]
  if math.dist(placed[:2],(px,py))>.1:raise RuntimeError('Incorrect calibration staged pose '+str(placed))
  stopdir=ROOT/'runs/ue_scene/test_sites';(stopdir/'motion_stop_request.json').write_text(json.dumps(dict(vehicles=[NAME])));ue_script(c,'stop_test_vehicle_motion.py',stopdir/'motion_stop.json');c.simPause(False);time.sleep(.2)
  console('py import builtins,runpy; builtins.fleet_monitor_request='+repr(dict(vehicles=[NAME],file='steering_calibration_live.json'))+'; runpy.run_path('+repr(str(ROOT/'scripts/ue/monitor_fleet.py'))+')')
  obs=FleetFileObserver(ROOT/'runs/ue_scene/steering_calibration_live.json')
  adapter=AirSimAdapter(AirSimConfig(vehicle_ids=(NAME,),origin_ue_m=tuple(origin),command_ttl_s=2,allow_reverse=True,speed_rise_m_s2=.6,speed_error_gain=.6),OUT/('adapter_'+str(target)+'.jsonl'),watchdog=False);adapter.open();adapter.snapshot_provider=obs.snapshot;adapter.last_source={};adapter.source_advanced={}
  initial=None;previous=None;samples=[];curvatures=[];travel=0;begin=time.monotonic();seq=1;baseline=c.simGetCollisionInfo(vehicle_name=NAME).time_stamp
  while time.monotonic()-begin<45:
   s=rear(adapter.read_states()[NAME]);audit,errors=obs.read(NAME)
   if errors:raise RuntimeError('Calibration body contact '+str(errors))
   hit=c.simGetCollisionInfo(vehicle_name=NAME)
   if hit.has_collided and hit.time_stamp!=baseline:raise RuntimeError('Calibration native contact '+hit.object_name)
   if initial is None:initial=s.heading
   if previous:
    ds=math.hypot(s.x-previous.x,s.y-previous.y);dh=(s.heading-previous.heading+math.pi)%(2*math.pi)-math.pi;travel+=ds
    if ds>.015 and dh>.001 and abs(s.speed)>.15 and time.monotonic()-begin>2:curvatures.append(dh/ds)
   yaw=(s.heading-initial+math.pi)%(2*math.pi)-math.pi
   samples.append(dict(t_s=time.monotonic()-begin,x=s.x,y=s.y,heading=s.heading,speed=s.speed,yaw=yaw))
   if yaw>=1.15 or travel>=6:break
   if seq==1:adapter.start_watchdog_after_staging()
   adapter.send(ControlCommand(NAME,seq,time.monotonic(),target,target*math.tan(math.radians(40))/2.3285,ttl=2));seq+=1
   previous=s;time.sleep(.05)
  adapter.send(ControlCommand(NAME,seq,time.monotonic(),0,0,ttl=2));adapter.close();adapter=None
  row=dict(target_m_s=target,completed_yaw_rad=yaw,travel_m=travel,median_radius_m=1/statistics.median(curvatures) if curvatures else None,samples=samples)
  if yaw<1.1 or len(curvatures)<10:raise RuntimeError('Insufficient calibration motion')
  report['cases'].append(row);print('CALIBRATED',target,row['median_radius_m'],flush=True)
  (OUT/'progress.json').write_text(json.dumps(report,indent=2)+'\n')
 report['status']='PASS'
except Exception as exc:report.update(status='FAIL',failure=repr(exc));print(report['failure'],flush=True)
finally:
 if adapter:adapter.close()
 c.enableApiControl(True,vehicle_name=NAME);c.setCarControls(airsim.CarControls(brake=1),vehicle_name=NAME)
 console("py import builtins,unreal; h=getattr(builtins,'flow_fleet_monitor',None); unreal.unregister_slate_post_tick_callback(h) if h else None; builtins.flow_fleet_monitor=None")
 c.reset();c.simPause(True)
 try:
  fixture('RESTORE')
  stopdir=ROOT/'runs/ue_scene/test_sites';(stopdir/'motion_stop_request.json').write_text(json.dumps(dict(vehicles=names)));ue_script(c,'stop_test_vehicle_motion.py',stopdir/'motion_stop.json')
 finally:c.simPause(False)
 for n,enabled in apis.items():c.enableApiControl(enabled,vehicle_name=n)
 time.sleep(.5)
 report.update(fleet_restored=max(math.dist(home[n][:2],absolute_ue(c,n,origin)[0][:2]) for n in names)<.1,api_restored=all(c.isApiControlEnabled(vehicle_name=n)==apis[n] for n in names),saved_maps_unchanged=map_fingerprints()==maps)
 if not all(report[k] for k in ('fleet_restored','api_restored','saved_maps_unchanged')):report['status']='FAIL'
 (OUT/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(0 if report['status']=='PASS' else 1)
