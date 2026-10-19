#!/usr/bin/env python3
"""Read-only fleet snapshot benchmark; never sends driving commands."""
import json,time
from validate_ue_node_pairs import airsim,ROOT
from logistics_park.adapters.airsim import AirSimAdapter,AirSimConfig
from logistics_park.runtime.body_observer import FleetFileObserver
c=airsim.CarClient(timeout_value=5);names=c.listVehicles()
if any(abs(c.getCarState(vehicle_name=n).speed)>.1 for n in names):raise RuntimeError('Stationary fleet required')
p=ROOT/'runs/ue_scene/flow_fleet_live.json';p.unlink(missing_ok=True)
try:
 console="py import builtins,runpy; builtins.fleet_monitor_request="+repr(dict(vehicles=names))+"; runpy.run_path("+repr(str(ROOT/'scripts/ue/monitor_fleet.py'))+")"
 if not c.simRunConsoleCommand(console):raise RuntimeError('Monitor rejected')
 deadline=time.monotonic()+2
 while not p.exists() and time.monotonic()<deadline:time.sleep(.02)
 observer=FleetFileObserver(p)
 a=AirSimAdapter(AirSimConfig(tuple(names),(0,0,0)),ROOT/'runs/fleet24_flow/snapshot_probe.jsonl',watchdog=False,snapshot_provider=observer.snapshot)
 try:
  times=[]
  for i in range(20):
   start=time.monotonic();states=a.read_states();times.append(time.monotonic()-start);time.sleep(.05)
  audits=[observer.read(n)[1] for n in names]
 finally:a.close()
 result=dict(vehicle_count=len(states),mean_read_s=sum(times)/len(times),maximum_read_s=max(times),body_audit_errors=audits,controls_sent=False,camera_changed=False)
 (ROOT/'runs/fleet24_flow/snapshot_probe.json').write_text(json.dumps(result,indent=2)+'\n')
 print({k:v for k,v in result.items() if k!='body_audit_errors'})
finally:
 c.simRunConsoleCommand("py import builtins,unreal; h=getattr(builtins,'flow_fleet_monitor',None); unreal.unregister_slate_post_tick_callback(h) if h is not None else None; builtins.flow_fleet_monitor=None")
