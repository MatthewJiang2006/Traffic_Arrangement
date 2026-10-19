#!/usr/bin/env python3
"""Read immutable round40 native logs; reconstruct utilization and diversion churn.

Position occupancy uses rear-distance thresholds, not resource reservations.
Durations are weighted by adjacent native sample timestamps. No UE controls.
"""
import json,math,collections
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root/'runs/fleet16_round39_ue_round40'
r=json.loads((out/'physical_acceptance.json').read_text());events=r['dispatch_events'];diag=[json.loads(s) for s in (out/'physical_control_diagnostics.jsonl').open()];traj=[json.loads(s) for s in (out/'physical_trajectory.jsonl').open()]
services=json.loads((root/'runs/right_turn_tracking_repair/industrialarea_ue_internal_waiting_network.json.before.json').read_text())['service_nodes']
berths={k:v for k,v in services.items() if v['kind'] in ('loading','unloading')};qs={k:v for k,v in services.items() if v['kind']=='waiting'}
edge_model=json.loads((root/'runs/right_turn_tracking_repair/industrialarea_ue_internal_waiting_transport.json.before.json').read_text());edge_rows={e['edge_id']:e for e in edge_model['edges']};motion_kinds=collections.defaultdict(collections.Counter)
total=r['elapsed_s'];metrics=collections.Counter();states=collections.Counter();reasons=collections.Counter();reasonvehicles=collections.defaultdict(collections.Counter);per=collections.defaultdict(collections.Counter);streak={};longest={};occupancy=collections.Counter();snapshots=[]
for i,row in enumerate(traj):
 dt=(traj[i+1]['t_s'] if i+1<len(traj) else total)-row['t_s'];busy=parked=slow=0;promises=0
 for name,pos in row['positions'].items():
  nearb=next((k for k,v in berths.items() if math.dist(pos[:2],v['rear_xy_m'])<=.6),None)
  nearq=next((k for k,v in qs.items() if math.dist(pos[:2],v['rear_xy_m'])<=.8),None)
  speed=abs(pos[3]);state=row['task_states'].get(name,'UNKNOWN');states[state]+=dt
  category='berth' if nearb else 'waiting' if nearq else 'elsewhere'
  metrics[category+'_vehicle_s']+=dt;per[name][category+'_s']+=dt
  if nearb:busy+=1;occupancy[nearb]+=dt
  if nearq:parked+=1
  if speed<.15:
   metrics['stopped_vehicle_s']+=dt
   if not nearb and not nearq:
    metrics['stopped_outside_nodes_s']+=dt;slow+=1;per[name]['stopped_outside_nodes_s']+=dt
  else:metrics['moving_vehicle_s']+=dt
 for node,owner in row['node_holders'].items():
  if node not in berths or owner not in row['positions']:continue
  promises+=1
  if math.dist(row['positions'][owner][:2],berths[node]['rear_xy_m'])>.6:metrics['unarrived_berth_reserved_s']+=dt
 metrics['berth_reserved_s']+=dt*promises
 if i==0 or row['t_s']-snapshots[-1]['time_s']>=60:snapshots.append(dict(time_s=row['t_s'],at_berths=busy,at_waiting=parked,stopped_outside_nodes=slow,reserved_berths=promises))
for i,row in enumerate(diag):
 dt=(diag[i+1]['t_s'] if i+1<len(diag) else total)-row['t_s']
 for n,details in row['execution_details'].items():
  cmd=abs(row['commanded_speeds'].get(n,0));actual=abs(row['positions'][n][3]);dest=details.get('destination','');atdest=details.get('finished',False)
  if atdest:continue
  edge=edge_rows.get(details['edge_id'],{});kind='reverse' if details['reverse'] else edge.get('kind','unknown')
  motion_kinds[kind]['vehicle_s']+=dt;motion_kinds[kind]['speed_integral']+=dt*actual;motion_kinds[kind]['stopped_vehicle_s']+=dt*(actual<.15)
  if dest.startswith('Q'):metrics['enroute_waiting_s']+=dt
  else:metrics['enroute_berth_s']+=dt
  if cmd<.15:
   trace=row['control_trace'].get(n,[]);stage=next((s['stage'] for s in trace if abs(s['target_m_s'])<.15),'post_trace_stop')
   reasons[stage]+=dt;reasonvehicles[stage][n]+=dt
   start=streak.setdefault(n,row['t_s']);length=row['t_s']+dt-start
   if length>longest.get(n,{}).get('duration_s',0):longest[n]=dict(duration_s=length,start_s=start,end_s=row['t_s']+dt,destination=dest,edge_id=details['edge_id'],stage=stage)
  else:streak.pop(n,None)
counts=collections.Counter(e['event'] for e in events)
div=collections.defaultdict(list)
for e in events:
 if e['event']=='FRONT_VEHICLE_DIVERTED':div[(e['vehicle_id'],e['command_id'])].append(e)
churn=[dict(vehicle_id=n,command_id=c,count=len(es),start_s=es[0]['time'],end_s=es[-1]['time'],sequence=[e['previous_destination']+'→'+e['waiting_node'] for e in es]) for (n,c),es in div.items()];churn.sort(key=lambda x:x['count'],reverse=True)
assigned={e['task_id']:e for e in events if e['event']=='TASK_DISPATCHED'}
pickups=[]
for e in events:
 if e['event']=='SERVICING' and e['operation']=='LOADING':
  a=assigned[e['task_id']];pickups.append(dict(vehicle=e['vehicle_id'],task=e['task_id'],warehouse=e['site'],predicted_s=a['estimated_pickup_delay_s'],actual_s=e['time']-a['time']))
service_s=0
for e in events:
 if e['event']=='SERVICING':
  typ=e['operation']+'_COMPLETE';end=next(x for x in events if x['event']==typ and x['task_id']==e['task_id'] and x['time']>=e['time']);service_s+=end['time']-e['time']
summary=dict(motion_kind_statistics={k:dict(v,mean_speed_m_s=v['speed_integral']/v['vehicle_s']) for k,v in motion_kinds.items()},elapsed_s=total,completed_tasks=r['completed_tasks'],expected_tasks=r['expected_tasks'],thresholds=dict(berth_rear_distance_m=.6,waiting_rear_distance_m=.8,stopped_speed_m_s=.15),metrics=dict(metrics),average_vehicles_at_berths=metrics['berth_vehicle_s']/total,average_vehicles_at_waiting=metrics['waiting_vehicle_s']/total,average_stopped_outside_nodes=metrics['stopped_outside_nodes_s']/total,average_moving_vehicles=metrics['moving_vehicle_s']/total,service_vehicle_s=service_s,berth_service_utilization=service_s/(12*total),actual_berth_pose_utilization=metrics['berth_vehicle_s']/(12*total),states_vehicle_s=dict(states),first_near_zero_stage_vehicle_s=dict(reasons),stage_top_vehicles={k:v.most_common(4) for k,v in reasonvehicles.items()},longest_command_stop_by_vehicle=longest,event_counts=dict(counts),diversion_churn=churn,pickup_eta_actual=pickups,berth_occupancy_s=dict(occupancy),snapshots=snapshots)
(out/'efficiency_analysis.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
for k in ('metrics','average_vehicles_at_berths','average_vehicles_at_waiting','average_stopped_outside_nodes','average_moving_vehicles','service_vehicle_s','berth_service_utilization','first_near_zero_stage_vehicle_s','stage_top_vehicles','pickup_eta_actual','snapshots'):print(k,summary[k])
print('CHURN',churn[:4]);print('STOPS',sorted(longest.items(),key=lambda x:x[1]['duration_s'],reverse=True)[:6])
