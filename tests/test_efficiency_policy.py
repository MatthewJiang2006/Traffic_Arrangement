"""Prevent round40 waiting churn and remote/physical reservation confusion."""
import math
from types import SimpleNamespace
import pytest
from logistics_park.scheduling.efficiency_policy import EfficiencyPolicy
from logistics_park.scheduling.travel_time import TravelTimeEstimator
from logistics_park.scheduling.task_dispatch import DispatchMove
from logistics_park.scheduling.conflict_timing import arrival_profile
from logistics_park.scheduling.local_traffic import LocalTraffic
from logistics_park.common.messages import VehicleState
from logistics_park.runtime.transport_control import envelope
from test_task_dispatch import admission_setup
from test_local_traffic import straight
from test_berth_admission import straight_plan


def dispatch():
 d=admission_setup();d.efficiency_policy=EfficiencyPolicy()
 return d


def test_waiting_target_is_retained_and_cannot_ping_pong_after_emergency_change():
 d=dispatch();d.network.services={k:dict(v) for k,v in d.network.services.items()}
 d.network.services['Q2']=dict(kind='waiting',capacity=1)
 m=d._issue(d.assignments['near'],'Q',d.network.route('A','Q'),True)
 d._time(20)
 with pytest.raises(ValueError,match='Committed waiting'):d.divert_to_waiting('near','Q2',d.network.route('A','Q2'))
 assert d.moves['near'] is m and d.holders['Q']=='near'
 d._time(61)
 new=d.divert_to_waiting('near','Q2',d.network.route('A','Q2'),persistent_block=True)
 d.efficiency_policy=EfficiencyPolicy(waiting_commit_s=5)
 d._time(80)
 assert not d.can_divert_waiting(new,'Q',persistent_block=True)  # Emergency does not bypass recent-target history.


def test_long_or_slow_buffer_is_rejected_and_audited_bays_are_quarantined():
 p=EfficiencyPolicy()
 assert p.affordable(30,20,15)
 assert not p.affordable(250,61,25)
 assert not p.affordable(30,20,31)
 assert not p.affordable(30,20,math.inf)
 assert not p.parking_enabled('Q26') and not p.parking_enabled('Q28')


def test_remote_queue_owner_can_depart_without_exclusively_booking_bay():
 d=dispatch();d.promote_approaches({'near':(40.,False),'far':(70.,False)})
 assert d.approach_intents['L']=='near' and 'L' not in d.holders
 moves=d.tick(0)
 assert [(m.vehicle_id,m.destination) for m in moves]==[('near','L')]
 assert 'L' not in d.holders and d.holders['A']=='near'


def test_busy_dock_queue_head_cannot_be_a_follower_of_another_candidate():
 # Actual UE: Car_7_0 stood ahead of Car_0_1, while Car_0_0 departed
 # W6. The faster follower's queue intent stopped the physical leader.
 d=dispatch();d.holders['L']='departing';d.positions['departing']='L'
 priorities={'near':(30.,False,frozenset({'far'}),3.),
             'far':(80.,False,frozenset(),3.)}
 d.promote_approaches(priorities)
 assert d.approach_intents['L']=='far'
 assert d.holders['L']=='departing'
 # Once the actual departing body releases the berth, the front truck gets
 # its admission. The follower remains behind it regardless of ETA rank.
 del d.holders['L'];del d.positions['departing']
 d.promote_approaches(priorities)
 assert d.approach_intents['L']=='far' and d.holders['L']=='far'


def test_near_admission_uses_action_entry_eta_not_long_reverse_parking_time():
 d=dispatch();d.promote_approaches({'near':(60.,False,frozenset(),3.),'far':(70.,False,frozenset(),40.)})
 assert d.holders['L']=='near'
 assert [(m.vehicle_id,m.destination) for m in d.tick(0)]==[('near','L')]


def test_remote_unentered_booking_is_released_but_real_body_is_protected():
 d=dispatch();d.holders['L']='far'
 d.promote_approaches({'near':(40.,False),'far':(70.,False)})
 assert 'L' not in d.holders
 d.holders['L']='far';d.positions['far']='L'
 d.promote_approaches({'near':(2.,False),'far':(70.,False)})
 assert d.holders['L']=='far'


def test_started_final_approach_cannot_be_preempted_by_new_nearer_candidate():
 d=dispatch();d.holders['L']='far'
 d.promote_approaches({'near':(2.,False),'far':(70.,True)})
 assert d.holders['L']=='far'


def passage_with_exit_obstacle(compact,position):
 ex={'ego':straight('ego',-20,0),'obstacle':straight('obstacle',position,0)}
 ex['ego'].plan.edge_starts=[10.];ex['ego'].plan.edge_ends=[20.]
 ex['obstacle'].finished=True
 states={n:VehicleState(n,*e.plan.point(e.progress),0) for n,e in ex.items()}
 p=LocalTraffic();p.compact_permissions=compact;p.prepare(states,ex,{},envelope)
 return p.claim_passage('ego',ex['ego'],0,0,states,{n:[envelope(s)] for n,s in states.items()},envelope)


def test_compact_exit_allows_clear_crossing_without_waiting_for_distant_body():
 assert not passage_with_exit_obstacle(False,20.)
 assert passage_with_exit_obstacle(True,20.)


def test_compact_exit_still_refuses_blocked_outlet():
 assert not passage_with_exit_obstacle(True,8.)


def test_observed_steady_response_extends_eta_but_launch_and_stop_are_not_samples():
 e=TravelTimeEstimator();ex=SimpleNamespace(plan=straight_plan(100,8),progress=0.,finished=False,checkpoint=0)
 states={'truck':VehicleState('truck',0,0,0,speed=2)}
 for t in (0,.5,1.,1.5):
  states['truck'].x=2*t;e.observe(states,{'truck':ex},{'truck':4},t)
 assert e.scale['road']==pytest.approx(.5)
 fast=arrival_profile(ex,states['truck'],0).eta(100)
 learned=arrival_profile(ex,states['truck'],0,speed_scales=e.scale).eta(100)
 assert learned>fast+5
 samples=e.samples['road'];states['truck'].speed=0
 e.observe(states,{'truck':ex},{'truck':0},2)
 assert e.samples['road']==samples


def test_unknown_blockage_is_not_predicted_as_immediate_departure():
 e=TravelTimeEstimator();local=SimpleNamespace(last_denials={'truck':{'reason':'stationary_body'}},
  passage_denials={},paused_by={},maneuver_holds={},clearance_pauses={},wait_since={'truck':0.})
 assert e.blockage_delay('truck',local,20,10)==pytest.approx(14.)
 local.last_denials={}
 assert e.blockage_delay('truck',local,21,10)==0


def test_wait_cycle_recovery_keeps_entered_and_physical_occupancy(monkeypatch):
 from logistics_park.runtime import wait_cycle_recovery as module
 events=[];ex={n:straight(n,0,20*i) for i,n in enumerate(('a','b'))}
 for n,start in (('a',20.),('b',2.)):
  ex[n].plan.edge_starts=[start];ex[n].local_passages={0}
 local=LocalTraffic();local.passages={(n,0):dict(plan=e.plan,end=40.) for n,e in ex.items()}
 d=SimpleNamespace(efficiency_policy=EfficiencyPolicy(),holders={'occupied':'b'},_event=lambda event,**kw:events.append((event,kw)))
 e=SimpleNamespace(dispatch=d,controller=SimpleNamespace(local_traffic=local,executions=ex))
 states={n:VehicleState(n,0,20*i,0) for i,n in enumerate(ex)}
 monkeypatch.setattr(module,'wait_snapshot',lambda *_:dict(cycles=[['a','b']]))
 commands={n:(0,0) for n in states}
 module.recover_wait_cycles(e,states,commands,0)
 module.recover_wait_cycles(e,states,commands,10)
 assert ('a',0) not in local.passages and ('b',0) in local.passages
 assert d.holders=={'occupied':'b'} and commands=={'a':(0,0),'b':(0,0)}
 assert not events[-1][1]['physical_leases_revoked']


def test_wait_cycle_does_not_revoke_permission_when_current_command_would_launch(monkeypatch):
 from logistics_park.runtime import wait_cycle_recovery as module
 ex={n:straight(n,0,i*20) for i,n in enumerate(('a','b'))}
 for e in ex.values():e.plan.edge_starts=[20.];e.local_passages={0}
 local=LocalTraffic();local.passages={(n,0):dict(plan=e.plan,end=40.) for n,e in ex.items()}
 executor=SimpleNamespace(dispatch=SimpleNamespace(efficiency_policy=EfficiencyPolicy()),controller=SimpleNamespace(local_traffic=local,executions=ex))
 monkeypatch.setattr(module,'wait_snapshot',lambda *_:dict(cycles=[['a','b']]))
 states={n:VehicleState(n,0,i*20,0) for i,n in enumerate(ex)}
 for t in (0,10):module.recover_wait_cycles(executor,states,{'a':(1,0),'b':(0,0)},t)
 assert len(local.passages)==2


@pytest.mark.parametrize('origin_kind',['waiting','loading','unloading'])
def test_queue_intent_survives_physical_occupancy_and_can_follow_known_departure(origin_kind):
 d=dispatch();d.positions['far']='L';d.holders['L']='far'
 d.network.services={k:dict(v) for k,v in d.network.services.items()}
 d.network.services['A']['kind']=origin_kind
 d.assignments['far'].operation='UNLOADING';d.assignments['far'].state='MOVING'
 d.moves['far']=DispatchMove(9,'work-far','far',d.assignments['far'].visit_id,'L','U',{},False)
 d.clearance_estimates['far']=11.
 original=d.network.route
 d.network.route=lambda a,b,**kw:{**original(a,b,**kw),'estimated_travel_time_s':5.} if original(a,b,**kw) else None
 d._time(10);d.promote_approaches({'near':(5.,False),'far':(40.,False)})
 assert d.approach_intents['L']=='near' and d.holders['L']=='far'
 moves=d.tick(10)
 assert [(m.vehicle_id,m.destination) for m in moves]==[('near','L')]
 assert d.holders['L']=='far'
 assert any(e['event']==('WAITING_ETA_DEPARTURE' if origin_kind=='waiting' else 'DOCK_ETA_DEPARTURE') for e in d.events)


def test_loaded_dock_does_not_depart_on_unknown_downstream_release():
 d=dispatch();d.network.services={k:dict(v) for k,v in d.network.services.items()}
 d.network.services['A']['kind']='loading';d.positions['far']='L';d.holders['L']='far'
 d.assignments['far'].operation='UNLOADING';d.assignments['far'].state='MOVING'
 d.moves['far']=DispatchMove(9,'work-far','far',d.assignments['far'].visit_id,'L','U',{},False)
 d._waiting_options=lambda *_:[]
 d._time(10);d.promote_approaches({'near':(5.,False),'far':(40.,False)})
 assert d.tick(10)==[] and d.holders['A']=='near' and d.holders['L']=='far'


def queue_executor():
 from logistics_park.runtime.task_execution import ContinuousTaskExecutor
 from logistics_park.runtime.transport_routes import RouteGeometry
 from logistics_park.map.geometry import Polyline
 from logistics_park.runtime.transport_control import Execution
 model=straight_plan(20,8).model
 for key,a,b,kind in [('junction',20,30,'junction'),('connector',30,60,'road'),('dock',60,80,'service_access')]:
  model.edges[key]=dict(centerline=[[a,0],[b,0]],speed_limit=1,kind=kind,capacity_alias=key)
  model.aliases[key]=key;model.physical_paths[key]=Polyline(((a,0),(b,0)))
 p=RouteGeometry(model,['road','junction','connector','dock'],[],'A',['L'])
 executor=ContinuousTaskExecutor.__new__(ContinuousTaskExecutor);executor.dispatch=dispatch()
 executor.dispatch.approach_intents={'L':'far'}
 executor.moves={'near':SimpleNamespace(destination='L')}
 executor.controller=SimpleNamespace(executions={'near':Execution(p)})
 return executor,p


def test_non_head_dock_queue_waits_before_intersection_and_keeps_line_stable():
 executor,p=queue_executor();state=VehicleState('near',0,0,0)
 assert executor._queue_gate('near',p,state)==pytest.approx(12.2)
 executor.controller.executions['near'].progress=12.
 state.speed=.1
 assert executor._queue_gate('near',p,state)==pytest.approx(12.2)
 executor.dispatch.approach_intents['L']='near'
 assert executor._queue_gate('near',p,state)==pytest.approx(52.2)
 # Admission still uses its actual gate, not the upstream queue holding line.
 assert executor._berth_gate(p)==pytest.approx(52.2)


@pytest.mark.parametrize('progress,speed',[(18.,0.),(10.,3.)])
def test_queue_gate_never_stops_entered_or_unbrakeable_truck(progress,speed):
 executor,p=queue_executor();executor.controller.executions['near'].progress=progress
 assert executor._queue_gate('near',p,VehicleState('near',progress,0,0,speed=speed))==pytest.approx(52.2)


def test_queue_head_waits_before_intersection_until_busy_dock_can_clear():
 executor,p=queue_executor();d=executor.dispatch;d.approach_intents['L']='near'
 d.holders['L']='far';d.positions['far']='L';state=VehicleState('near',0,0,0)
 assert executor._queue_gate('near',p,state)==pytest.approx(12.2)
 d.clearance_estimates['far']=1.
 assert executor._queue_gate('near',p,state)==pytest.approx(52.2)
 assert d.holders['L']=='far'  # Prediction never revokes the occupied dock.


@pytest.mark.parametrize('source_demand,gate_eta,origin_kind,leaves',[
 (True,40.,'loading',True),(True,40.,'unloading',True),
 (False,40.,'loading',False),(True,5.,'loading',False),(True,40.,'waiting',False)])
def test_far_queue_intent_does_not_pin_a_dock_needed_by_following_tasks(source_demand,gate_eta,origin_kind,leaves):
 d=dispatch();d.network.services={k:dict(v) for k,v in d.network.services.items()}
 d.network.services['A']['kind']=origin_kind;d.holders['L']='far'
 d.assignments['far'].state='MOVING'
 d.moves['far']=DispatchMove(9,'work-far','far',d.assignments['far'].visit_id,'B','L',{},False)
 d._waiting_options=lambda *_:[]
 if source_demand:d.moves['follower']=DispatchMove(10,'following','follower','v','B','A',{},False)
 d._time(10);d.promote_approaches({'near':(70.,False,frozenset(),gate_eta),'far':(5.,True)})
 moves=d.tick(10)
 assert bool(moves)==leaves
 assert d.holders['L']=='far' and d.holders['A']=='near'
 if leaves:
  assert moves[0].destination=='L' and moves[0].vehicle_id=='near'
  assert any(e['event']=='DOCK_SOURCE_CLEARANCE_DEPARTURE' for e in d.events)


def test_wait_snapshot_explains_remote_queue_intent_that_holds_a_source_dock():
 from logistics_park.scheduling.wait_dependencies import wait_snapshot
 d=dispatch();d.approach_intents={'L':'far'}
 ex=SimpleNamespace(dispatch=d,controller=SimpleNamespace(executions={}))
 rows=wait_snapshot(ex,{})['dependencies']
 assert dict(vehicle='near',blocked_by='far',reason='destination_queue_intent',resource='L') in rows


def test_two_ready_occupied_docks_can_depart_without_waiting_for_issued_followers():
 from dataclasses import replace
 d=dispatch();d.network.services={k:dict(v) for k,v in d.network.services.items()}
 d.network.services['A']['kind']='loading'
 # This is the W4-unloading <-> W6-loading ready/ready cycle: no movement
 # request exists yet, so counting only already moving followers misses it.
 d.assignments['far'].work=replace(d.assignments['far'].work,loading_node='A')
 d.positions['far']='L';d.holders['L']='far';d.holders.pop('B',None)
 d._waiting_options=lambda *_:[]
 d._time(10);d.promote_approaches({'near':(70.,False,frozenset(),40.),'far':(80.,False,frozenset(),45.)})
 moves=d.tick(10)
 assert {m.vehicle_id for m in moves}=={'near','far'}
 assert d.holders['A']=='near' and d.holders['L']=='far'
 assert len([e for e in d.events if e['event']=='DOCK_SOURCE_CLEARANCE_DEPARTURE'])==2


def test_committed_waiting_target_cannot_be_taken_by_a_second_truck():
 d=dispatch();d.network.services={k:dict(v) for k,v in d.network.services.items()}
 d.network.services['Q2']=dict(kind='waiting',capacity=1)
 d._issue(d.assignments['near'],'Q',d.network.route('A','Q'),True)
 d._issue(d.assignments['far'],'Q2',d.network.route('B','Q2'),True)
 d.waiting_lenders={'near':100.}
 with pytest.raises(ValueError,match='cannot be lent'):
  d.divert_to_waiting('far','Q',d.network.route('B','Q'),reassign_from='near',persistent_block=True)
 assert d.holders['Q']=='near' and d.moves['near'].destination=='Q'


def test_free_bay_allows_road_staging_truck_to_advance_without_stealing_admission():
 d=dispatch();d.network.services={k:dict(v) for k,v in d.network.services.items()}
 d.network.services['B']['kind']='staging';d.assignments['far'].state='READY'
 d.promote_approaches({'near':(40.,False),'far':(70.,False)})
 moves=d.tick(0)
 assert {(m.vehicle_id,m.destination) for m in moves}=={('near','L'),('far','L')}
 assert 'L' not in d.holders
 assert any(e['event']=='STAGING_TO_ADMISSION_GATE' for e in d.events)


def test_actual_slow_position_progress_not_game_velocity_drives_eta_scale():
 e=TravelTimeEstimator();ex=SimpleNamespace(plan=straight_plan(100,8),progress=0.,finished=False)
 state=VehicleState('truck',0,0,0,speed=4,timestamp=100.)
 e.observe({'truck':state},{'truck':ex},{'truck':4},0)
 state.x=.25;state.timestamp=100.25
 e.observe({'truck':state},{'truck':ex},{'truck':4},.25)
 assert e.scale['road']==pytest.approx(.25)


def test_final_dock_admission_precedes_indivisible_junction_access_holding_line():
 from logistics_park.runtime.task_execution import ContinuousTaskExecutor
 from logistics_park.runtime.transport_routes import RouteGeometry
 from logistics_park.map.geometry import Polyline
 model=straight_plan(20,8).model
 for key,a,b,kind in [('junction',20,30,'junction'),('connector',30,34,'road'),('dock',34,60,'service_access')]:
  model.edges[key]=dict(centerline=[[a,0],[b,0]],speed_limit=1,kind=kind,capacity_alias=key)
  model.aliases[key]=key;model.physical_paths[key]=Polyline(((a,0),(b,0)))
 p=RouteGeometry(model,['road','junction','connector','dock'],[],'A',['L'])
 executor=ContinuousTaskExecutor.__new__(ContinuousTaskExecutor)
 assert executor._berth_gate(p)==pytest.approx(20-7.8)
 # Waiting at the gate qualifies without traversing an unapproved bundle.
 ex=SimpleNamespace(plan=p,progress=executor._berth_gate(p),checkpoint=0,finished=False)
 assert arrival_profile(ex,VehicleState('truck',ex.progress,0,0),0).eta(ex.progress)==0


@pytest.mark.parametrize('kind,gap,expected',[('uturn',17.3,12.2),('junction',14.,36.2),('junction',20.,42.2)])
def test_dock_gate_precedes_previous_maneuver_when_outlet_crosses_hold(kind,gap,expected):
 from logistics_park.runtime.task_execution import ContinuousTaskExecutor
 from logistics_park.runtime.transport_routes import RouteGeometry
 from logistics_park.map.geometry import Polyline
 model=straight_plan(20,8).model
 for key,a,b,k in [('action',20,30,kind),('connector',30,30+gap,'road'),('dock',30+gap,60+gap,'service_access')]:
  model.edges[key]=dict(centerline=[[a,0],[b,0]],speed_limit=1,kind=k,capacity_alias=key)
  model.aliases[key]=key;model.physical_paths[key]=Polyline(((a,0),(b,0)))
 p=RouteGeometry(model,['road','action','connector','dock'],[],'A',['L'])
 executor=ContinuousTaskExecutor.__new__(ContinuousTaskExecutor);executor.dispatch=dispatch()
 assert executor._berth_gate(p)==pytest.approx(expected)


def test_current_passage_books_one_car_access_and_preserves_shared_road_capacity():
 from logistics_park.runtime.transport_control import TransportController
 from logistics_park.scheduling.road_capacity import RoadCapacity
 keys=['access','junction','road','remote']
 c=TransportController.__new__(TransportController)
 c.model=SimpleNamespace(aliases={k:k for k in keys},edges={
  'access':{'kind':'service_access'},'junction':{'kind':'junction'},
  'road':{'kind':'road'},'remote':{'kind':'service_access'}})
 c.capacity=RoadCapacity(SimpleNamespace(edges={k:SimpleNamespace(storage_capacity=1 if k in ('access','remote') else 3) for k in keys}))
 p=SimpleNamespace(edges=keys,edge_index=lambda s:2)
 aliases=c._passage_capacity_aliases(p,0,20.)
 assert aliases=={'access','road'}  # No whole-route or shared junction lock.
 for a in aliases:assert c.capacity.reserve('first',a)
 assert not c.capacity.reserve('second','access')
 assert c.capacity.reserve('second','road')
 c.capacity.update({'first':VehicleState('first',0,0,0,edge_id='access')})
 assert not c.capacity.reserve('second','access')
 c.capacity.update({'first':VehicleState('first',10,0,0,edge_id='road')})
 assert c.capacity.reserve('second','access') and not c.capacity.violations


@pytest.mark.parametrize('demand', ['shared_resource','transit_node','adjacent_bay','unrelated'])
def test_source_clearance_counts_adjacent_and_through_access_demand(demand):
 d=dispatch();d.network.services={k:dict(v) for k,v in d.network.services.items()}
 d.network.services['A'].update(kind='loading',occupation_resources=['source-clearance'])
 d.holders['L']='far';d.assignments['far'].state='MOVING'
 route={}
 if demand=='shared_resource':route['required_resources']=['source-clearance']
 if demand=='transit_node':route['must_be_empty_nodes']=['A']
 d.moves['far']=DispatchMove(9,'work-far','far',d.assignments['far'].visit_id,'B','L',route,False)
 if demand=='adjacent_bay':
  d.network.services['L']['occupation_resources']=['source-clearance']
 d._waiting_options=lambda *_:[]
 d._time(10);d.promote_approaches({'near':(70.,False,frozenset(),40.),'far':(5.,True)})
 moves=d.tick(10)
 assert bool(moves)==(demand!='unrelated')
 assert d.holders['A']=='near' and d.holders['L']=='far'
 if moves:
  assert moves[0].vehicle_id=='near' and moves[0].destination=='L'
  assert next(e for e in d.events if e['event']=='DOCK_SOURCE_CLEARANCE_DEPARTURE')['source_waiters']==['far']


def test_ue_shared_access_capacity_denial_exposes_actual_blocker():
 import json
 from pathlib import Path
 from logistics_park.routing.service_network import ServiceNetwork
 from logistics_park.runtime.task_execution import ServiceRouteGeometry
 from logistics_park.runtime.transport_routes import TransportMap
 from logistics_park.runtime.transport_control import TransportController,Execution
 from logistics_park.scheduling.wait_dependencies import wait_snapshot
 root=Path(__file__).resolve().parents[1]
 f=json.loads((root/'tests/fixtures/round47_ue_shared_access_cycle.json').read_text())
 spec=json.loads((root/'config/runtime/industrialarea_ue_fleet16_round47_proactive_full_v21.json').read_text())
 network=ServiceNetwork(root/spec['network'],blocked_edges=spec['blocked_candidate_edges'])
 # W1's access_002 terminates at the source dock, but its resource metadata
 # only says access:W1, rather than berth:W1-loading. Inspect real traversal.
 from logistics_park.scheduling.task_dispatch import TaskDispatcher
 from logistics_park.scheduling.fleet_lifecycle import FleetRegistry
 d=TaskDispatcher(network,FleetRegistry())
 d.moves['Car_7_0']=DispatchMove(46,'transport-25','Car_7_0','visit',f['origin'],f['destination'],f['route'],False)
 assert d._source_waiters('W1-loading','Car_0_0')=={'Car_7_0'}
 geometry=ServiceRouteGeometry(network,TransportMap(root/spec['map']))
 plan=geometry.plan(SimpleNamespace(**f))
 c=TransportController(geometry.model,{},local_traffic=True,retain_finished_vehicles=True)
 c.local_traffic.proactive_admission=True;c.local_traffic.compact_permissions=True
 c.executions={'Car_7_0':Execution(plan,progress=f['progress_m'],initial_station=f['progress_m']),
               'Car_0_0':Execution(geometry.plan(SimpleNamespace(**f['parked_route'])),
                                  progress=f['parked_progress_m'],finished=True)}
 c.plans={n:e.plan for n,e in c.executions.items()}
 # The runtime route has a measured source snap; this reconstructed parked
 # route uses the nominal source, so station it at the same terminal access.
 c.executions['Car_0_0'].progress=c.plans['Car_0_0'].length
 c.executions['Car_0_0'].checkpoint=len(c.plans['Car_0_0'].checkpoints)
 c.capacity.capacities.update(c.model.raw['physical_capacities'])
 states={n:VehicleState(n,**row) for n,row in f['states'].items()}
 commands=c.tick(states,1.)
 assert commands['Car_7_0'][0]==0.
 denial=next(e for e in c.control_denials['Car_7_0'] if e['reason']=='passage_capacity')
 assert denial['blocked_by']=='Car_0_0' and denial['resource']=='access_002' and denial['start']==47
 assert ('Car_7_0',47) not in c.local_traffic.passages
 d=SimpleNamespace(assignments={},moves={},holders={},positions={})
 rows=wait_snapshot(SimpleNamespace(dispatch=d,controller=c),states)['dependencies']
 assert any(e['vehicle']=='Car_7_0' and e['blocked_by']=='Car_0_0' and e['reason']=='passage_capacity' for e in rows)
