from types import SimpleNamespace
from logistics_park.common.messages import VehicleState
from logistics_park.scheduling.local_traffic import LocalTraffic
from logistics_park.scheduling.maneuver_admission import prospective_cycle
from logistics_park.runtime.transport_control import envelope
from test_local_traffic import straight


def test_proposed_yield_cannot_close_wait_cycle_and_does_not_mutate():
    local=LocalTraffic();local.admission_dependencies=[
        dict(vehicle='new',blocked_by='middle'),dict(vehicle='middle',blocked_by='old')]
    states={n:VehicleState(n,0,0,0) for n in ('new','middle','old')}
    assert prospective_cycle(local,'new',{'old'},states)==['middle','new','old']
    assert not local.paused_by and not local.passages
    states['middle'].speed=2.
    assert prospective_cycle(local,'new',{'old'},states) is None


def test_proposal_replaces_only_its_own_rechecked_denial_not_physical_or_other_requests():
    local=LocalTraffic()
    states={n:VehicleState(n,0,0,0) for n in ('new','old')}
    row=dict(vehicle='new',blocked_by='old',reason='passage_reservation',resource='passage:3')
    local.admission_dependencies=[row]
    assert prospective_cycle(local,'new',{'old'},states)==['new','old']
    assert prospective_cycle(local,'new',{'old'},states,resolved_passage_start=3) is None
    assert prospective_cycle(local,'new',{'old'},states,resolved_passage_start=4)==['new','old']
    local.admission_dependencies.append(dict(vehicle='new',blocked_by='old',reason='physical_leader',resource='route'))
    assert prospective_cycle(local,'new',{'old'},states,resolved_passage_start=3)==['new','old']


def test_forward_access_space_hold_cannot_be_bypassed_by_ordinary_road_time_exception():
    from unittest.mock import patch
    owner=straight('road',12,0);ego=straight('access',0,0)
    executions={'road':owner,'access':ego}
    states={n:VehicleState(n,*e.plan.point(0),0,speed=2) for n,e in executions.items()}
    for e in executions.values():e.plan.edge_starts=[0.];e.plan.edge_ends=[50.]
    ego.plan.model.edges[ego.plan.edges[0]]['kind']='service_access'
    local=LocalTraffic(point_conflicts=True);local.proactive_admission=True
    local.action_groups={n:((0,0,0.,50.),) for n in executions};local._executions=executions
    local.passages['road',0]=dict(plan=owner.plan,end=50.,stations=[],sweep=[])
    with patch.object(local,'_forecast',side_effect=AssertionError('spatial/time policies cannot mix')):
        assert not local._time_compatible('access','road',ego,0,states,envelope)
    # The same rule applies when the existing owner is the access truck.
    local.passages={('access',0):dict(plan=ego.plan,end=50.,stations=[(0.,envelope(states['access']))],sweep=[])}
    with patch.object(local,'_forecast',side_effect=AssertionError('spatial/time policies cannot mix')):
        assert not local._time_compatible('road','access',owner,0,states,envelope)


def test_entered_special_action_cannot_be_paused_by_new_crossing():
    owner=straight('owner',0,-20,1.57079632679)
    owner.plan.model.edges[owner.plan.edges[0]]['kind']='uturn'
    owner.plan.edge_starts=[20.];owner.plan.edge_ends=[40.];owner.progress=25.
    new=straight('new',-20,0);new.plan.edge_starts=[5.];new.plan.edge_ends=[30.]
    states={n:VehicleState(n,*e.plan.point(e.progress),e.plan.heading(e.progress)) for n,e in [('owner',owner),('new',new)]}
    local=LocalTraffic();local.proactive_admission=True
    lease=dict(plan=owner.plan,end=40.,sweep=[envelope(VehicleState('owner',0,0,1.57079632679))],created_at=0.)
    local.passages['owner',0]=lease
    local.prepare(states,{'owner':owner,'new':new},{},envelope,1.)
    assert not local.claim_passage('new',new,0,0,states,{n:[envelope(s)] for n,s in states.items()},envelope)
    assert local.passages=={('owner',0):lease} and not local.paused_by
    assert local.passage_denials['new'][0]['reason']=='entered_special_conflict'


def test_proactive_parallel_roads_still_share():
    local=LocalTraffic();local.proactive_admission=True
    a=straight('a',-20,0);b=straight('b',-20,20)
    for e in (a,b):e.plan.edge_starts=[5.];e.plan.edge_ends=[30.]
    states={n:VehicleState(n,*e.plan.point(0),0) for n,e in [('a',a),('b',b)]}
    local.prepare(states,{'a':a,'b':b},{},envelope)
    boxes={n:[envelope(s)] for n,s in states.items()}
    assert local.claim_passage('a',a,0,0,states,boxes,envelope)
    assert local.claim_passage('b',b,0,0,states,boxes,envelope)


def test_safe_landing_moves_wait_line_upstream_when_final_pose_is_occupied():
    from test_efficiency_policy import queue_executor
    from logistics_park.runtime.safe_landing import safe_landing
    executor,p=queue_executor()
    executor.controller.local_traffic=None
    states={'near':VehicleState('near',0,0,0),'blocker':VehicleState('blocker',52.2,0,0)}
    assert safe_landing(executor,'near',p,52.2,states)==12.2
    states['another']=VehicleState('another',12.2,0,0)
    assert safe_landing(executor,'near',p,52.2,states) is None


def test_safe_landing_rejects_wait_inside_maneuver_outlet_or_braking_distance():
    from test_efficiency_policy import queue_executor
    from logistics_park.runtime.safe_landing import safe_landing
    executor,p=queue_executor();executor.controller.local_traffic=None
    states={'near':VehicleState('near',0,0,0)}
    assert safe_landing(executor,'near',p,32.,states)==12.2
    assert safe_landing(executor,'near',p,52.2,states,speed=8.) is None


def test_predeparture_without_safe_hold_selects_waiting_and_retains_source():
    from test_efficiency_policy import dispatch
    d=dispatch();d.departure_guard=lambda vehicle,origin,target,route,waiting:waiting
    d.promote_approaches({'near':(.5,False),'far':(70.,False)})
    moves=d.tick(0)
    assert moves and moves[0].vehicle_id=='near' and moves[0].waiting
    assert d.holders['A']=='near'
    assert any(e['event']=='PREDEPARTURE_WAITING_SELECTED' for e in d.events)


def test_moving_diversion_preserves_full_braking_prefix_and_occupancy():
    import pytest
    from logistics_park.runtime.transport_control import TransportController
    old=straight('ego',0,0);old.progress=20.
    new=straight('ego',0,0).plan;new.stops=['Q']
    original=new.point;new.point=lambda s:original(s) if s<40 else (s,1.)
    c=TransportController.__new__(TransportController);c.local_traffic=LocalTraffic();c.local_traffic.proactive_admission=True
    c.executions={'ego':old};c.plans={'ego':old.plan};c.model=old.plan.model;c.events=[]
    occupancy={'road-ego':{'ego'}}
    c.capacity=SimpleNamespace(committed={},occupants=occupancy)
    with pytest.raises(ValueError):c.reroute_forward('ego',new,1,state=VehicleState('ego',20,0,0,speed=8))
    assert old.plan is c.plans['ego']
    c.reroute_forward('ego',new,1,state=VehicleState('ego',20,0,0,speed=4))
    assert old.plan is new and c.capacity.occupants==occupancy
    assert c.events[-1]['event']=='PROACTIVE_ROAD_DIVERSION'


def test_claim_rejects_prospective_cycle_before_changing_any_permission():
    import math
    owner=straight('owner',0,-30,math.pi/2);owner.progress=17.
    owner.plan.edge_starts=[20.];owner.plan.edge_ends=[40.]
    ego=straight('ego',-20,0);ego.plan.edge_starts=[5.];ego.plan.edge_ends=[30.]
    third=straight('third',80,80);third.finished=True
    executions={'owner':owner,'ego':ego,'third':third}
    states={n:VehicleState(n,*e.plan.point(e.progress),e.plan.heading(e.progress)) for n,e in executions.items()}
    local=LocalTraffic();local.proactive_admission=True
    lease=dict(plan=owner.plan,end=40.,sweep=[envelope(VehicleState('owner',0,0,math.pi/2))],created_at=0.)
    local.passages['owner',0]=lease
    local.admission_dependencies=[dict(vehicle='ego',blocked_by='third'),dict(vehicle='third',blocked_by='owner')]
    local.prepare(states,executions,{},envelope,1.)
    assert not local.claim_passage('ego',ego,0,0,states,{n:[envelope(s)] for n,s in states.items()},envelope)
    assert local.passage_denials['ego'][0]['reason']=='prospective_wait_cycle'
    assert local.passages=={('owner',0):lease} and not local.paused_by and not local.released


def test_diversion_rejects_new_action_bundle_inside_braking_prefix():
    import pytest
    from test_efficiency_policy import queue_executor
    from test_berth_admission import straight_plan
    from logistics_park.runtime.transport_control import TransportController,Execution
    executor,p=queue_executor();old=straight_plan(100,8)
    c=TransportController.__new__(TransportController);c.local_traffic=LocalTraffic();c.local_traffic.proactive_admission=True
    c.executions={'ego':Execution(old,progress=10.)};c.plans={'ego':old};c.model=old.model;c.events=[]
    c.capacity=SimpleNamespace(committed={},occupants={'road':{'ego'}})
    with pytest.raises(ValueError,match='action stop line'):
        c.reroute_forward('ego',p,1,state=VehicleState('ego',10,0,0,speed=3))
    assert c.executions['ego'].plan is old and not c.events


def w6_exit_hold_fixture():
    import json
    from pathlib import Path
    from logistics_park.routing.service_network import ServiceNetwork
    from logistics_park.runtime.task_execution import ServiceRouteGeometry
    from logistics_park.runtime.transport_routes import TransportMap
    root=Path(__file__).resolve().parents[1]
    fixture=json.loads((root/'tests/fixtures/round47_w6_exit_hold.json').read_text())
    spec=json.loads((root/'config/runtime/industrialarea_ue_fleet16_round47_proactive_full_v4.json').read_text())
    network=ServiceNetwork(root/spec['network'],blocked_edges=spec['blocked_candidate_edges'])
    geometry=ServiceRouteGeometry(network,TransportMap(root/spec['map']))
    executor=SimpleNamespace(dispatch=SimpleNamespace(network=network,holders={}),geometry=geometry,
                             controller=SimpleNamespace(local_traffic=None))
    plan=geometry.plan(SimpleNamespace(**fixture))
    return executor,plan,fixture


def test_w6_queue_hold_leaves_a_future_departure_clear_even_before_berth_is_occupied():
    from logistics_park.runtime.safe_landing import safe_landing,service_exit_sweeps
    from logistics_park.safety.collision import overlaps
    executor,plan,fixture=w6_exit_hold_fixture()
    states={'queue':VehicleState('queue',*plan.point(0),plan.heading(0))}
    old=fixture['old_hold_s']
    selected=safe_landing(executor,'queue',plan,old,states,target='W6-loading')
    assert selected is not None and selected<old-20
    body=envelope(VehicleState('queue',*plan.point(selected),plan.heading(selected)))
    exits=service_exit_sweeps(executor,'W6-loading')
    assert not any(overlaps(body,b,.35) for b in exits)
    old_body=envelope(VehicleState('queue',*plan.point(old),plan.heading(old)))
    assert any(overlaps(old_body,b,.35) for b in exits)
    # Admitted arrival still reaches the dock; this is a holding-pose check.
    assert safe_landing(executor,'queue',plan,plan.length,states,target='W6-loading')==plan.length


def test_w6_physically_occupied_dock_exit_is_protected_without_a_passage_lease():
    from logistics_park.runtime.safe_landing import safe_landing
    executor,plan,fixture=w6_exit_hold_fixture()
    site=executor.dispatch.network.services['W6-loading']
    executor.dispatch.holders={'W6-loading':'dock'}
    states={'queue':VehicleState('queue',*plan.point(0),plan.heading(0)),
            'dock':VehicleState('dock',*site['rear_xy_m'],0)}
    assert safe_landing(executor,'queue',plan,fixture['old_hold_s'],states)<fixture['old_hold_s']-20
    # A stale source token alone cannot reserve an empty exit forever.
    states['dock'].x=1000
    assert safe_landing(executor,'queue',plan,fixture['old_hold_s'],states)==fixture['old_hold_s']


def test_dock_exit_protection_does_not_include_a_later_merged_intersection():
    from logistics_park.runtime.safe_landing import service_exit_sweeps
    from logistics_park.safety.collision import overlaps
    executor,_,_=w6_exit_hold_fixture()
    network=executor.dispatch.network
    route=network.route('W6-loading','W2-unloading')
    plan=executor.geometry.plan(SimpleNamespace(origin='W6-loading',destination='W2-unloading',route=route))
    # The existing permission groups this later junction with the departure;
    # only immediate access + tail clearance determines forbidden queue poses.
    body=envelope(VehicleState('later',*plan.point(60.),plan.heading(60.)))
    assert not any(overlaps(body,b,.35) for b in service_exit_sweeps(executor,'W6-loading'))


def test_exit_safe_gate_is_also_the_berth_admission_eta_boundary():
    import json
    from pathlib import Path
    from logistics_park.runtime.task_execution import ContinuousTaskExecutor
    from logistics_park.runtime.transport_control import Execution
    from logistics_park.scheduling.efficiency_policy import EfficiencyPolicy
    base,_,_=w6_exit_hold_fixture()
    fixture=json.loads((Path(__file__).parent/'fixtures/round47_w5_admission_gate.json').read_text())
    plan=base.geometry.plan(SimpleNamespace(**fixture))
    executor=ContinuousTaskExecutor.__new__(ContinuousTaskExecutor)
    executor.dispatch=base.dispatch;executor.geometry=base.geometry;executor.controller=base.controller
    executor.dispatch.efficiency_policy=EfficiencyPolicy(proactive_admission=True)
    executor.dispatch.now=fixture['checkpoint_s']
    state=VehicleState('head',**fixture['state']);execution=Execution(plan,progress=fixture['progress'])
    gate=executor._berth_gate(plan)
    assert gate<5.
    assert executor._admission_eta('head',plan,state,execution,station=gate)<15.
    executor.dispatch.efficiency_policy=EfficiencyPolicy(proactive_admission=False)
    old=executor._berth_gate(plan)
    assert old>40.
    assert executor._admission_eta('head',plan,state,execution,station=old)>15.


def test_forward_dock_departure_keeps_approaching_traffic_out_of_its_merge_sweep():
    import json,math
    from pathlib import Path
    from logistics_park.runtime.transport_control import Execution
    from logistics_park.scheduling.conflict_timing import maneuver_groups
    from logistics_park.scheduling.local_traffic import sweeps_intersect
    base,_,_=w6_exit_hold_fixture()
    rows=json.loads((Path(__file__).parent/'fixtures/round47_forward_access_merge.json').read_text())
    executions={n:Execution(base.geometry.plan(SimpleNamespace(**row)),progress=row['progress'])
                for n,row in rows.items()}
    # Before the approaching truck reaches the original wedge.
    executions['Car_7_0'].progress-=15.
    states={n:VehicleState(n,*e.plan.point(e.progress),e.plan.heading(e.progress)) for n,e in executions.items()}
    owner=executions['Car_2_1'];group=maneuver_groups(owner.plan)[0];count=math.ceil(group[3]/.25)
    stations=[(group[3]*i/count,envelope(VehicleState('Car_2_1',*owner.plan.point(group[3]*i/count),owner.plan.heading(group[3]*i/count))))
              for i in range(count+1)]
    local=LocalTraffic(point_conflicts=True);local.proactive_admission=True
    local._executions=executions;local.now=327.
    local.passages['Car_2_1',group[0]]=dict(plan=owner.plan,end=group[3],start=group[0],stations=stations,
                                        sweep=[b for _,b in stations],created_at=300.)
    local._protect_maneuver_space(states,executions,envelope)
    hold=local.maneuver_holds['Car_7_0']['stop_s']
    assert executions['Car_7_0'].progress<hold<rows['Car_7_0']['progress']
    road=executions['Car_7_0'].plan
    body=envelope(VehicleState('Car_7_0',*road.point(hold),road.heading(hold)))
    protected=local._special_boxes(local.passages['Car_2_1',group[0]],owner.progress-.25)
    assert not sweeps_intersect([body],protected,.35)
    local.proactive_admission=False
    local._protect_maneuver_space(states,executions,envelope)
    assert 'Car_7_0' not in local.maneuver_holds


def test_w5_unloading_admission_cannot_block_physically_occupied_loading_exit():
    import math
    from logistics_park.runtime.safe_landing import berth_exit_blockers,parking_exit_conflicts
    executor,_,_=w6_exit_hold_fixture()
    d=executor.dispatch;site=d.network.services['W5-loading']
    d.holders={'W5-loading':'departing'};d.moves={};d.admission_priorities={};d.now=420.
    d._event=lambda *args,**kwargs:None
    executor.latest_states={'departing':VehicleState('departing',*site['rear_xy_m'],math.radians(site['heading_deg']))}
    assert 'W5-loading' in parking_exit_conflicts(executor,'W5-unloading')
    assert berth_exit_blockers(executor,'W5-unloading','arriving')=={'departing'}
    # The dispatcher's idle-clearance query intentionally has no vehicle id.
    assert berth_exit_blockers(executor,'W5-unloading','')=={'departing'}
    # An unrelated dock remains usable, and the exiting truck cannot block itself.
    assert not berth_exit_blockers(executor,'W1-unloading','arriving')
    assert not berth_exit_blockers(executor,'W5-unloading','departing')
    executor.latest_states['departing'].x=1000.
    assert not berth_exit_blockers(executor,'W5-unloading','arriving')
    # Final approach already entered protects the future exit; a remote
    # unentered promise by itself does not occupy the dock.
    d.moves['departing']=SimpleNamespace(origin='Q2',destination='W5-loading')
    d.admission_priorities={'departing':(1.,False)}
    assert not berth_exit_blockers(executor,'W5-unloading','arriving')
    d.admission_priorities['departing']=(1.,True)
    assert berth_exit_blockers(executor,'W5-unloading','arriving')=={'departing'}


def test_owned_goal_admission_rejects_earlier_hold_instead_of_ignoring_it():
    from unittest.mock import patch
    from logistics_park.runtime.task_execution import ContinuousTaskExecutor
    base,_,_=w6_exit_hold_fixture()
    executor=ContinuousTaskExecutor.__new__(ContinuousTaskExecutor)
    executor.dispatch=base.dispatch;executor.geometry=base.geometry;executor.controller=base.controller
    executor.latest_states={};executor.landing_rejections={}
    executor.dispatch.now=420.;executor.dispatch.holders={'W5-unloading':'arriving'}
    executor.dispatch._event=lambda *args,**kwargs:None
    route=executor.dispatch.network.route('W3-loading','W5-unloading')
    with patch('logistics_park.runtime.safe_landing.safe_landing',return_value=10.):
        assert not executor._departure_guard('arriving','W3-loading','W5-unloading',route,False)
        assert not executor._departure_guard('arriving','W3-loading','W5-unloading',route,True)
    assert not getattr(executor,'safe_landing_states',{})


def test_older_junction_exit_can_hold_before_a_later_reverse_dock_sweep():
    import json,math
    from pathlib import Path
    from logistics_park.runtime.transport_control import Execution
    from logistics_park.scheduling.conflict_timing import maneuver_groups
    from logistics_park.scheduling.local_traffic import sweeps_intersect
    base,_,_=w6_exit_hold_fixture()
    rows=json.loads((Path(__file__).parent/'fixtures/round47_w2_older_exit_boundary.json').read_text())
    executions={n:Execution(base.geometry.plan(SimpleNamespace(**row))) for n,row in rows.items()}
    road=executions['Car_1_1'];dock=executions['Car_2_0']
    ordinary=maneuver_groups(road.plan)[1];reverse=maneuver_groups(dock.plan)[-1]
    road.progress=ordinary[3]-1.;dock.progress=reverse[2]-25.
    states={n:VehicleState(n,*ex.plan.point(ex.progress),ex.plan.heading(ex.progress)) for n,ex in executions.items()}
    policy=LocalTraffic(point_conflicts=True);policy.proactive_admission=True;policy.now=1230.
    policy._executions=executions
    for n,group,created in [('Car_1_1',ordinary,1214.7),('Car_2_0',reverse,1229.9)]:
        plan=executions[n].plan;count=math.ceil((group[3]-group[2])/.25)
        stations=[(s,envelope(VehicleState(n,*plan.point(s),plan.heading(s))))
                  for s in (group[2]+(group[3]-group[2])*i/count for i in range(count+1))]
        policy.passages[n,group[0]]=dict(plan=plan,start=group[0],end=group[3],outlet=min(plan.length,group[3]+6),
            stations=stations,sweep=[b for _,b in stations],created_at=created)
    policy._protect_maneuver_space(states,executions,envelope)
    hold=policy.maneuver_holds['Car_1_1']['stop_s']
    assert hold>ordinary[3]+6. and hold>road.progress
    body=envelope(VehicleState('Car_1_1',*road.plan.point(hold),road.plan.heading(hold)))
    protected=policy._special_boxes(policy.passages['Car_2_0',reverse[0]])
    assert not sweeps_intersect([body],protected,.35)
    states['Car_1_1'].speed=3.
    forecast,_=policy._forecast('Car_1_1',states['Car_1_1'],road,8.,None,envelope)
    assert not sweeps_intersect(forecast,protected,.35)
    # Old behavior exempted the whole route until the reverse began.
    policy.proactive_admission=False
    policy._protect_maneuver_space(states,executions,envelope)
    assert 'Car_1_1' not in policy.maneuver_holds


def test_w3_queue_keeps_clear_of_the_selected_merged_departure_not_just_short_access():
    import json,math
    from pathlib import Path
    from logistics_park.runtime.transport_control import Execution
    from logistics_park.runtime.safe_landing import safe_landing,occupied_exit_sweeps,service_exit_sweeps
    from logistics_park.scheduling.local_traffic import sweeps_intersect
    base,_,_=w6_exit_hold_fixture()
    rows=json.loads((Path(__file__).parent/'fixtures/round47_w3_selected_exit_queue.json').read_text())
    executions={n:Execution(base.geometry.plan(SimpleNamespace(**row)),progress=row['progress']) for n,row in rows.items()}
    base.controller.executions=executions
    base.dispatch.holders={'W3-loading':'Car_4_1'}
    base.dispatch.moves={'Car_4_1':SimpleNamespace(origin='W3-loading')}
    states={n:VehicleState(n,*e.plan.point(e.progress),e.plan.heading(e.progress)) for n,e in executions.items()}
    queued=executions['Car_1_1'];old_body=envelope(states['Car_7_0'])
    assert not sweeps_intersect([old_body],service_exit_sweeps(base,'W3-loading'),.35)
    chosen=occupied_exit_sweeps(base,'W3-loading','Car_4_1')
    # The second queued truck (Car_1_1) already prevents the complete exit.
    assert sweeps_intersect([envelope(states['Car_1_1'])],chosen,.35)
    # Find a safe queue boundary before vehicles enter this frozen wedge.
    desired=queued.progress;queued.progress-=50.
    states['Car_1_1']=VehicleState('Car_1_1',*queued.plan.point(queued.progress),queued.plan.heading(queued.progress))
    selected=safe_landing(base,'Car_1_1',queued.plan,desired,states,progress=queued.progress,target='W3-loading')
    assert selected is not None and queued.progress<selected<desired
    body=envelope(VehicleState('Car_1_1',*queued.plan.point(selected),queued.plan.heading(selected)))
    assert not sweeps_intersect([body],chosen,.35)
    front=executions['Car_7_0'];confirmed=front.progress;front.progress-=40.
    base.safe_landing_states={('Car_7_0',front.plan.edges):confirmed}
    states['Car_7_0']=VehicleState('Car_7_0',*front.plan.point(front.progress),front.plan.heading(front.progress),speed=3.)
    predicted=safe_landing(base,'Car_1_1',queued.plan,desired,states,progress=queued.progress,target='W3-loading')
    assert predicted is not None and predicted<=selected


def test_loading_task_exit_is_protected_before_departure_command_is_issued():
    import json
    from pathlib import Path
    from logistics_park.runtime.safe_landing import occupied_exit_sweeps
    from logistics_park.scheduling.local_traffic import sweeps_intersect
    from logistics_park.scheduling.task_dispatch import TransportWork
    base,_,_=w6_exit_hold_fixture()
    row=json.loads((Path(__file__).parent/'fixtures/round47_w3_selected_exit_queue.json').read_text())['Car_4_1']
    base.dispatch.moves={};base.dispatch.assignments={'loader':SimpleNamespace(work=TransportWork('known-next','W3-loading','W1-unloading'))}
    base.dispatch._route=lambda *args,**kwargs:row['route']
    sweep=occupied_exit_sweeps(base,'W3-loading','loader')
    queue=envelope(VehicleState('queue',-90.2938,114.84,-1.5708))
    assert sweeps_intersect([queue],sweep,.35)
    # No actual assignment means speculative exits retain the smaller access.
    base.dispatch.assignments={}
    assert not sweeps_intersect([queue],occupied_exit_sweeps(base,'W3-loading','loader'),.35)


def test_ue_opposite_lane_margin_escape_may_reduce_a_still_positive_gap():
    import json
    from pathlib import Path
    from logistics_park.runtime.transport_control import Execution
    from logistics_park.safety.collision import overlaps,separating_gap
    base,_,_=w6_exit_hold_fixture()
    rows=json.loads((Path(__file__).parent/'fixtures/round47_ue_opposite_margin.json').read_text())
    executions={n:Execution(base.geometry.plan(SimpleNamespace(**row)),progress=row['progress']) for n,row in rows.items()}
    states={n:VehicleState(n,**row['state']) for n,row in rows.items()}
    road,departing='Car_2_0','Car_5_1'
    assert overlaps(envelope(states[road]),envelope(states[departing]),.35)
    assert not overlaps(envelope(states[road]),envelope(states[departing]),.2)
    policy=LocalTraffic(point_conflicts=True);policy.proactive_admission=True
    policy.prepare(states,executions,{},envelope)
    result=policy.regulate(states,executions,{road:(0.,0.),departing:(3.,0.)},{},279.,envelope)
    assert result[road][0]==0. and 0.<result[departing][0]<=.5
    forecast=policy._forecast(departing,states[departing],executions[departing],result[departing][0],None,envelope)[0]
    gaps=[separating_gap(body,envelope(states[road]),.35) for body in forecast]
    assert gaps[0]<0. and all(gap>0. for gap in gaps[1:])
    assert min(gaps[2:])<gaps[1]  # Falling gap remains outside the full margin.
    assert all(not overlaps(body,envelope(states[road]),.2) for body in forecast)


def test_arrived_queue_head_uses_actual_exit_safe_line_for_near_admission():
    import json
    from pathlib import Path
    from logistics_park.runtime.task_execution import ContinuousTaskExecutor
    from logistics_park.runtime.transport_control import Execution
    from logistics_park.scheduling.efficiency_policy import EfficiencyPolicy
    base,_,_=w6_exit_hold_fixture()
    row=json.loads((Path(__file__).parent/'fixtures/round47_dynamic_admission_hold.json').read_text())
    n='head';plan=base.geometry.plan(SimpleNamespace(**row));ex=Execution(plan,progress=row['progress'])
    state=VehicleState(n,**row['state'])
    executor=ContinuousTaskExecutor.__new__(ContinuousTaskExecutor)
    executor.dispatch=base.dispatch;executor.geometry=base.geometry;executor.controller=base.controller
    d=executor.dispatch;d.now=736.;d.efficiency_policy=EfficiencyPolicy(proactive_admission=True)
    d.offroad_busy_wait=True;d.traffic_routing=None;d.approach_intents={row['destination']:n};d._event=lambda *args,**kwargs:None
    d.moves={n:SimpleNamespace(**row)};executor.moves=d.moves
    executor.controller.executions={n:ex};executor.controller.destination_holds={n:row['hold']}
    assert executor._berth_gate(plan)>row['hold']+40.
    assert executor._approach_priorities({n:state})[n][3]<3.
    # A follower cannot acquire a remote promise using the head's queue line.
    d.approach_intents[row['destination']]='other'
    assert executor._approach_priorities({n:state})[n][3]>15.
    # Nor can the head claim early merely because a future hold was selected.
    d.approach_intents[row['destination']]=n;ex.progress-=10.;d.now+=1.
    assert executor._approach_priorities({n:state})[n][3]>15.


def q29_hold_fixture():
    import json
    from pathlib import Path
    from logistics_park.runtime.transport_control import Execution
    base,_,_=w6_exit_hold_fixture()
    rows=json.loads((Path(__file__).parent/'fixtures/round47_q29_parking_hold.json').read_text())
    names=['Car_2_1','Car_6_1']
    executions={n:Execution(base.geometry.plan(SimpleNamespace(**rows[n])),progress=rows[n]['progress']) for n in names}
    states={n:VehicleState(n,**rows[n]['state']) for n in names}
    return executions,states,rows['admission_probe']


def test_q29_protected_stop_has_room_for_sampled_braking_and_parking_pose_error():
    import math
    from logistics_park.scheduling.conflict_timing import maneuver_groups
    from logistics_park.scheduling.local_traffic import sweeps_intersect
    executions,states,_=q29_hold_fixture();owner='Car_6_1';road='Car_2_1';dock=executions[owner]
    group=maneuver_groups(dock.plan)[-1];count=math.ceil((group[3]-group[2])/.25)
    stations=[(s,envelope(VehicleState(owner,*dock.plan.point(s),dock.plan.heading(s))))
              for s in (group[2]+(group[3]-group[2])*i/count for i in range(count+1))]
    ex=executions[road];ex.progress-=20.
    states[road]=VehicleState(road,*ex.plan.point(ex.progress),ex.plan.heading(ex.progress),speed=3.)
    policy=LocalTraffic(point_conflicts=True);policy.proactive_admission=True;policy._executions=executions;policy.now=1070.
    lease=dict(plan=dock.plan,start=group[0],end=group[3],stations=stations,sweep=[b for _,b in stations],created_at=1053.6)
    policy.passages[owner,group[0]]=lease
    policy._protect_maneuver_space(states,executions,envelope)
    stop=policy.maneuver_holds[road]['stop_s']
    assert ex.progress<stop<264.
    forecast=policy._forecast(road,states[road],ex,3.,None,envelope)[0]
    remaining=policy._special_boxes(lease,dock.progress-.25)
    assert not sweeps_intersect(forecast,remaining,.35)
    assert not sweeps_intersect(forecast,[envelope(states[owner])],.35)


def test_q29_admission_rejects_braking_that_cannot_reach_the_buffered_holding_line():
    from logistics_park.scheduling.conflict_timing import maneuver_groups
    executions,states,probe=q29_hold_fixture();owner='Car_6_1';road='Car_2_1'
    executions[owner].progress=probe['owner_progress'];executions[road].progress=probe['road_progress']
    for n,ex in executions.items():
        states[n]=VehicleState(n,*ex.plan.point(ex.progress),ex.plan.heading(ex.progress),speed=probe['road_speed'] if n==road else 0.)
    group=maneuver_groups(executions[owner].plan)[-1]
    policy=LocalTraffic(point_conflicts=True);policy.proactive_admission=True
    policy.prepare(states,executions,{},envelope,1000.)
    boxes={n:[envelope(s)] for n,s in states.items()}
    assert not policy.claim_passage(owner,executions[owner],group[0],group[1],states,boxes,envelope)
    assert policy.passage_denials[owner][-1]['reason']=='special_hold_stopping_room'
    assert not policy.passages
    # A truck already stopped safely upstream does not prevent parking.
    states[road].speed=0.;policy.prepare(states,executions,{},envelope,1001.)
    assert policy.claim_passage(owner,executions[owner],group[0],group[1],states,boxes,envelope)
