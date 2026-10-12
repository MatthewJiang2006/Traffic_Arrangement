"""Operational safety and downstream capacity in the flow scheduler."""
import math,json
from types import SimpleNamespace
from pathlib import Path
import pytest
from logistics_park.common.messages import VehicleState
from logistics_park.runtime.transport_control import envelope
from logistics_park.scheduling.local_traffic import LocalTraffic
from logistics_park.runtime.body_observer import FleetFileObserver
from logistics_park.adapters.airsim import AirSimAdapter,AirSimConfig,AdapterFault
from test_airsim_adapter import FakeRPC
from test_traffic_routing import network
from logistics_park.scheduling.task_dispatch import TaskDispatcher,TransportWork
from logistics_park.scheduling.fleet_lifecycle import FleetRegistry

def straight(name,x,y,heading=0):
    def point(s):return x+s*math.cos(heading),y+s*math.sin(heading)
    def project(p,s,**kwargs):
        dx,dy=p[0]-x,p[1]-y
        return abs(-dx*math.sin(heading)+dy*math.cos(heading)),max(0.,dx*math.cos(heading)+dy*math.sin(heading))
    alias='road-'+name
    model=SimpleNamespace(edges={alias:{'kind':'road','reverse':False}},aliases={alias:alias},raw={'physical_capacities':{alias:10}})
    plan=SimpleNamespace(length=100.,point=point,project=project,heading=lambda s:heading,speed_cap=lambda s:8.,edge_index=lambda s:0,reverse=lambda s:False,edges=(alias,),model=model,checkpoints=[])
    return SimpleNamespace(plan=plan,progress=0.,finished=False,checkpoint=0)

def test_parallel_opposite_lanes_can_both_move():
    ex={'a':straight('a',-20,0),'b':straight('b',20,4,math.pi)}
    states={'a':VehicleState('a',-20,0,0),'b':VehicleState('b',20,4,math.pi)}
    result=LocalTraffic().regulate(states,ex,{'a':(8,0),'b':(8,0)},{},0,envelope)
    assert result['a'][0]==8 and result['b'][0]==8

def test_conflicting_crossings_do_not_receive_equal_full_speed():
    ex={'a':straight('a',-20,0),'b':straight('b',0,-20,math.pi/2)}
    states={'a':VehicleState('a',-20,0,0),'b':VehicleState('b',0,-20,math.pi/2)}
    result=LocalTraffic().regulate(states,ex,{'a':(8,0),'b':(8,0)},{},0,envelope)
    assert result['a'][0]==8 and result['b'][0]<8

def test_stopped_truck_remains_an_obstacle():
    ex={'a':straight('a',-20,0),'b':straight('b',0,0)};ex['b'].finished=True
    states={'a':VehicleState('a',-20,0,0),'b':VehicleState('b',0,0,0)}
    result=LocalTraffic().regulate(states,ex,{'a':(8,0),'b':(0,0)},{},0,envelope)
    assert result['a'][0]<8 and result['b'][0]==0

def test_busy_bay_uses_free_waiting_place_without_stealing_lease(tmp_path):
    n=network(tmp_path,alternatives=False);f=FleetRegistry();d=TaskDispatcher(n,f,service_s=0,follow_to_busy_destination=True)
    for name,node in [('ego','L'),('busy','U')]:
        v=f.adopt_present_vehicle(name,0,fully_inside=True,resources=[]);d.place_vehicle(name,v.visit_id,node,0,stopped=True)
    d.submit(TransportWork('work','L','U'));d.tick(0);move=d.tick(1)[0]
    assert move.destination=='Q' and move.waiting and d.holders['U']=='busy'
    d.promote_approaches();assert d.holders['U']=='busy'

def test_busy_bay_can_be_approached_when_waiting_places_are_full(tmp_path):
    n=network(tmp_path,alternatives=False);f=FleetRegistry();d=TaskDispatcher(n,f,service_s=0,follow_to_busy_destination=True)
    for name,node in [('ego','L'),('busy','U'),('waiting','Q')]:
        v=f.adopt_present_vehicle(name,0,fully_inside=True,resources=[]);d.place_vehicle(name,v.visit_id,node,0,stopped=True)
    d.submit(TransportWork('work','L','U'));d.tick(0);move=d.tick(1)[0]
    assert move.destination=='U' and d.holders['U']=='busy'
    d.promote_approaches();assert d.holders['U']=='busy'
    d.holders.pop('U');d.promote_approaches();assert d.holders['U']=='ego'

def frame(now=1.,sequence=1):
    return dict(monotonic_s=now,sequence=sequence,paused=False,states={'truck':dict(x=12.,y=23.,z=26.,heading=math.pi/2,speed=0.)},audits={'truck':dict(monotonic_s=now,bodies=[],wheels=[],pitch_deg=0,roll_deg=0)})

def test_batch_snapshot_uses_world_frame_and_brakes_on_stale_data(tmp_path):
    path=tmp_path/'frame.json';path.write_text(json.dumps(frame()));clock=SimpleNamespace(now=1.)
    observer=FleetFileObserver(path,clock=lambda:clock.now);rpc=FakeRPC()
    a=AirSimAdapter(AirSimConfig(('truck',),(10,20,30)),tmp_path/'adapter.jsonl',transport_factory=lambda:rpc,clock=lambda:clock.now,watchdog=False,snapshot_provider=observer.snapshot)
    a.open()
    try:
        state=a.read_states()['truck'];assert (state.x,state.y)==(12.,23.)
        clock.now=2.3
        with pytest.raises(AdapterFault):a.read_states()
        assert rpc.native[-1]['brake']==1
    finally:a.close()

def test_missing_vehicle_snapshot_fails_closed(tmp_path):
    path=tmp_path/'frame.json';row=frame();row['audits']={};path.write_text(json.dumps(row))
    observer=FleetFileObserver(path,clock=lambda:1.)
    with pytest.raises(RuntimeError,match='Incomplete'):observer.snapshot()

def test_near_queue_leader_can_replace_only_an_unentered_berth_reservation(tmp_path):
    from logistics_park.scheduling.task_dispatch import Assignment,DispatchMove
    n=network(tmp_path,alternatives=False);f=FleetRegistry();d=TaskDispatcher(n,f,follow_to_busy_destination=True)
    work=TransportWork('work','L','U')
    for name,origin,distance in [('far','Q',100),('near','L',40)]:
        v=f.adopt_present_vehicle(name,0,fully_inside=True,resources=[])
        d.positions[name]=origin;d.assignments[name]=Assignment(work,name,v.visit_id,operation='UNLOADING',state='MOVING')
        d.moves[name]=DispatchMove(distance,work.task_id,name,v.visit_id,origin,'U',{},False)
    d.holders={'U':'far'}
    d.promote_approaches({'far':(100,True),'near':(40,False)})
    assert d.holders['U']=='far'  # Already committed approach.
    d.promote_approaches({'far':(100,False),'near':(40,False)})
    assert d.holders['U']=='near'
    d.positions['near']='U'
    d.promote_approaches({'far':(1,False),'near':(40,False)})
    assert d.holders['U']=='near'  # Actual parked body cannot yield a lease.

def test_adjacent_maneuvers_can_be_reserved_before_the_first_tail_clears():
    ex=straight('ego',0,0);ex.plan.edge_starts=[5.,20.];ex.plan.edge_ends=[10.,25.]
    states={'ego':VehicleState('ego',0,0,0)};policy=LocalTraffic();policy.prepare(states,{'ego':ex},{},envelope)
    boxes={'ego':[envelope(states['ego'])]}
    assert policy.claim_passage('ego',ex,0,0,states,boxes,envelope)
    assert policy.claim_passage('ego',ex,1,1,states,boxes,envelope)
    assert len(policy.passages)==2
    ex.progress=23.;policy.prepare(states,{'ego':ex},{},envelope)
    assert ('ego',0) not in policy.passages and ('ego',1) in policy.passages

def test_maneuver_exit_cannot_be_reserved_over_a_real_parked_body():
    ex=straight('ego',0,0);ex.plan.edge_starts=[5.];ex.plan.edge_ends=[10.]
    parked=straight('parked',15,0);parked.finished=True
    states={'ego':VehicleState('ego',0,0,0),'parked':VehicleState('parked',15,0,0)}
    policy=LocalTraffic();policy.prepare(states,{'ego':ex,'parked':parked},{},envelope)
    assert not policy.claim_passage('ego',ex,0,0,states,{n:[envelope(s)] for n,s in states.items()},envelope)
    assert not policy.passages
    assert policy.passage_denials['ego'][0]['blocked_by']=='parked'

def test_passage_releases_only_the_path_behind_the_tail():
    ex=straight('ego',0,0);ex.plan.edge_starts=[5.];ex.plan.edge_ends=[30.]
    states={'ego':VehicleState('ego',0,0,0)};policy=LocalTraffic()
    policy.prepare(states,{'ego':ex},{},envelope)
    assert policy.claim_passage('ego',ex,0,0,states,{'ego':[envelope(states['ego'])]},envelope)
    original=list(policy.passages['ego',0]['sweep'])
    ex.progress=25.;policy.prepare(states,{'ego':ex},{},envelope)
    remaining=policy.passages['ego',0]['sweep']
    assert len(remaining)<len(original)
    assert all(b.x>=25.-6.+.75 for b in remaining)
    assert original[-1] in remaining

def test_suffix_routing_preserves_the_incoming_lane_movement(tmp_path):
    n=network(tmp_path)
    route=n.route('X','U',incoming_edge='alt1')
    assert route['edge_ids']==['alt2']
    assert n.route('X','U',incoming_edge='alt1',occupied_nodes={'U'}) is None
    with pytest.raises(ValueError):n.route('X','U',incoming_edge='direct')
    with pytest.raises(ValueError):n.route('X','U')

def test_waiting_diversion_preserves_task_cargo_command_and_source_lease(tmp_path):
    from logistics_park.scheduling.task_dispatch import Assignment,DispatchMove
    n=network(tmp_path);f=FleetRegistry();d=TaskDispatcher(n,f,follow_to_busy_destination=True)
    v=f.adopt_present_vehicle('ego',0,fully_inside=True,resources=[])
    work=TransportWork('work','L','U')
    d.assignments['ego']=Assignment(work,'ego',v.visit_id,operation='UNLOADING',state='MOVING',cargo_units=1)
    d.moves['ego']=DispatchMove(7,'work','ego',v.visit_id,'L','U',n.route('L','U'),False)
    d.holders={'L':'ego','U':'ego'}
    new=d.divert_to_waiting('ego','Q',n.route('L','Q'))
    assert new.command_id==7 and new.origin=='L' and new.waiting
    assert d.holders=={'L':'ego','Q':'ego'}
    assert d.assignments['ego'].work is work and d.assignments['ego'].cargo_units==1

def test_stopped_diversion_cannot_change_the_executed_prefix_or_live_permission():
    from logistics_park.runtime.transport_control import TransportController
    old=straight('ego',0,0);old.progress=20.;old.plan.reverse=lambda s:False
    candidate=straight('ego',0,0).plan;candidate.reverse=lambda s:False;candidate.stops=['Q']
    c=TransportController.__new__(TransportController);c.local_traffic=LocalTraffic();c.executions={'ego':old}
    c.model=SimpleNamespace(edges={old.plan.edges[0]:{'kind':'road'}});c.plans={'ego':old.plan};c.events=[]
    released=[];c.capacity=SimpleNamespace(committed={'outlet':{'ego'}},release_commitment=lambda n,a:released.append((n,a)))
    state=VehicleState('ego',20,0,0)
    c.local_traffic.passages[('ego',0)]={}
    with pytest.raises(ValueError):c.reroute_stopped('ego',candidate,1,state=state)
    assert old.plan is c.plans['ego'] and not released
    c.local_traffic.passages.clear();point=candidate.point;candidate.point=lambda s:(s,1)
    with pytest.raises(ValueError):c.reroute_stopped('ego',candidate,1,state=state)
    assert old.plan is c.plans['ego'] and not released
    candidate.point=lambda s:point(s) if s<40 else (s,1)
    c.reroute_stopped('ego',candidate,1,state=state)
    assert old.plan is candidate and old.progress==20 and released==[('ego','outlet')]

def test_vehicle_already_waiting_does_not_join_a_busy_bay_queue(tmp_path):
    from logistics_park.scheduling.task_dispatch import Assignment
    n=network(tmp_path);f=FleetRegistry();d=TaskDispatcher(n,f,follow_to_busy_destination=True)
    for name,node in [('ego','Q'),('busy','U')]:
        v=f.adopt_present_vehicle(name,0,fully_inside=True,resources=[]);d.place_vehicle(name,v.visit_id,node,0,stopped=True)
    work=TransportWork('work','L','U');d.tasks['work']=work
    d.assignments['ego']=Assignment(work,'ego',f.vehicles['ego'].visit_id,operation='UNLOADING',state='WAITING',cargo_units=1)
    from logistics_park.scheduling.task_dispatch import WorkStatus
    d.status['work']=WorkStatus.ACTIVE
    assert not any(m.vehicle_id=='ego' for m in d.tick(30))
    assert d.holders['Q']=='ego' and d.assignments['ego'].cargo_units==1

def test_unentered_booking_is_not_a_parked_transit_obstacle(tmp_path):
    n=network(tmp_path);f=FleetRegistry();d=TaskDispatcher(n,f,follow_to_busy_destination=True)
    d.holders={'L':'ego','U':'approaching'};d.positions={'ego':'L'}
    assert d._route('L','U','ego') is not None
    assert d._blockers('U','ego')=={'approaching'}  # Still exclusive admission.
    d.positions['approaching']='U'
    assert d._route('L','U','ego') is None

def test_skipped_short_road_booking_releases_after_tail_but_not_actual_occupancy():
    from logistics_park.runtime.transport_control import TransportController
    from logistics_park.scheduling.road_capacity import RoadCapacity
    graph=SimpleNamespace(edges={a:SimpleNamespace(storage_capacity=2) for a in ['short','next']})
    c=TransportController.__new__(TransportController);c.capacity=RoadCapacity(graph)
    plan=SimpleNamespace(edges=('short','next'),edge_ends=(10.,100.))
    ex=SimpleNamespace(plan=plan,progress=15.)
    c.executions={'ego':ex};c.model=SimpleNamespace(aliases={'short':'short','next':'next'})
    c.capacity.reserve('ego','short');c.capacity.update({'ego':VehicleState('ego',15,0,0,edge_id='next')})
    c._release_passed_commitments();assert 'ego' in c.capacity.committed['short']
    ex.progress=17.;c._release_passed_commitments()
    assert 'ego' not in c.capacity.committed['short'] and c.capacity.occupants['next']=={'ego'}

def test_cleared_access_is_not_reclaimed_after_another_truck_parks_there():
    from logistics_park.routing.service_network import ServiceNetwork
    from logistics_park.runtime.transport_routes import TransportMap
    from logistics_park.runtime.task_execution import ServiceRouteGeometry
    from logistics_park.runtime.transport_control import TransportController,Execution
    from logistics_park.scheduling.task_dispatch import DispatchMove
    root=Path(__file__).resolve().parents[1]
    cfg=json.loads((root/'config/runtime/industrialarea_ue_fleet24_flow.json').read_text())
    network=ServiceNetwork(root/cfg['network'],blocked_edges=cfg['blocked_candidate_edges'])
    model=TransportMap(root/cfg['map']);geometry=ServiceRouteGeometry(network,model)
    plan=geometry.plan(DispatchMove(1,'task','ego','visit','W4-loading','W2-unloading',network.route('W4-loading','W2-unloading'),False))
    last=0
    while model.edges[plan.edges[last+1]]['kind']=='service_access':last+=1
    station=plan.edge_ends[last]+7.
    parked=geometry.stationary('W4-loading')
    controller=TransportController(model,{'ego':plan},local_traffic=True,retain_finished_vehicles=True)
    controller.activate('ego',0)
    # The resumed fixture carries the permission obtained before departure.
    controller.tick({'ego':VehicleState('ego',*plan.point(0),plan.heading(0))},0)
    assert controller.local_traffic.passages
    controller.executions['ego'].progress=station
    controller.plans['parked']=parked;controller.executions['parked']=Execution(parked,finished=True)
    states={'ego':VehicleState('ego',*plan.point(station),plan.heading(station)),
        'parked':VehicleState('parked',*parked.point(0),parked.heading(0))}
    commands=controller.tick(states,1)
    from logistics_park.safety.collision import overlaps
    assert all(not overlaps(envelope(states['parked']),box,.35)
        for lease in controller.local_traffic.passages.values() for box in lease['sweep'])
    assert commands['ego'][0]>0

def test_stopped_permission_owner_can_hold_while_a_clear_passage_goes_past():
    owner=straight('owner',0,-30,math.pi/2);owner.progress=20.
    owner.plan.edge_starts=[20.,60.];owner.plan.edge_ends=[40.,80.]
    ego=straight('ego',-20,0);ego.plan.edge_starts=[5.];ego.plan.edge_ends=[30.]
    ex={'owner':owner,'ego':ego};states={'owner':VehicleState('owner',0,-10,math.pi/2),'ego':VehicleState('ego',-20,0,0)}
    policy=LocalTraffic();policy.prepare(states,ex,{},envelope);boxes={n:[envelope(s)] for n,s in states.items()}
    assert policy.claim_passage('owner',owner,0,0,states,boxes,envelope)
    assert policy.claim_passage('ego',ego,0,0,states,boxes,envelope)
    assert ('owner',0) in policy.passages  # Permission and occupancy retained.
    result=policy.regulate(states,ex,{'owner':(8,0),'ego':(8,0)},{},0,envelope)
    assert result['owner'][0]==0 and result['ego'][0]>0
    assert not policy.claim_passage('owner',owner,1,1,states,boxes,envelope)
    ego.progress=37.;states['ego'].x=17.;policy.prepare(states,ex,{},envelope)
    assert not policy.paused_by
    assert policy.regulate(states,ex,{'owner':(8,0),'ego':(8,0)},{},1,envelope)['owner'][0]>0

def test_moving_permission_owner_is_not_forced_to_yield():
    owner=straight('owner',0,-30,math.pi/2);owner.progress=20.
    owner.plan.edge_starts=[20.];owner.plan.edge_ends=[40.]
    ego=straight('ego',-20,0);ego.plan.edge_starts=[5.];ego.plan.edge_ends=[30.]
    states={'owner':VehicleState('owner',0,-10,math.pi/2,speed=.5),'ego':VehicleState('ego',-20,0,0)}
    policy=LocalTraffic();policy.prepare(states,{'owner':owner,'ego':ego},{},envelope);boxes={n:[envelope(s)] for n,s in states.items()}
    assert policy.claim_passage('owner',owner,0,0,states,boxes,envelope)
    assert not policy.claim_passage('ego',ego,0,0,states,boxes,envelope)
    assert not policy.paused_by

def test_yield_lasts_until_passing_body_clears_owners_outlet():
    owner=straight('owner',0,-30,math.pi/2);owner.progress=20.
    owner.plan.edge_starts=[20.];owner.plan.edge_ends=[40.]
    ego=straight('ego',-20,0);ego.plan.edge_starts=[5.];ego.plan.edge_ends=[30.]
    ex={'owner':owner,'ego':ego}
    states={'owner':VehicleState('owner',0,-10,math.pi/2),'ego':VehicleState('ego',-20,0,0)}
    policy=LocalTraffic();policy.prepare(states,ex,{},envelope)
    boxes={n:[envelope(s)] for n,s in states.items()}
    assert policy.claim_passage('owner',owner,0,0,states,boxes,envelope)
    assert policy.claim_passage('ego',ego,0,0,states,boxes,envelope)
    # Reproduce an action that has expired while its body still occupies the
    # yielding owner's outlet: releasing the action cannot revoke clearance.
    del policy.passages[('ego',0)]
    ego.progress=20.;states['ego'].x=0.
    policy.prepare(states,ex,{},envelope)
    assert policy.paused_by['owner']=={('ego',0)}
    commands=policy.regulate(states,ex,{'owner':(8,0),'ego':(8,0)},{},1,envelope)
    assert commands['owner'][0]==0 and commands['ego'][0]>0
    ego.progress=37.;states['ego'].x=17.
    policy.prepare(states,ex,{},envelope)
    assert not policy.paused_by


def test_pairwise_yield_releases_even_while_the_donors_permission_is_live():
    owner=straight('owner',0,-30,math.pi/2);owner.progress=20.
    owner.plan.edge_starts=[20.];owner.plan.edge_ends=[40.]
    ego=straight('ego',-20,0);ego.plan.edge_starts=[5.];ego.plan.edge_ends=[40.]
    ex={'owner':owner,'ego':ego};states={'owner':VehicleState('owner',0,-10,math.pi/2),'ego':VehicleState('ego',-20,0,0)}
    policy=LocalTraffic();policy.prepare(states,ex,{},envelope,0.)
    boxes={n:[envelope(s)] for n,s in states.items()}
    assert policy.claim_passage('owner',owner,0,0,states,boxes,envelope)
    assert policy.claim_passage('ego',ego,0,0,states,boxes,envelope)
    ego.progress=37.;states['ego'].x=17.
    policy.prepare(states,ex,{},envelope,10.)
    assert ('ego',0) in policy.passages
    assert not policy.paused_by and not policy.yield_holds


def test_aged_yielding_owner_rejects_a_new_donor_instead_of_waiting_forever():
    owner=straight('owner',0,-30,math.pi/2);owner.progress=20.
    owner.plan.edge_starts=[20.];owner.plan.edge_ends=[40.]
    ego=straight('new',-20,0);ego.plan.edge_starts=[5.];ego.plan.edge_ends=[30.]
    states={'owner':VehicleState('owner',0,-10,math.pi/2),'new':VehicleState('new',-20,0,0)}
    policy=LocalTraffic();policy.prepare(states,{'owner':owner,'new':ego},{},envelope,10.)
    boxes={n:[envelope(s)] for n,s in states.items()}
    assert policy.claim_passage('owner',owner,0,0,states,boxes,envelope)
    policy.paused_by['owner']={('old',0)};policy.yield_since['owner']=0.
    assert not policy.claim_passage('new',ego,0,0,states,boxes,envelope)
    assert policy.passage_denials['new'][-1]['reason']=='aged_yield_priority'


@pytest.mark.parametrize('heading,allowed',[(0.,True),(math.pi,False)])
def test_aged_yield_does_not_block_a_passing_trucks_directed_lane_leader(heading,allowed):
    owner=straight('owner',0,-30,math.pi/2);owner.progress=20.
    owner.plan.edge_starts=[20.];owner.plan.edge_ends=[40.]
    passing=straight('passing',-20,0)
    front=straight('front',0,0,heading);front.plan.edge_starts=[5.];front.plan.edge_ends=[30.]
    ex={'owner':owner,'passing':passing,'front':front}
    states={'owner':VehicleState('owner',0,-10,math.pi/2),'passing':VehicleState('passing',-20,0,0),
            'front':VehicleState('front',40,0,heading)}
    policy=LocalTraffic();policy.prepare(states,ex,{},envelope,10.)
    boxes={n:[envelope(s)] for n,s in states.items()}
    assert policy.claim_passage('owner',owner,0,0,states,boxes,envelope)
    states['front'].x=0.
    boxes={n:[envelope(s)] for n,s in states.items()}
    policy.paused_by['owner']={('passing',0)};policy.yield_since['owner']=0.
    assert policy.claim_passage('front',front,0,0,states,boxes,envelope)==allowed


def test_yielding_truck_gets_a_chance_to_restart_before_another_new_yield():
    owner=straight('owner',0,-30,math.pi/2);owner.progress=20.
    owner.plan.edge_starts=[20.];owner.plan.edge_ends=[40.]
    ego=straight('ego',-20,0);ego.plan.edge_starts=[5.];ego.plan.edge_ends=[40.]
    new=straight('new',-40,0);new.plan.edge_starts=[5.];new.plan.edge_ends=[30.]
    ex={'owner':owner,'ego':ego,'new':new}
    states={'owner':VehicleState('owner',0,-10,math.pi/2),'ego':VehicleState('ego',-20,0,0),'new':VehicleState('new',-40,0,0)}
    policy=LocalTraffic();policy.prepare(states,ex,{},envelope,0.)
    boxes={n:[envelope(s)] for n,s in states.items()}
    assert policy.claim_passage('owner',owner,0,0,states,boxes,envelope)
    assert policy.claim_passage('ego',ego,0,0,states,boxes,envelope)
    ego.progress=37.;states['ego'].x=17.
    policy.prepare(states,ex,{},envelope,10.)
    assert not policy.paused_by and 'owner' in policy.resume_priorities
    boxes={n:[envelope(s)] for n,s in states.items()}
    assert not policy.claim_passage('new',new,0,0,states,boxes,envelope)
    assert policy.passage_denials['new'][-1]['reason']=='aged_yield_priority'
    owner.progress+=.3;states['owner'].y+=.3
    policy.prepare(states,ex,{},envelope,11.)
    assert not policy.resume_priorities


def test_aged_yield_allows_the_leader_of_a_grantees_leader_to_clear():
    owner=straight('owner',-35,-30,math.pi/2);owner.progress=20.
    owner.plan.edge_starts=[20.];owner.plan.edge_ends=[40.]
    ex={'owner':owner,'passing':straight('passing',-100,0),'middle':straight('middle',-65,0),
        'front':straight('front',-30,0)}
    ex['front'].plan.edge_starts=[5.];ex['front'].plan.edge_ends=[30.]
    states={'owner':VehicleState('owner',-35,-10,math.pi/2),'passing':VehicleState('passing',-100,0,0),
            'middle':VehicleState('middle',-65,0,0),'front':VehicleState('front',40,0,0)}
    policy=LocalTraffic();policy.prepare(states,ex,{},envelope,10.)
    boxes={n:[envelope(s)] for n,s in states.items()}
    assert policy.claim_passage('owner',owner,0,0,states,boxes,envelope)
    states['front'].x=-30.;boxes={n:[envelope(s)] for n,s in states.items()}
    policy.paused_by['owner']={('passing',0)};policy.yield_since['owner']=0.
    assert policy.claim_passage('front',ex['front'],0,0,states,boxes,envelope)


def test_point_conflict_yield_allows_approaching_its_safe_holding_boundary():
    owner=straight('owner',0,-50,math.pi/2);owner.progress=10.
    owner.plan.edge_starts=[10.];owner.plan.edge_ends=[70.]
    ego=straight('ego',-20,0);ego.plan.edge_starts=[5.];ego.plan.edge_ends=[30.]
    ex={'owner':owner,'ego':ego};states={'owner':VehicleState('owner',0,-40,math.pi/2),'ego':VehicleState('ego',-20,0,0)}
    policy=LocalTraffic(point_conflicts=True);policy.prepare(states,ex,{},envelope,0.)
    boxes={n:[envelope(s)] for n,s in states.items()}
    assert policy.claim_passage('owner',owner,0,0,states,boxes,envelope)
    assert policy.claim_passage('ego',ego,0,0,states,boxes,envelope)
    policy.prepare(states,ex,{},envelope,1.)
    assert policy.yield_holds['owner']['stop_s']>owner.progress+20.
    commands=policy.regulate(states,ex,{'owner':(8,0),'ego':(8,0)},{},1.,envelope)
    assert commands['owner'][0]>0

def test_following_truck_cannot_pause_its_leader_to_claim_the_same_turn():
    owner=straight('front',0,0);owner.plan.edge_starts=[10.];owner.plan.edge_ends=[25.]
    ego=straight('rear',-12,0);ego.plan.edge_starts=[35.];ego.plan.edge_ends=[50.]
    ex={'front':owner,'rear':ego}
    states={'front':VehicleState('front',0,0,0),'rear':VehicleState('rear',-12,0,0)}
    policy=LocalTraffic();policy.prepare(states,ex,{},envelope)
    boxes={n:[envelope(s)] for n,s in states.items()}
    assert policy.claim_passage('front',owner,0,0,states,boxes,envelope)
    assert not policy.claim_passage('rear',ego,0,0,states,boxes,envelope)
    assert not policy.paused_by
    assert policy.passage_denials['rear'][0]['blocked_by']=='front'
    assert policy.regulate(states,ex,{'front':(8,0),'rear':(8,0)},{},0,envelope)['front'][0]>0

def test_repair_configuration_has_executable_ingress_and_egress_for_dead_end_sites():
    from logistics_park.routing.service_network import ServiceNetwork
    from logistics_park.runtime.transport_routes import TransportMap
    from logistics_park.runtime.task_execution import ServiceRouteGeometry
    from logistics_park.scheduling.task_dispatch import DispatchMove
    root=Path(__file__).resolve().parents[1]
    cfg=json.loads((root/'config/runtime/industrialarea_ue_fleet24_repair.json').read_text())
    network=ServiceNetwork(root/cfg['network'],blocked_edges=cfg['blocked_candidate_edges'])
    geometry=ServiceRouteGeometry(network,TransportMap(root/cfg['map']))
    for origin,destination,required in [('W1-loading','W3-loading','access_164'),
            ('W4-unloading','W1-loading','access_165'),('Q3','W1-loading','access_167')]:
        route=network.route(origin,destination)
        assert route is not None and required in route['edge_ids']
        plan=geometry.plan(DispatchMove(1,'task','ego','visit',origin,destination,route,False))
        assert plan.length>0 and any(geometry.model.edges[k].get('reverse') for k in plan.edges)

def test_unused_future_permission_expires_but_entered_permission_is_retained():
    ex=straight('ego',0,0);ex.plan.edge_starts=[20.];ex.plan.edge_ends=[40.]
    states={'ego':VehicleState('ego',0,0,0)}
    policy=LocalTraffic();policy.prepare(states,{'ego':ex},{},envelope,0)
    assert policy.claim_passage('ego',ex,0,0,states,{'ego':[envelope(states['ego'])]},envelope)
    policy.prepare(states,{'ego':ex},{},envelope,9)
    assert not policy.passages and len(policy.released)==1
    assert not policy.claim_passage('ego',ex,0,0,states,{'ego':[envelope(states['ego'])]},envelope)
    ex.progress=20.;states['ego'].x=20.
    policy.prepare(states,{'ego':ex},{},envelope,12)
    assert policy.claim_passage('ego',ex,0,0,states,{'ego':[envelope(states['ego'])]},envelope)
    policy.prepare(states,{'ego':ex},{},envelope,30)
    assert ('ego',0) in policy.passages

def test_clear_crossing_withdraws_unentered_promise_instead_of_pausing_owner():
    owner=straight('owner',0,-30,math.pi/2)
    owner.plan.edge_starts=[20.];owner.plan.edge_ends=[40.]
    ego=straight('ego',-20,0);ego.plan.edge_starts=[5.];ego.plan.edge_ends=[30.]
    states={'owner':VehicleState('owner',0,-30,math.pi/2),'ego':VehicleState('ego',-20,0,0)}
    policy=LocalTraffic();policy.prepare(states,{'owner':owner,'ego':ego},{},envelope,0)
    boxes={n:[envelope(s)] for n,s in states.items()}
    assert policy.claim_passage('owner',owner,0,0,states,boxes,envelope)
    assert policy.claim_passage('ego',ego,0,0,states,boxes,envelope)
    assert ('owner',0) not in policy.passages and not policy.paused_by
    assert policy.released[0][0]==('owner',0)

def test_unused_permission_release_cancels_exit_booking_but_keeps_actual_road_occupancy():
    from logistics_park.runtime.transport_control import TransportController
    from logistics_park.scheduling.road_capacity import RoadCapacity
    graph=SimpleNamespace(edges={a:SimpleNamespace(storage_capacity=2) for a in ['outlet','actual']})
    c=TransportController.__new__(TransportController);c.capacity=RoadCapacity(graph)
    plan=SimpleNamespace(length=100.)
    c.local_traffic=SimpleNamespace(released=[(('ego',0),dict(plan=plan,end=40.))],passages={})
    c.local_exit_bookings={('ego','outlet'):(plan,51.344)}
    c.capacity.reserve('ego','outlet');c.capacity.update({'ego':VehicleState('ego',0,0,0,edge_id='actual')})
    c._release_unused_local_bookings()
    assert 'ego' not in c.capacity.committed['outlet'] and c.capacity.occupants['actual']=={'ego'}
    assert not c.local_exit_bookings and not c.local_traffic.released

def test_coupled_docks_do_not_admit_two_trucks_that_block_each_others_exit():
    from logistics_park.routing.service_network import ServiceNetwork
    root=Path(__file__).resolve().parents[1]
    cfg=json.loads((root/'config/runtime/industrialarea_ue_fleet24_repair.json').read_text())
    n=ServiceNetwork(root/cfg['network'],blocked_edges=cfg['blocked_candidate_edges'])
    d=TaskDispatcher(n,FleetRegistry(),follow_to_busy_destination=True)
    d.holders={'W4-loading':'front','W2-unloading':'remote'};d.positions={'front':'W4-loading'}
    assert d._blockers('W4-unloading','rear')=={'front'}
    # Dock admission coordination must not become a road access lock that
    # prevents the parked owner leaving toward a waiting place.
    assert d._route('W4-loading','Q7','front') is not None
    assert d._waiting_options('W4-loading','W2-unloading','front')

def test_grid_sweep_search_agrees_with_pairwise_sat_across_cell_boundaries():
    import random
    from logistics_park.scheduling.local_traffic import sweeps_intersect
    from logistics_park.safety.collision import OBB,overlaps
    rng=random.Random(29)
    for _ in range(50):
        one=[OBB(rng.uniform(-40,40),rng.uniform(-40,40),rng.uniform(-math.pi,math.pi),8.344,2.6) for i in range(7)]
        two=[OBB(rng.uniform(-40,40),rng.uniform(-40,40),rng.uniform(-math.pi,math.pi),8.344,2.6) for i in range(15)]
        assert sweeps_intersect(one,two,.35)==any(overlaps(a,b,.35) for a in one for b in two)

def test_waiting_vehicle_competes_for_unentered_bay_before_joining_road_queue(tmp_path):
    from logistics_park.scheduling.task_dispatch import Assignment,DispatchMove
    n=network(tmp_path);d=TaskDispatcher(n,FleetRegistry(),follow_to_busy_destination=True)
    d.assignments={'far':Assignment(TransportWork('farwork','L','U'),'far','farvisit',operation='UNLOADING',state='MOVING'),
        'near':Assignment(TransportWork('nearwork','L','U'),'near','nearvisit',operation='UNLOADING',state='WAITING')}
    d.moves={'far':DispatchMove(1,'farwork','far','farvisit','L','U',n.route('L','U'),False)}
    d.positions={'near':'Q'};d.holders={'Q':'near','U':'far'}
    d.promote_approaches({'far':(100.,False),'near':(20.,False)})
    assert d.holders['U']=='near' and d.holders['Q']=='near'
    assert d.assignments['near'].work.unloading_node=='U'

def test_empty_waiting_promise_can_transfer_without_changing_displaced_task(tmp_path):
    from logistics_park.scheduling.task_dispatch import Assignment,DispatchMove
    n=network(tmp_path);d=TaskDispatcher(n,FleetRegistry(),follow_to_busy_destination=True)
    work=TransportWork('ego-work','L','U');other=TransportWork('other-work','L','U')
    d.assignments={'ego':Assignment(work,'ego','ego-visit',operation='UNLOADING',state='MOVING',cargo_units=1),
        'far':Assignment(other,'far','far-visit',operation='UNLOADING',state='MOVING',cargo_units=1)}
    old_far=DispatchMove(2,'other-work','far','far-visit','L','Q',n.route('L','Q'),True)
    d.moves={'ego':DispatchMove(1,'ego-work','ego','ego-visit','L','U',n.route('L','U'),False),'far':old_far}
    d.holders={'Q':'far','U':'ego','L':'ego'};d.waiting_lenders={'far':100.}
    new=d.divert_to_waiting('ego','Q',n.route('L','Q'),reassign_from='far')
    assert d.holders['Q']=='ego' and d.holders['L']=='ego'
    assert d.moves['far'] is old_far and d.assignments['far'].work is other
    assert d.assignments['far'].cargo_units==1 and new.command_id==1
    d.holders['Q']='far';d.positions['far']='Q'
    d.moves['ego']=DispatchMove(1,'ego-work','ego','ego-visit','L','U',n.route('L','U'),False)
    with pytest.raises(ValueError):d.divert_to_waiting('ego','Q',n.route('L','Q'),reassign_from='far')

def test_clearance_recovery_allows_only_slow_motion_away_from_stopped_body():
    ex={'front':straight('front',9,0),'rear':straight('rear',0,0)};ex['rear'].finished=True
    states={'front':VehicleState('front',9,0,0),'rear':VehicleState('rear',0,0,0)}
    policy=LocalTraffic();policy.prepare(states,ex,{},envelope)
    result=policy.regulate(states,ex,{'front':(8,0),'rear':(0,0)},{},0,envelope)
    assert 0<result['front'][0]<=.5 and result['rear'][0]==0
    ex['rear'].finished=False;ex['front'].finished=True
    policy.prepare(states,ex,{},envelope)
    result=policy.regulate(states,ex,{'front':(0,0),'rear':(8,0)},{},1,envelope)
    assert result['rear'][0]==0

def test_assigned_parked_waiting_truck_can_clear_an_access_without_losing_cargo(monkeypatch):
    from logistics_park.routing.service_network import ServiceNetwork
    from logistics_park.runtime.transport_routes import TransportMap
    from logistics_park.runtime.transport_control import TransportController
    from logistics_park.runtime.task_execution import ServiceRouteGeometry,ContinuousTaskExecutor
    from logistics_park.runtime.queue_clearance import _clear_parked_waiting
    from logistics_park.scheduling.task_dispatch import Assignment
    root=Path(__file__).resolve().parents[1]
    cfg=json.loads((root/'config/runtime/industrialarea_ue_fleet24_repair.json').read_text())
    n=ServiceNetwork(root/cfg['network'],blocked_edges=cfg['blocked_candidate_edges']);model=TransportMap(root/cfg['map'])
    f=FleetRegistry();d=TaskDispatcher(n,f,follow_to_busy_destination=True)
    v=f.adopt_present_vehicle('ego',0,fully_inside=True,resources=[]);d.place_vehicle('ego',v.visit_id,'Q3',0,stopped=True)
    work=TransportWork('work','W1-loading','W3-unloading')
    d.assignments['ego']=Assignment(work,'ego',v.visit_id,operation='UNLOADING',state='WAITING',cargo_units=1)
    c=TransportController(model,{},local_traffic=True,retain_finished_vehicles=True)
    e=ContinuousTaskExecutor(d,ServiceRouteGeometry(n,model),c)
    site=n.services['Q3'];state=VehicleState('ego',*site['rear_xy_m'],math.radians(site['heading_deg']))
    e.attach('ego','Q3',state,0);e.clearance_attempts={}
    calls=[];build_plan=e.geometry.plan
    def counted(move):
        calls.append(move.destination)
        return build_plan(move)
    monkeypatch.setattr(e.geometry,'plan',counted)
    c.local_traffic.passage_denials={'blocked':[dict(blocked_by='ego',reason='passage_occupied_body')]}
    _clear_parked_waiting(e,{'ego':state},1)
    move=e.moves['ego']
    assert move.origin=='Q3' and move.destination!='Q3' and move.waiting
    assert d.holders['Q3']=='ego' and d.positions['ego']=='Q3'
    assert d.assignments['ego'].work is work and d.assignments['ego'].cargo_units==1
    assert not c.executions['ego'].finished
    assert math.dist(c.executions['ego'].plan.point(0),(state.x,state.y))<.15
    assert calls==[move.destination]  # Compile the selected route, not every Q.

def test_near_waiting_truck_can_take_unentered_other_bay_shared_dock_promise(tmp_path):
    from logistics_park.scheduling.task_dispatch import Assignment,DispatchMove
    n=network(tmp_path)
    n.services['L']['occupation_resources']=['shared-dock']
    n.services['U']['occupation_resources']=['shared-dock']
    d=TaskDispatcher(n,FleetRegistry(),follow_to_busy_destination=True)
    far=Assignment(TransportWork('farwork','L','U'),'far','fv',operation='UNLOADING',state='MOVING',cargo_units=1)
    near=Assignment(TransportWork('nearwork','L','U'),'near','nv',operation='LOADING',state='WAITING')
    d.assignments={'far':far,'near':near}
    move=DispatchMove(1,'farwork','far','fv','Q','U',{},False)
    d.moves={'far':move};d.positions={'near':'Q'};d.holders={'U':'far','Q':'near'}
    d.promote_approaches({'far':(500,True),'near':(20,False)})
    assert d.holders=={'U':'far','Q':'near'}
    d.promote_approaches({'far':(500,False),'near':(20,False)})
    assert d.holders=={'L':'near','Q':'near'}
    assert d.moves['far'] is move and far.cargo_units==1 and far.work.unloading_node=='U'
    # The remote unloading request must not steal the shared promise back.
    d.promote_approaches({'far':(500,False),'near':(20,False)})
    assert d.holders=={'L':'near','Q':'near'}
    d.holders={'U':'far','Q':'near'};d.positions['far']='U'
    d.promote_approaches({'far':(500,False),'near':(20,False)})
    assert d.holders=={'U':'far','Q':'near'}

def test_shared_dock_front_truck_precedes_shorter_route_of_truck_behind(tmp_path):
    from logistics_park.scheduling.task_dispatch import Assignment,DispatchMove
    n=network(tmp_path)
    for node in ('L','U'):n.services[node]['occupation_resources']=['dock']
    d=TaskDispatcher(n,FleetRegistry(),follow_to_busy_destination=True)
    work=TransportWork('work','L','U')
    d.assignments={'rear':Assignment(work,'rear','rv',operation='LOADING',state='MOVING'),
                   'front':Assignment(work,'front','fv',operation='UNLOADING',state='MOVING')}
    d.moves={'rear':DispatchMove(1,'work','rear','rv','Q','L',{},False),
             'front':DispatchMove(2,'work','front','fv','Q','U',{},False)}
    d.holders={'L':'rear'}
    priorities={'rear':(35,False,frozenset({'front'})),'front':(38,False,frozenset())}
    d.promote_approaches(priorities)
    assert d.holders=={'U':'front'}
    d.promote_approaches(priorities)
    assert d.holders=={'U':'front'}

def test_future_reserved_crossing_does_not_freeze_road_before_its_stop_line():
    ex={'ego':straight('ego',-40,0),'owner':straight('owner',0,-25,math.pi/2)}
    states={'ego':VehicleState('ego',-40,0,0),'owner':VehicleState('owner',0,-25,math.pi/2)}
    owner=ex['owner'];owner.plan.edge_starts=[25.];owner.plan.edge_ends=[35.]
    policy=LocalTraffic();policy.prepare(states,ex,{},envelope)
    assert policy.claim_passage('owner',owner,0,0,states,{n:[envelope(s)] for n,s in states.items()},envelope)
    policy.action_groups={'ego':[(0,0,40.,50.)],'owner':[(0,0,25.,35.)]}
    predicted=policy._forecast('ego',states['ego'],ex['ego'],8,None,envelope)
    assert max(b.x for b in predicted[0])<=-7.8+.75+1e-6
    result=policy.regulate(states,ex,{'ego':(8,0),'owner':(0,0)},{},0,envelope)
    assert result['ego'][0]==8  # Accelerate now; stop safely at the future gate.


def test_disjoint_opposing_junction_paths_can_claim_and_move_together():
    ex={'east':straight('east',-20,0),'west':straight('west',20,6,math.pi)}
    states={'east':VehicleState('east',-20,0,0),'west':VehicleState('west',20,6,math.pi)}
    policy=LocalTraffic();policy.prepare(states,ex,{},envelope)
    boxes={n:[envelope(s)] for n,s in states.items()}
    for name in states:
        ex[name].plan.edge_starts=[20.];ex[name].plan.edge_ends=[30.]
        assert policy.claim_passage(name,ex[name],0,0,states,boxes,envelope)
    policy.action_groups={n:[(0,0,20.,30.)] for n in states}
    result=policy.regulate(states,ex,{n:(8,0) for n in states},{},0,envelope)
    assert all(result[n][0]==8 for n in states)


def test_forecast_cannot_clip_real_braking_distance_at_a_stop_line():
    ex=straight('ego',0,0);state=VehicleState('ego',0,0,0,speed=8.)
    boxes,_=LocalTraffic()._forecast('ego',state,ex,0,3.,envelope)
    assert boxes[-1].x>50  # 8 m/s cannot physically brake within 3 m.
