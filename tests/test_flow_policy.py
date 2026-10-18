"""Flow improvements preserve physical occupancy and bounded turning speeds."""
import math
from types import SimpleNamespace
import pytest
from logistics_park.runtime.traffic_policy import following_speed, stop_line_speed
from logistics_park.runtime.speed_limits import edge_speed_limit, SpeedLimits
from logistics_park.runtime.transport_control import envelope
from logistics_park.common.messages import VehicleState
from logistics_park.scheduling.local_traffic import LocalTraffic
from logistics_park.scheduling.task_dispatch import TaskDispatcher, TransportWork
from logistics_park.scheduling.fleet_lifecycle import FleetRegistry
from test_local_traffic import straight
from test_traffic_routing import network


def test_compact_following_keeps_stopping_distance_and_special_room():
    separation=8.344+1.2
    assert following_speed(separation,0)==pytest.approx(0)
    # A moving leader may brake immediately while the follower's old command
    # remains live. At the static minimum gap it must run slower than 3 m/s.
    selected=following_speed(separation,3)
    assert selected>0 and selected*1.2+selected**2/1.2<=3**2/1.2+1e-10
    assert following_speed(8.344+3,0,special=True)==0
    assert following_speed(12,0,special=True)<following_speed(12,0)


def test_stop_line_brakes_without_far_away_eta_crawl():
    assert stop_line_speed(.25)==0
    assert stop_line_speed(64)>8
    assert stop_line_speed(10)<math.sqrt(1.2*10)


def test_turn_speed_is_curvature_limited_and_uturn_unchanged():
    radius=3.2
    points=[(radius*math.cos(a),radius*math.sin(a)) for a in (0,.1,.2,.3)]
    model=SimpleNamespace(raw={'curvature_turn_speeds':True},physical_paths={'bend':SimpleNamespace(points=points)})
    edge={'speed_limit':1.5,'kind':'junction','maneuver':'RIGHT','capacity_alias':'bend'}
    limits=SpeedLimits(straight_m_s=8,turn_m_s=2.2)
    assert edge_speed_limit(edge,model,limits)==pytest.approx(math.sqrt(1.2*radius))
    assert edge_speed_limit(dict(edge,kind='uturn'),model,limits)==1.5


def test_busy_dock_with_full_waiting_area_does_not_feed_road_queue(tmp_path):
    n=network(tmp_path,alternatives=False);f=FleetRegistry()
    d=TaskDispatcher(n,f,service_s=0,follow_to_busy_destination=True,offroad_busy_wait=True)
    for name,node in [('ego','L'),('busy','U'),('waiting','Q')]:
        v=f.adopt_present_vehicle(name,0,fully_inside=True,resources=[])
        d.place_vehicle(name,v.visit_id,node,0,stopped=True)
    d.submit(TransportWork('work','L','U'));d.tick(0)
    assert d.tick(1)==[]
    assert d.positions['ego']=='L' and d.holders['U']=='busy'


def crossing_policy():
    ex={'a':straight('a',0,-30,math.pi/2),'b':straight('b',-10,0)}
    states={'a':VehicleState('a',0,-30,math.pi/2,speed=2),'b':VehicleState('b',-10,0,0,speed=4)}
    for e in ex.values():e.plan.edge_starts=[0.];e.plan.edge_ends=[50.]
    policy=LocalTraffic(point_conflicts=True)
    policy.action_groups={n:((0,0,0.,50.),) for n in ex}
    policy.prepare(states,ex,{},envelope)
    b=ex['b'].plan
    policy.passages['b',0]={'plan':b,'end':50.,'sweep':[envelope(VehicleState('b',*b.point(s),0)) for s in range(62)],'created_at':0}
    policy.last_targets={'b':4.}
    return policy,ex,states


def test_different_conflict_windows_share_permission_before_whole_exit():
    policy,ex,states=crossing_policy()
    assert policy.claim_passage('a',ex['a'],0,0,states,{n:[envelope(s)] for n,s in states.items()},envelope)
    assert len(policy.passages)==2 and policy.compatible
    assert not policy.paused_by


def test_stopped_or_simultaneous_conflicting_owner_cannot_share():
    policy,ex,states=crossing_policy()
    policy.last_targets['b']=0
    assert not policy._time_compatible('a','b',ex['a'],0,states,envelope)
    policy.last_targets['b']=4
    states['a'].y=-10;ex['a'].progress=20;states['a'].speed=4
    assert not policy._time_compatible('a','b',ex['a'],0,states,envelope)


def test_parked_waiter_leaves_only_with_known_moving_owner_clearance(tmp_path):
    n=network(tmp_path,alternatives=False);f=FleetRegistry()
    d=TaskDispatcher(n,f,service_s=0,follow_to_busy_destination=True,offroad_busy_wait=True)
    for name,node in [('ego','L'),('busy','U')]:
        v=f.adopt_present_vehicle(name,0,fully_inside=True,resources=[])
        d.place_vehicle(name,v.visit_id,node,0,stopped=True)
    d.submit(TransportWork('work','L','U'));d.tick(0)
    move=d.tick(1)[0]
    d.departed(move.command_id,'ego',move.visit_id,2,tail_clear=True)
    d.arrived(move.command_id,'ego',move.visit_id,3,stopped=True)
    assert d.tick(4)==[]  # A service deadline is not a clear exit.
    d.clearance_estimates['busy']=1000
    assert d.tick(5)==[]
    d.clearance_estimates['busy']=5
    departure=d.tick(6)[0]
    assert departure.destination=='U' and d.holders['U']=='busy'
    assert any(x['event']=='WAITING_ETA_DEPARTURE' for x in d.events)


def test_following_can_share_leader_permission_but_reverse_needs_room():
    owner=straight('b',12,0);ego=straight('a',0,0)
    executions={'a':ego,'b':owner}
    states={'a':VehicleState('a',0,0,0,speed=2),'b':VehicleState('b',12,0,0,speed=2)}
    for ex in executions.values():
        ex.plan.edge_starts=[0.];ex.plan.edge_ends=[50.]
        ex.plan.model.edges={ex.plan.edges[0]:{'kind':'road'}}
    p=LocalTraffic(point_conflicts=True);p.action_groups={n:((0,0,0.,50.),) for n in executions}
    p.prepare(states,executions,{},envelope)
    p.passages['b',0]={'plan':owner.plan,'end':50.,'sweep':[], 'created_at':0}
    p.last_targets={'b':2}
    assert p._time_compatible('a','b',ego,0,states,envelope)
    owner.plan.model.edges[owner.plan.edges[0]]['reverse']=True
    assert not p._time_compatible('a','b',ego,0,states,envelope)


def test_parking_ingress_respects_in_transit_waiting_bookings(tmp_path):
    n=network(tmp_path,alternatives=False);d=TaskDispatcher(n,FleetRegistry(),offroad_busy_wait=True)
    route={'must_be_empty_nodes':['Q'],'edge_ids':[]}
    d.holders['Q']='other'  # Booking, not yet a parked body.
    assert not d._parking_approach_clear(route,'ego')
    assert d._parking_approach_clear(route,'other')
    d.holders.clear()
    assert d._parking_approach_clear(route,'ego')


def test_road_clearance_detects_followers_and_excludes_opposite_lane():
    from logistics_park.runtime.queue_clearance import blocked_followers
    ex={'front':straight('front',15,0),'behind':straight('behind',0,0),
        'opposite':straight('opposite',30,0,math.pi)}
    states={'front':VehicleState('front',15,0,0), 'behind':VehicleState('behind',0,0,0),
        'opposite':VehicleState('opposite',30,0,math.pi)}
    local=LocalTraffic();c=SimpleNamespace(executions=ex,local_traffic=local)
    assert blocked_followers(c,states,'front')=={'behind'}
    local.last_denials={'opposite':{'blocked_by':'front','reason':'predicted_body'}}
    assert blocked_followers(c,states,'front')=={'behind','opposite'}


def test_w5_front_can_keep_its_prefix_and_divert_past_micrometre_join():
    from pathlib import Path
    from dataclasses import replace
    from logistics_park.routing.service_network import ServiceNetwork
    from logistics_park.runtime.transport_routes import TransportMap
    from logistics_park.runtime.task_execution import ServiceRouteGeometry
    from logistics_park.runtime.transport_control import TransportController
    from logistics_park.scheduling.task_dispatch import DispatchMove
    root=Path(__file__).resolve().parents[1]
    n=ServiceNetwork(root/'config/topology/industrialarea_ue_parking24_network.json',blocked_edges={'access_133','access_134','access_167'})
    m=TransportMap(root/'config/topology/industrialarea_ue_flow_optimized_transport.json');g=ServiceRouteGeometry(n,m)
    prefix=['fleet24_depart_12','turn_j_0_2_W_N_single_single','road_j_0_2_j_1_2','turn_j_1_2_S_W_single_single',
        'road_j_1_2_j_1_1__p0','road_j_1_2_j_1_1__p1','road_j_1_2_j_1_1__p2','road_j_1_2_j_1_1__p3',
        'turn_j_1_1_E_N_single_single','road_j_1_1_j_2_1__service_0','road_j_1_1_j_2_1__service_1','road_j_1_1_j_2_1__service_2']
    move=DispatchMove(1,'work','front','visit','fleet24-start-12','Q1',{'edge_ids':prefix+['access_028','access_029','access_030','access_081','access_068']},True)
    old=g.plan(move)
    suffix=n.route('access_point_237','Q22',incoming_edge=prefix[-1],occupied_nodes={'Q2','W5-loading'})
    new=g.plan(replace(move,destination='Q22',route=dict(suffix,edge_ids=prefix+suffix['edge_ids'])))
    c=TransportController(m,{'front':old},local_traffic=True);c.activate('front',0)
    ex=c.executions['front'];index=old.edges.index('road_j_1_1_j_2_1__service_2__test0');ex.progress=old.edge_starts[index]-.05
    deviations=[math.dist(old.point(s/4),new.point(s/4)) for s in range(math.ceil((ex.progress+6)*4)+1)]
    assert 1e-6<max(deviations)<.001
    state=VehicleState('front',*old.point(ex.progress),old.heading(ex.progress))
    c.reroute_stopped('front',new,1,state=state)
    assert c.executions['front'].plan is new
    assert new.stops==['Q22']


def test_cached_service_envelopes_match_uncached_positions_and_track_geometry_changes():
    from logistics_park.scheduling.task_dispatch import _access_body_samples
    from logistics_park.map.geometry import Polyline
    points=((0.,0.),(3.,1.),(6.,1.))
    for reverse in (False,True):
        cached=_access_body_samples(points,reverse);line=Polyline(points);reference=[]
        for i in range(math.ceil(line.length/.5)+1):
            (x,y),heading=line.at(min(i*.5,line.length));heading+=math.pi if reverse else 0
            reference.append(envelope(VehicleState('truck',x,y,heading)))
        assert cached==tuple(reference)
        assert _access_body_samples(points,reverse) is cached
        assert _access_body_samples(((0.,0.),(3.,2.),(6.,1.)),reverse)!=cached
