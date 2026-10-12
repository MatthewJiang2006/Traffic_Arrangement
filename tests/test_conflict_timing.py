import math
from types import SimpleNamespace
import pytest
from test_local_traffic import straight,envelope
from logistics_park.common.messages import VehicleState
from logistics_park.scheduling.local_traffic import LocalTraffic
from logistics_park.scheduling.conflict_timing import arrival_profile,ConflictTiming,ArrivalProfile
from logistics_park.runtime.speed_limits import SpeedLimits,edge_speed_limit
from logistics_park.map.geometry import Polyline


def test_eta_endpoint_tolerance_does_not_index_past_samples():
    profile=ArrivalProfile([0.,10.],[0.,4.])
    assert profile.eta(10.+5e-7)==4.
    assert math.isinf(profile.eta(10.+2e-6))
    stationary=ArrivalProfile([10.],[0.])
    assert stationary.eta(10.+5e-7)==0.


def test_eta_gear_transition_keeps_stations_monotonic():
    ex=straight('a',0,0);ex.progress=10.1
    ex.plan.checkpoints=[{'s':10.,'event':'GEAR_CHANGE'}]
    p=arrival_profile(ex,VehicleState('a',0,0,0),0)
    assert all(b>=a for a,b in zip(p.stations,p.stations[1:]))


def test_eta_integrates_actual_speed_and_acceleration():
    ex=straight('a',0,0);state=VehicleState('a',0,0,0,speed=2)
    profile=arrival_profile(ex,state,0)
    assert profile.eta(10)==pytest.approx((-2+math.sqrt(16))/.6,abs=.02)
    stopped=arrival_profile(ex,VehicleState('a',0,0,0),0)
    assert stopped.eta(10)>profile.eta(10)
    assert math.isinf(arrival_profile(ex,state,0,hold=3).eta(10))


def crossing():
    ex={'far':straight('far',0,-16,math.pi/2),'near':straight('near',-15,0)}
    states={'far':VehicleState('far',0,-16,math.pi/2,speed=2),'near':VehicleState('near',-15,0,0,speed=2)}
    groups={'far':[(0,0,16.,26.)],'near':[(0,0,15.,25.)]}
    policy=LocalTraffic()
    policy.timing.prepare(states,ex,groups,{},policy,envelope,0)
    return policy,ex,states,groups


def test_nearer_equal_speed_vehicle_precedes_dict_order_and_delays_other_window():
    policy,ex,states,groups=crossing();timing=policy.timing
    assert timing.order==['near','far']
    assert timing.windows['near']['delay_s']==0
    assert timing.windows['far']['delay_s']>0
    assert 0<timing.rolling_cap('far',8)<8
    assert timing.rolling_cap('near',8)==8
    conflict=timing.conflict(timing.sweep(ex['far'].plan,groups['far'][0],envelope),timing.sweep(ex['near'].plan,groups['near'][0],envelope))
    f0,f1,n0,n1=conflict
    fp=arrival_profile(ex['far'],states['far'],0);np=arrival_profile(ex['near'],states['near'],0)
    assert fp.eta(f0)+timing.windows['far']['delay_s']>=np.eta(n1+.5)+timing.buffer_s-1e-6


def test_disjoint_opposite_windows_do_not_create_speed_delay():
    ex={'a':straight('a',-20,0),'b':straight('b',20,6,math.pi)}
    states={'a':VehicleState('a',-20,0,0),'b':VehicleState('b',20,6,math.pi)}
    groups={n:[(0,0,20.,30.)] for n in ex};p=LocalTraffic()
    p.timing.prepare(states,ex,groups,{},p,envelope,0)
    assert all(w['delay_s']==0 for w in p.timing.windows.values())


def test_lane_leader_keeps_priority_over_faster_follower():
    ex={'rear':straight('rear',0,0),'front':straight('front',14,0)}
    states={'rear':VehicleState('rear',0,0,0,speed=4),'front':VehicleState('front',14,0,0,speed=1)}
    groups={'rear':[(0,0,40.,50.)],'front':[(0,0,26.,36.)]};p=LocalTraffic()
    p.timing.prepare(states,ex,groups,{},p,envelope,0)
    assert p.timing.order==['front','rear']
    assert p.timing.windows['rear']['delay_s']==0  # Physical following guard applies.


def test_moving_far_future_owner_can_yield_only_with_clear_braking_path():
    ex={'far':straight('far',0,-30,math.pi/2),'near':straight('near',-15,0)}
    states={'far':VehicleState('far',0,-30,math.pi/2,speed=.5),'near':VehicleState('near',-15,0,0)}
    for n,entry in [('far',30),('near',15)]:ex[n].plan.edge_starts=[entry];ex[n].plan.edge_ends=[entry+10]
    p=LocalTraffic();p.prepare(states,ex,{},envelope)
    boxes={n:[envelope(s)] for n,s in states.items()}
    assert p.claim_passage('far',ex['far'],0,0,states,boxes,envelope)
    p.timing.windows={'near':{'entry_eta_s':3},'far':{'entry_eta_s':10}}
    assert p.claim_passage('near',ex['near'],0,0,states,boxes,envelope)
    assert ('far',0) not in p.passages and p.released
    assert states['far'].speed==.5  # No physical state or occupancy is revoked.


def test_entered_moving_owner_cannot_be_preempted_by_a_new_eta():
    ex={'far':straight('far',0,-30,math.pi/2),'near':straight('near',-15,0)}
    states={'far':VehicleState('far',0,-5,math.pi/2,speed=.5),'near':VehicleState('near',-15,0,0)}
    for n,entry in [('far',30),('near',15)]:ex[n].plan.edge_starts=[entry];ex[n].plan.edge_ends=[entry+10]
    ex['far'].progress=25
    p=LocalTraffic();p.prepare(states,ex,{},envelope)
    p.passages[('far',0)]={'plan':ex['far'].plan,'end':40,'sweep':[envelope(VehicleState('probe',0,0,math.pi/2))]}
    p.timing.windows={'near':{'entry_eta_s':0},'far':{'entry_eta_s':10}}
    assert not p.claim_passage('near',ex['near'],0,0,states,{n:[envelope(s)] for n,s in states.items()},envelope)
    assert ('far',0) in p.passages


def test_freeflow_speed_override_is_opt_in_and_preserves_curves():
    limits=SpeedLimits(straight_m_s=8)
    edge=dict(kind='junction',maneuver='STRAIGHT',speed_limit=2.,capacity_alias='a')
    model=SimpleNamespace(raw={},physical_paths={'a':Polyline(((0,0),(10,0),(20,0)))})
    assert edge_speed_limit(edge,model,limits)==2
    model.raw['freeflow_straight_junctions']=True
    assert edge_speed_limit(edge,model,limits)==8
    model.physical_paths['a']=Polyline(((0,0),(10,0),(10,10)))
    assert edge_speed_limit(edge,model,limits)==2
    model.physical_paths['a']=Polyline(((0,0),(10,0),(20,0)))
    assert edge_speed_limit(dict(edge,maneuver='RIGHT',speed_limit=1.5),model,limits)==1.5

def test_earlier_eta_cannot_preempt_a_future_owner_that_cannot_brake_clear():
    ex={'far':straight('far',0,-30,math.pi/2),'near':straight('near',-15,0)}
    states={'far':VehicleState('far',0,-30,math.pi/2,speed=8),'near':VehicleState('near',-15,0,0)}
    for n,entry in [('far',30),('near',15)]:ex[n].plan.edge_starts=[entry];ex[n].plan.edge_ends=[entry+10]
    p=LocalTraffic();p.prepare(states,ex,{},envelope)
    p.passages[('far',0)]={'plan':ex['far'].plan,'end':40,'sweep':[envelope(VehicleState('probe',0,0,math.pi/2))]}
    p.timing.windows={'near':{'entry_eta_s':0},'far':{'entry_eta_s':10}}
    assert not p.claim_passage('near',ex['near'],0,0,states,{n:[envelope(s)] for n,s in states.items()},envelope)
    assert ('far',0) in p.passages and not p.released

def test_actual_local_entry_without_a_live_permission_is_rejected():
    import json
    from pathlib import Path
    from logistics_park.routing.service_network import ServiceNetwork
    from logistics_park.runtime.transport_routes import TransportMap
    from logistics_park.runtime.transport_control import TransportController
    from logistics_park.runtime.task_execution import ServiceRouteGeometry
    from logistics_park.scheduling.task_dispatch import DispatchMove
    root=Path(__file__).resolve().parents[1]
    cfg=json.loads((root/'config/runtime/industrialarea_ue_fleet4_timing.json').read_text())
    network=ServiceNetwork(root/cfg['network']);model=TransportMap(root/cfg['map'])
    route=network.route('W5-loading','W3-unloading')
    plan=ServiceRouteGeometry(network,model).plan(DispatchMove(1,'work','ego','visit','W5-loading','W3-unloading',route,False))
    c=TransportController(model,{'ego':plan},local_traffic=True,retain_finished_vehicles=True)
    c.activate('ego',0);c.executions['ego'].progress=3.
    state=VehicleState('ego',*plan.point(3),plan.heading(3))
    with pytest.raises(RuntimeError,match='actual local entry without a live passage'):
        c.tick({'ego':state},1)
    assert c.local_unauthorized_count==1


def test_future_permission_is_kept_when_braking_cannot_reach_stop_line():
    ex={'far':straight('far',0,-20,math.pi/2),'near':straight('near',-15,0)}
    states={'far':VehicleState('far',0,-20,math.pi/2,speed=3.5),'near':VehicleState('near',-15,0,0)}
    for n,entry in [('far',20),('near',15)]:
        ex[n].plan.edge_starts=[entry];ex[n].plan.edge_ends=[entry+10]
    p=LocalTraffic();p.prepare(states,ex,{},envelope)
    p.passages[('far',0)]={'plan':ex['far'].plan,'end':30,'sweep':[envelope(VehicleState('probe',0,0,math.pi/2))]}
    p.timing.windows={'near':{'entry_eta_s':0},'far':{'entry_eta_s':10}}
    assert not p.claim_passage('near',ex['near'],0,0,states,{n:[envelope(s)] for n,s in states.items()},envelope)
    assert p.passage_denials['near'][0]['reason']=='passage_stop_line_committed'
    assert ('far',0) in p.passages and not p.released


def test_stationary_leg_switch_offset_is_not_a_new_entry(monkeypatch):
    import json
    from pathlib import Path
    from logistics_park.routing.service_network import ServiceNetwork
    from logistics_park.runtime.transport_routes import TransportMap
    from logistics_park.runtime.transport_control import TransportController
    from logistics_park.runtime.task_execution import ServiceRouteGeometry
    from logistics_park.scheduling.task_dispatch import DispatchMove
    root=Path(__file__).resolve().parents[1]
    cfg=json.loads((root/'config/runtime/industrialarea_ue_fleet8_timing.json').read_text())
    network=ServiceNetwork(root/cfg['network']);model=TransportMap(root/cfg['map'])
    route=network.route('W2-loading','Q3')
    plan=ServiceRouteGeometry(network,model).plan(DispatchMove(1,'work','ego','visit','W2-loading','Q3',route,True))
    c=TransportController(model,{'ego':plan},local_traffic=True,retain_finished_vehicles=True)
    c.activate('ego',0);c.executions['ego'].finished=True
    state=VehicleState('ego',*plan.point(.3),plan.heading(.3))
    c.continue_route('ego',plan,1,state=state)
    assert c.executions['ego'].initial_station==pytest.approx(.3,abs=.01)
    monkeypatch.setattr(c.local_traffic,'claim_passage',lambda *args:False)
    c.tick({'ego':state},1)
    assert c.local_unauthorized_count==0
    moved=VehicleState('ego',*plan.point(.65),plan.heading(.65))
    with pytest.raises(RuntimeError,match='actual local entry without a live passage'):
        c.tick({'ego':moved},2)
    assert c.local_unauthorized_count==1


def test_earlier_clearance_needs_no_wait_for_later_high_priority_crossing():
    ex={'far':straight('far',0,-35,math.pi/2),'near':straight('near',-15,0)}
    states={'far':VehicleState('far',0,-35,math.pi/2,speed=.5),'near':VehicleState('near',-15,0,0,speed=4)}
    groups={'far':[(0,0,0.,50.)],'near':[(0,0,15.,25.)]}
    p=LocalTraffic();p.passages[('far',0)]={'plan':ex['far'].plan,'end':50}
    p.timing.prepare(states,ex,groups,{},p,envelope,0)
    assert p.timing.order==['far','near']
    assert p.timing.windows['near']['delay_s']==0
    assert p.timing.rolling_cap('near',8)==8
