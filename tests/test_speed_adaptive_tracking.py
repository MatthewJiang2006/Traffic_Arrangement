import math
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from logistics_park.runtime.transport_routes import RouteGeometry


def plan():
    model=SimpleNamespace(raw={'speed_adaptive_steering':True,'speed_limits':{'straight_m_s':8}},
        edges={'road':dict(centerline=[[0,0],[30,0]],kind='road',capacity_alias='r',speed_limit=8),
               'turn':dict(centerline=[[30,0],[32,1],[33,3]],kind='junction',maneuver='LEFT',capacity_alias='t',speed_limit=1.5)},
        aliases={'road':'r','turn':'t'},shared_resources={})
    return RouteGeometry(model,['road','turn'],[],'a',['b'])


def test_fast_straight_tracking_uses_gentler_correction():
    p=plan();short=p.steering((10,.2),.03,10,speed=1.5)
    long=p.steering((10,.2),.03,10,speed=8)
    assert abs(long)<abs(short)
    assert p.speed_cap(10)>1.5


def test_long_preview_does_not_cut_ahead_into_validated_turn():
    p=plan()
    for s in [29,31]:
        fast=p.steering(p.point(s),p.heading(s),s,speed=8)
        p.model.raw['speed_adaptive_steering']=False
        original=p.steering(p.point(s),p.heading(s),s,speed=8)
        assert fast==pytest.approx(original)
        p.model.raw['speed_adaptive_steering']=True


def test_high_speed_straight_feedback_remains_inside_original_error_limit():
    p=plan();x,y,yaw=0.,.2,.05;station=0.;peak=0.
    for _ in range(22):
        error,station=p.project((x,y),station,ahead=3);peak=max(peak,error)
        delta=p.steering((x,y),yaw,station,speed=8);dt=.13
        mid=yaw+.5*8*math.tan(delta)/2.3285*dt
        x+=8*math.cos(mid)*dt;y+=8*math.sin(mid)*dt;yaw+=8*math.tan(delta)/2.3285*dt
    assert peak<.7 and abs(y)<.15


def exit_plan():
    model=SimpleNamespace(raw={'speed_adaptive_steering':True,'speed_limits':{'straight_m_s':8}},
        edges={'turn':dict(centerline=[[-4,-4],[-1,-1],[0,0]],kind='junction',maneuver='LEFT',capacity_alias='t',speed_limit=1.5),
               'road':dict(centerline=[[0,0],[30,0]],kind='road',capacity_alias='r',speed_limit=8)},
        aliases={'road':'r','turn':'t'},shared_resources={})
    return RouteGeometry(model,['turn','road'],[],'a',['b'])


def test_curve_exit_corrects_body_yaw_then_restores_high_speed_preview():
    p=exit_plan();start=p.edge_starts[1]
    for offset in [1.,5.,8.]:
        s=start+offset
        fast=p.steering((offset,.2),-.15,s,speed=8)
        assert fast>0  # Correct negative residual yaw instead of following it.
    s=start+18
    fast=p.steering((18,.2),.03,s,speed=8)
    slow=p.steering((18,.2),.03,s,speed=1.5)
    assert abs(fast)<abs(slow)


def test_curve_exit_correction_keeps_complete_body_inside_lane_at_native_control_interval():
    p=exit_plan();x,y,yaw=0.,.21,-math.radians(9);station=p.edge_starts[1];peak=-math.inf
    for _ in range(20):
        _,station=p.project((x,y),station,ahead=3)
        speed=min(8,p.curve_exit_alignment_speed_cap((x,y),yaw,station))
        delta=p.steering((x,y),yaw,station,speed=speed);dt=.13
        mid=yaw+.5*speed*math.tan(delta)/2.3285*dt
        x+=speed*math.cos(mid)*dt;y+=speed*math.sin(mid)*dt;yaw+=speed*math.tan(delta)/2.3285*dt
        longitudinal=4.172*abs(math.cos(yaw))+1.3*abs(math.sin(yaw))
        if x+.75*math.cos(yaw)-longitudinal>=0:
            lateral=abs(y+.75*math.sin(yaw))+4.172*abs(math.sin(yaw))+1.3*abs(math.cos(yaw))
            peak=max(peak,lateral-1.9)
    assert math.isfinite(peak) and peak<=.05


def test_curve_exit_waits_for_measured_yaw_and_lateral_alignment_before_accelerating():
    p=exit_plan();s=p.edge_starts[1]+2.
    assert p.curve_exit_alignment_speed_cap((2,.02),math.radians(8),s)==2.0
    assert p.curve_exit_alignment_speed_cap((2,.21),0,s)==2.0
    assert math.isinf(p.curve_exit_alignment_speed_cap((2,.02),math.radians(2),s))
    assert math.isinf(p.curve_exit_alignment_speed_cap((18,.21),math.radians(8),p.edge_starts[1]+18))


def test_alignment_gate_does_not_apply_to_ordinary_straight_or_disabled_tracking():
    p=plan()
    assert math.isinf(p.curve_exit_alignment_speed_cap((10,.21),math.radians(8),10))
    p=exit_plan();p.model.raw['speed_adaptive_steering']=False
    assert math.isinf(p.curve_exit_alignment_speed_cap((2,.21),math.radians(8),p.edge_starts[1]+2))


def west_exit_plan():
    model=SimpleNamespace(raw={'speed_adaptive_steering':True},
        edges={'turn':dict(centerline=[[40,131],[38,134],[34.9817,135.107]],kind='junction',maneuver='LEFT',capacity_alias='t',speed_limit=1.5),
               'road':dict(centerline=[[34.9817,135.107],[-2.18,135.107]],kind='road',capacity_alias='r',speed_limit=8)},
        aliases={'road':'r','turn':'t'},shared_resources={})
    return RouteGeometry(model,['turn','road'],[],'a',['b'])


def test_recorded_diagonal_exit_requests_yaw_correction_instead_of_zero_steering():
    p=west_exit_plan();s=p.edge_starts[1]+3.844124298035638
    delta=p.steering((31.13757570196451,134.71228885906498),2.996195509664154,s,speed=1.5016390589029565)
    assert delta>0  # Correct residual yaw; envelope tests bound the resulting motion.


def test_recorded_entry_pose_stays_inside_lane_with_control_and_steering_delay():
    p=west_exit_plan();x,y,yaw=32.69745041846687,134.61563877780728,-3.102298510654001
    station=p.edge_starts[1]+2.2842495815332313;actual_delta=0.;peak=-math.inf
    for _ in range(30):
        _,station=p.project((x,y),station,ahead=3)
        speed=min(8,p.curve_exit_alignment_speed_cap((x,y),yaw,station))
        wanted=p.steering((x,y),yaw,station,speed=speed);dt=.15
        actual_delta+=(wanted-actual_delta)*min(1.,dt/.2)
        mid=yaw+.5*speed*math.tan(actual_delta)/2.3285*dt
        x+=speed*math.cos(mid)*dt;y+=speed*math.sin(mid)*dt;yaw+=speed*math.tan(actual_delta)/2.3285*dt
        diff=yaw-math.pi;along=4.172*abs(math.cos(diff))+1.3*abs(math.sin(diff))
        if 34.9817-(x+.75*math.cos(yaw))>=along+.25:
            excess=abs(y+.75*math.sin(yaw)-135.107)+4.172*abs(math.sin(diff))+1.3*abs(math.cos(diff))-1.9
            peak=max(peak,excess)
    assert math.isfinite(peak) and peak<=.05
