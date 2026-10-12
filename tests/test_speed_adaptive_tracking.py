import math
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
