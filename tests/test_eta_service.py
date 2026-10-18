import math
from types import SimpleNamespace as NS
import pytest
from logistics_park.scheduling.eta_service import ETAService
from logistics_park.scheduling.conflict_timing import ArrivalProfile


def test_shared_profile_multi_point_queries_and_hold_are_consistent(monkeypatch):
    calls=[]
    def profile(*args,**kwargs):
        calls.append(1);return ArrivalProfile([0.,10.,20.],[0.,5.,10.])
    monkeypatch.setattr('logistics_park.scheduling.conflict_timing.arrival_profile',profile)
    service=ETAService();ex=NS(plan=NS(length=20.),progress=0.,checkpoint=0,finished=False)
    state=NS(speed=0.,timestamp=1.)
    a=service.query('car',ex,state,1.,10.,hold=10.)
    b=service.query('car',ex,state,1.,20.,hold=10.)
    assert a.eta_s==5. and a.reachable
    assert b.freeflow_s==10. and math.isinf(b.eta_s) and not b.reachable
    assert a.version==b.version and len(calls)==1
    c=service.query('car',ex,state,1.,20.,hold=None)
    assert c.reachable and c.eta_s==10. and len(calls)==1
    ex.checkpoint=1;service.query('car',ex,state,1.,20.)
    assert len(calls)==2
    service.query('car',ex,state,2.,20.)
    assert len(calls)==3
    with pytest.raises(ValueError):service.query('car',ex,state,1.,20.)


def test_unreachable_horizon_is_unknown_not_zero(monkeypatch):
    monkeypatch.setattr('logistics_park.scheduling.conflict_timing.arrival_profile',lambda *a,**k:ArrivalProfile([0.],[0.]))
    service=ETAService();ex=NS(plan=NS(length=20.),progress=0.,checkpoint=0,finished=False)
    result=service.query('car',ex,NS(speed=0.,timestamp=1.),1.,20.)
    assert not result.reachable and math.isinf(result.eta_s) and result.reason=='outside_prediction_horizon'


def test_profile_reuse_has_bounded_age_and_fresh_hold_state(monkeypatch):
    calls=[]
    def profile(*a,**k):calls.append(1);return ArrivalProfile([0.,20.],[0.,10.])
    monkeypatch.setattr('logistics_park.scheduling.conflict_timing.arrival_profile',profile)
    service=ETAService();ex=NS(plan=NS(length=20.),progress=0.,checkpoint=0,finished=False)
    state=NS(speed=0.,timestamp=1.)
    service.query('car',ex,state,1.,20.)
    blocked=service.query('car',ex,state,1.1,20.,hold=10.)
    assert not blocked.reachable and len(calls)==1
    service.query('car',ex,state,1.5,20.)
    assert len(calls)==2
    ex.progress=3.;service.query('car',ex,state,1.6,20.)
    assert len(calls)==3
