"""Replay round36's two actually entered reverse actions, without teleporting."""
import json
import math
from pathlib import Path
from unittest.mock import patch
from logistics_park.runtime.transport_routes import TransportMap,RouteGeometry
from logistics_park.runtime.transport_control import Execution,envelope
from logistics_park.common.messages import VehicleState
from logistics_park.scheduling.local_traffic import LocalTraffic
from logistics_park.scheduling.conflict_timing import maneuver_groups
from logistics_park.safety.collision import overlaps

ROOT=Path(__file__).resolve().parents[1]


def reverse_pair():
    r=json.loads((ROOT/'tests/fixtures/round36_reverse_cycle.json').read_text())
    model=TransportMap(ROOT/'config/topology/industrialarea_ue_internal_waiting_transport.json')
    states={};executions={};local=LocalTraffic(point_conflicts=True)
    for n,d in r['execution_details'].items():
        plan=RouteGeometry(model,d['route_edges'],[d['checkpoint']],n,[d['destination']])
        executions[n]=Execution(plan,progress=d['progress_m'])
        states[n]=VehicleState(n,*r['positions'][n])
    local.action_groups={n:maneuver_groups(ex.plan) for n,ex in executions.items()}
    for permit in r['local_permissions']:
        n=permit['vehicle_id'];plan=executions[n].plan
        lo=plan.edge_starts[permit['start']];hi=min(plan.length,permit['end_m']+11.344)
        count=math.ceil((hi-lo)/.25)
        stations=[(lo+(hi-lo)*i/count,envelope(VehicleState(n,*plan.point(lo+(hi-lo)*i/count),
                    plan.heading(lo+(hi-lo)*i/count)))) for i in range(count+1)]
        local.passages[n,permit['start']]=dict(plan=plan,start=permit['start'],end=permit['end_m'],
            created_at=r['elapsed_s']-permit['age_s'],stations=stations,sweep=[b for _,b in stations])
    local.prepare(states,executions,{},envelope,r['elapsed_s'])
    return local,executions,states,r['elapsed_s']


def test_round36_entered_reverse_pair_has_one_order_and_safe_motion():
    local,ex,states,now=reverse_pair()
    assert 'Car_3_0' not in local.maneuver_holds
    assert local.maneuver_holds['Car_4_1']['blocked_by']=='Car_3_0'
    local.timing.prepare(states,ex,local.action_groups,{},local,envelope,now)
    result=local.regulate(states,ex,{'Car_3_0':(-1.,0),'Car_4_1':(0.,0)},{},now,envelope)
    assert result['Car_3_0'][0]<0 and result['Car_4_1'][0]==0
    a=local._forecast('Car_3_0',states['Car_3_0'],ex['Car_3_0'],abs(result['Car_3_0'][0]),None,envelope)[0]
    b=local._forecast('Car_4_1',states['Car_4_1'],ex['Car_4_1'],0,None,envelope)[0]
    assert len(a)==len(b)==33
    assert all(not overlaps(x,y,.35) for x,y in zip(a,b))
    assert local.last_denials=={}
    assert len(local.passages)==2  # Entered occupancy is never revoked.
    assert any(e['event']=='SPECIAL_MANEUVER_PRECEDENCE' for e in local.events)


def test_intersecting_reverse_cannot_bypass_admission_by_eta_forecast():
    local,ex,states,_=reverse_pair()
    local.last_targets['Car_3_0']=1.
    with patch.object(local,'_forecast',side_effect=AssertionError('time exception must not run')):
        assert not local._time_compatible('Car_4_1','Car_3_0',ex['Car_4_1'],0,states,envelope)


def test_reverse_precedence_cannot_override_a_real_stopped_body():
    local,ex,states,now=reverse_pair()
    p=ex['Car_3_0'].plan
    states['Car_4_1']=VehicleState('Car_4_1',*p.point(ex['Car_3_0'].progress+5),p.heading(ex['Car_3_0'].progress))
    local.braking={n:local._forecast(n,s,ex[n],0,None,envelope) for n,s in states.items()}
    result=local.regulate(states,ex,{'Car_3_0':(-1.,0),'Car_4_1':(0.,0)},{},now,envelope)
    assert result['Car_3_0'][0]==0
    assert local.last_denials['Car_3_0']['reason']=='predicted_body'


def test_reverse_order_does_not_hold_after_older_action_clears():
    local,ex,states,now=reverse_pair()
    ex['Car_3_0'].progress=ex['Car_3_0'].plan.length;ex['Car_3_0'].finished=True
    local.prepare(states,ex,{},envelope,now+20)
    assert ('Car_3_0',49) not in local.passages
    assert 'Car_4_1' not in local.maneuver_holds
