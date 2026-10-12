"""Reproduce the real round33 stopped-body prediction cycle."""
import json
from pathlib import Path
from logistics_park.runtime.transport_routes import TransportMap,RouteGeometry
from logistics_park.runtime.transport_control import Execution,envelope
from logistics_park.common.messages import VehicleState
from logistics_park.scheduling.local_traffic import LocalTraffic
from logistics_park.scheduling.conflict_timing import maneuver_groups
from logistics_park.safety.collision import overlaps
from test_local_traffic import straight

ROOT=Path(__file__).resolve().parents[1]


def test_actual_round33_pair_can_advance_and_stop_without_forecast_collision():
    r=json.loads((ROOT/'tests/fixtures/round33_prediction_cycle.json').read_text())
    model=TransportMap(ROOT/'config/topology/industrialarea_ue_internal_waiting_transport.json')
    states={};executions={}
    for n,d in r['execution_details'].items():
        plan=RouteGeometry(model,d['route_edges'],[d['checkpoint']],n,[d['destination']])
        executions[n]=Execution(plan,progress=d['progress_m'])
        states[n]=VehicleState(n,*r['positions'][n])
    local=LocalTraffic(point_conflicts=True)
    local.action_groups={n:maneuver_groups(ex.plan) for n,ex in executions.items()}
    for permit in r['local_permissions']:
        n=permit['vehicle_id']
        local.passages[(n,permit['start'])]={'plan':executions[n].plan,'end':permit['end_m'],'created_at':0,'sweep':[]}
    local.prepare(states,executions,{},envelope,0)
    local.timing.prepare(states,executions,local.action_groups,{},local,envelope,0)
    commands={n:(r['control_trace'][n][-2]['target_m_s'],0.) for n in states}
    result=local.regulate(states,executions,commands,{},0,envelope)
    assert result['Car_0_1'][0]>0 and result['Car_3_1'][0]>0
    assert local.braking_advice.keys()==states.keys()
    forecasts=[]
    for n in states:
        advice=local.braking_advice[n]
        assert advice['brake_after_s']>=2.5>advice['command_revalidation_s']
        forecast=local._forecast(n,states[n],executions[n],abs(result[n][0]),None,envelope,
                                brake_after_s=advice['brake_after_s'])[0]
        assert len(forecast)==33  # All 16 seconds, not a shortened horizon.
        assert forecast[-1]==forecast[-2]  # Full stop at the horizon.
        forecasts.append(forecast)
    assert all(not overlaps(a,b,local.margin_m) for a,b in zip(*forecasts))
    assert local.last_denials=={}


def test_prefix_cannot_move_into_a_close_stopped_body():
    executions={'ego':straight('ego',0,0),'parked':straight('parked',9.1,0)}
    executions['parked'].finished=True
    states={'ego':VehicleState('ego',0,0,0),'parked':VehicleState('parked',9.1,0,0)}
    local=LocalTraffic(point_conflicts=True)
    result=local.regulate(states,executions,{'ego':(8,0),'parked':(0,0)},{},0,envelope)
    assert result['ego'][0]==0
    assert local.last_denials['ego']['blocked_by']=='parked'
    assert local.last_denials['ego']['reason']=='predicted_body'
    assert not local.braking_advice


def test_prefix_never_overrides_clearance_pause_or_destination_hold():
    ex={'ego':straight('ego',0,0)};states={'ego':VehicleState('ego',0,0,0)}
    local=LocalTraffic(point_conflicts=True);local.clearance_pauses={'ego':{'owner'}}
    assert local.regulate(states,ex,{'ego':(8,0)},{},0,envelope)['ego'][0]==0
    local.clearance_pauses={}
    forecast=local._forecast('ego',states['ego'],ex['ego'],8,3,envelope,brake_after_s=4)[0]
    assert forecast[-1].x<=3+.75


def test_prefix_retains_capacity_limit():
    ex={'ego':straight('ego',0,0),'other':straight('other',0,10)}
    ex['other'].plan.model.aliases[ex['other'].plan.edges[0]]='road-ego'
    ex['ego'].plan.model.raw['physical_capacities']['road-ego']=1
    states={'ego':VehicleState('ego',0,0,0),'other':VehicleState('other',0,10,0)}
    local=LocalTraffic(point_conflicts=True)
    result=local.regulate(states,ex,{'ego':(8,0),'other':(0,0)},{},0,envelope)
    assert result['ego'][0]==0 and not local.braking_advice


def test_recovery_half_metre_speed_is_tried_despite_normal_crawl_filter():
    ex={'ego':straight('ego',0,0),'other':straight('other',0,3.2)}
    states={'ego':VehicleState('ego',0,0,0),'other':VehicleState('other',0,3.2,0)}
    assert overlaps(envelope(states['ego']),envelope(states['other']),.35)
    assert not overlaps(envelope(states['ego']),envelope(states['other']),.2)
    local=LocalTraffic(point_conflicts=True)
    result=local.regulate(states,ex,{'ego':(4,0),'other':(0,0)},{},0,envelope)
    assert 0<result['ego'][0]<=.5
    assert any(e['event']=='CLEARANCE_RECOVERY_CHECK' for e in local.events)


def test_hard_body_overlap_is_never_a_clearance_recovery_exemption():
    ex={'ego':straight('ego',0,0),'other':straight('other',0,2.8)}
    states={'ego':VehicleState('ego',0,0,0),'other':VehicleState('other',0,2.8,0)}
    local=LocalTraffic(point_conflicts=True)
    result=local.regulate(states,ex,{'ego':(4,0),'other':(0,0)},{},0,envelope)
    assert result['ego'][0]==0
    assert not any(e['event']=='CLEARANCE_RECOVERY_CHECK' for e in local.events)
