"""Berth arbitration uses measured speed and complete parked route forecasts."""
from types import SimpleNamespace
import pytest
from logistics_park.common.messages import VehicleState
from logistics_park.map.geometry import Polyline
from logistics_park.runtime.transport_routes import RouteGeometry
from logistics_park.runtime.task_execution import ContinuousTaskExecutor
from test_task_dispatch import admission_setup


def straight_plan(length,speed):
    edge=dict(centerline=[[0,0],[length,0]],speed_limit=speed,kind='road',maneuver='STRAIGHT',resource=None,capacity_alias='road',reverse=False)
    model=SimpleNamespace(edges={'road':edge},aliases={'road':'road'},zones={},shared_resources={},physical_paths={'road':Polyline(((0,0),(length,0)))},raw={'speed_limits':dict(straight_m_s=8,turn_m_s=2.2,uturn_m_s=1,approach_deceleration_m_s2=.6)})
    return RouteGeometry(model,['road'],[], 'A',['L'])


def executor():
    e=ContinuousTaskExecutor.__new__(ContinuousTaskExecutor)
    e.dispatch=admission_setup();e.moves={}
    e.controller=SimpleNamespace(executions={},local_traffic=None)
    return e


def test_short_slow_route_loses_to_long_fast_route_for_parked_trucks():
    e=executor();slow=straight_plan(10,.5);fast=straight_plan(40,8)
    e.geometry=SimpleNamespace(plan=lambda move:slow if move.origin=='A' else fast)
    states={n:VehicleState(n,0,0,0,speed=0) for n in ('near','far')}
    priorities=e._berth_priorities(states)
    assert priorities['near'][0]>priorities['far'][0]
    e.dispatch.holders['L']='near';e.dispatch.promote_approaches(priorities)
    assert e.dispatch.holders['L']=='far'
    assert [m.vehicle_id for m in e.dispatch.tick(0)]==['far']


def test_measured_speed_changes_eta_without_waiting_for_cache_expiry():
    e=executor();plan=straight_plan(100,8)
    state=VehicleState('near',0,0,0,speed=0)
    stopped=e._admission_eta('near',plan,state)
    state.speed=8
    moving=e._admission_eta('near',plan,state)
    assert stopped>moving+3


def test_stationary_eta_does_not_count_elapsed_time_as_distance_travelled():
    e=executor();plan=straight_plan(100,8);state=VehicleState('near',0,0,0,speed=0)
    eta=e._admission_eta('near',plan,state)
    e.dispatch._time(.4)
    assert e._admission_eta('near',plan,state)==pytest.approx(eta)


def test_waiting_owner_stays_in_every_priority_refresh():
    e=executor();plan=straight_plan(100,8);e.geometry=SimpleNamespace(plan=lambda move:plan)
    states={n:VehicleState(n,0,0,0,speed=0) for n in ('near','far')}
    e.dispatch.holders['L']='far'
    for now in (0,.1,.6):
        e.dispatch._time(now)
        priorities=e._berth_priorities(states)
        assert set(priorities)=={'near','far'}
        e.dispatch.promote_approaches(priorities)


def test_final_approach_protection_begins_at_action_entry_not_six_metres_before():
    e=executor();model=straight_plan(20,8).model
    model.edges['dock']=dict(centerline=[[20,0],[40,0]],speed_limit=1,kind='service_access',maneuver='STRAIGHT',resource=None,capacity_alias='dock',reverse=False)
    model.aliases['dock']='dock';model.physical_paths['dock']=Polyline(((20,0),(40,0)))
    plan=RouteGeometry(model,['road','dock'],[],'B',['L'])
    ex=SimpleNamespace(plan=plan,progress=17.,checkpoint=0,finished=False)
    e.moves={'far':SimpleNamespace(destination='L',route={'edge_ids':[]})}
    e.controller.executions={'far':ex};e.geometry=SimpleNamespace(model=model)
    states={'far':VehicleState('far',17,0,0,speed=1)}
    assert not e._approach_priorities(states)['far'][1]
    ex.progress=20.1;states['far'].x=20.1
    assert e._approach_priorities(states)['far'][1]


def parking_exit_dispatch():
    d=admission_setup()
    # Separate the real bay bodies enough to avoid initial physical overlap.
    d.network.services={k:dict(v) for k,v in d.network.services.items()}
    d.network.services['L'].update(rear_xy_m=[0,0],heading_deg=0)
    d.network.edges={
        'reverse':dict(kind='service_access',centerline=[[14,0],[-12,0]],reverse=True),
        'forward':dict(kind='service_access',centerline=[[14,0],[30,0]],reverse=False)}
    d.holders['L']='far'
    return d


def test_parking_exit_avoids_physically_occupied_adjacent_service_bay():
    d=parking_exit_dispatch();d.positions['far']='L'
    assert not d._parking_approach_clear({'edge_ids':['reverse']},'near')
    assert d._parking_approach_clear({'edge_ids':['forward']},'near')


def test_remote_promise_does_not_count_as_a_body_but_committed_ingress_does():
    d=parking_exit_dispatch();route={'edge_ids':['reverse']}
    assert d._parking_approach_clear(route,'near')
    d.admission_priorities['far']=(1.,True)
    assert not d._parking_approach_clear(route,'near')


def test_parking_ingress_after_road_does_not_enter_an_occupied_dock_corridor():
    d=parking_exit_dispatch();d.positions['far']='L'
    d.network.edges['road']=dict(kind='road',centerline=[[40,10],[14,10]],reverse=False)
    assert not d._parking_approach_clear({'edge_ids':['road','reverse']},'near')
    assert d._parking_approach_clear({'edge_ids':['road','forward']},'near')


def test_parking_ingress_after_road_ignores_remote_promise_until_committed():
    d=parking_exit_dispatch()
    d.network.edges['road']=dict(kind='road',centerline=[[40,10],[14,10]],reverse=False)
    route={'edge_ids':['road','reverse']}
    assert d._parking_approach_clear(route,'near')
    d.admission_priorities['far']=(1.,True)
    assert not d._parking_approach_clear(route,'near')
