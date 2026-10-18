"""A speculative outlet must not claim a different truck's private berth."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from logistics_park.common.messages import VehicleState
from logistics_park.runtime.safe_landing import (
    berth_exit_blockers, immediate_departure_sweep, occupied_exit_sweeps,
    selected_exit_plan, service_exit_sweeps,
)
from logistics_park.runtime.transport_control import Execution, envelope
from logistics_park.scheduling.local_traffic import sweeps_intersect
from test_maneuver_admission import w6_exit_hold_fixture

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))


def inbound(node='W5-loading',target='W3-unloading'):
    e,_,_=w6_exit_hold_fixture();d=e.dispatch
    d.moves={'rear':NS(origin='staging',destination=node)}
    d.assignments={'rear':NS(work=NS(task_id='pending-load',loading_node=node,unloading_node=target))}
    d.holders={node:'rear'};d.admission_priorities={'rear':(20.,True)}
    d._route=lambda a,b,*args,**kwargs:d.network.route(a,b)
    d.now=0.;d._event=lambda *args,**kwargs:None
    e.controller.executions={};e.latest_states={}
    return e


def goal(e,node):
    s=e.dispatch.network.services[node]
    return envelope(VehicleState('goal',*s['rear_xy_m'],math.radians(s['heading_deg'])))


@pytest.mark.parametrize('node',['W6-loading','W6-unloading'])
def test_pending_w5_outlets_leave_unrelated_w6_berth_available(node):
    e=inbound();body=goal(e,node)
    selected=selected_exit_plan(e,'W5-loading','rear')
    assert not sweeps_intersect([body],immediate_departure_sweep(e,'rear',selected),.35)
    assert not sweeps_intersect([body],occupied_exit_sweeps(e,'W5-loading','rear'),.35)
    entry=e.dispatch.network.services['W5-loading']['entry_road_ports'][0]
    x,y=entry['road_reference_rear_xy_m']
    e.latest_states={'rear':VehicleState('rear',x,y-20.,math.pi/2)}
    assert not berth_exit_blockers(e,node,'front')


def test_selected_short_departure_keeps_its_actual_destination_protected():
    e=inbound(target='W6-unloading');body=goal(e,'W6-unloading')
    plan=selected_exit_plan(e,'W5-loading','rear')
    assert sweeps_intersect([body],immediate_departure_sweep(e,'rear',plan),.35)
    assert sweeps_intersect([body],occupied_exit_sweeps(e,'W5-loading','rear'),.35)
    e.dispatch.moves['rear']=NS(origin='W5-loading')
    e.controller.executions['rear']=Execution(plan)
    assert sweeps_intersect([body],occupied_exit_sweeps(e,'W5-loading','rear'),.35)


@pytest.mark.parametrize('assigned',[True,False])
def test_w3_still_protects_its_road_outlet_for_alternative_departures(assigned):
    e=inbound('W3-loading','W2-unloading')
    body=envelope(VehicleState('queue',-90.3,115.08,-math.pi/2))
    if not assigned:e.dispatch.assignments={};e.dispatch.moves={}
    assert sweeps_intersect([body],occupied_exit_sweeps(e,'W3-loading','rear'),.35)


def test_hypothetical_outlet_keeps_the_road_before_the_neighboring_arrival():
    e=inbound();r=e.dispatch.network.route('W5-loading','W6-loading')
    p=e.geometry.plan(NS(origin='W5-loading',destination='W6-loading',route=r))
    body=envelope(VehicleState('queue',*p.point(36.),p.heading(36.)))
    assert sweeps_intersect([body],service_exit_sweeps(e,'W5-loading',full_bundle=True),.35)


def same_target_sources(other_source):
    from simulate_ue_continuous_tasks import setup
    from logistics_park.scheduling.task_dispatch import TransportWork
    e,states=setup(positions={'first':'W6-unloading','following':other_source},tasks=[])
    e.dispatch.holders['W6-loading']='first'
    for name in states:
        e.dispatch.assignments[name]=NS(work=TransportWork('next-'+name,'W6-loading','W1-unloading'),
                                       state='READY',operation='LOADING')
    e.latest_states=states
    e.controller.boxes={n:[envelope(s)] for n,s in states.items()}
    route=e.dispatch.network.route('W6-unloading','W6-loading')
    return e,states,route


@pytest.mark.parametrize('other_source',['W5-unloading','W5-loading'])
def test_following_sources_future_arrival_does_not_block_the_owned_berth(other_source):
    e,states,route=same_target_sources(other_source)
    assert e._departure_guard('first','W6-unloading','W6-loading',route,False)
    assert e.dispatch.holders['W6-loading']=='first'


def test_actual_body_at_the_owned_arrival_still_blocks_departure():
    e,states,route=same_target_sources('W5-unloading')
    site=e.dispatch.network.services['W6-loading']
    states['following']=VehicleState('following',*site['rear_xy_m'],math.radians(site['heading_deg']))
    e.controller.boxes['following']=[envelope(states['following'])]
    assert not e._departure_guard('first','W6-unloading','W6-loading',route,False)
