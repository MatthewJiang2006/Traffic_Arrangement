"""The last actual UE task cannot depart toward an unreachable queue pose."""
import json
from pathlib import Path
from types import SimpleNamespace

from logistics_park.common.messages import VehicleState
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.runtime.safe_landing import safe_landing
from logistics_park.runtime.task_execution import ContinuousTaskExecutor, ServiceRouteGeometry
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.scheduling.efficiency_policy import EfficiencyPolicy


def test_q4_to_w3_requires_dock_admission_before_leaving_source():
    root=Path(__file__).resolve().parents[1]
    spec=json.loads((root/'config/runtime/industrialarea_ue_full_repeat_20261007.json').read_text())
    network=ServiceNetwork(root/spec['network'],blocked_edges=spec['blocked_candidate_edges'])
    geometry=ServiceRouteGeometry(network,TransportMap(root/spec['map']))
    route=network.route('Q4','W3-loading')
    plan=geometry.plan(SimpleNamespace(origin='Q4',destination='W3-loading',route=route))
    executor=ContinuousTaskExecutor.__new__(ContinuousTaskExecutor)
    executor.geometry=geometry
    executor.controller=SimpleNamespace(local_traffic=None)
    executor.dispatch=SimpleNamespace(network=network,holders={},now=0.,
        efficiency_policy=EfficiencyPolicy(proactive_admission=True),_event=lambda *a,**k:None)
    executor.latest_states={'truck':VehicleState('truck',*plan.point(0),plan.heading(0))}
    executor.landing_rejections={}
    # The old 74.927 m line is inside a protected dock exit. There is no
    # earlier safe stopping pose on this actual access/turn/uturn route.
    assert safe_landing(executor,'truck',plan,74.92661771781708,{},
                        target='W3-loading',traffic=False) is None
    assert executor._berth_gate(plan)==0.
    assert not executor._departure_guard('truck','Q4','W3-loading',route,False)
    assert not getattr(executor,'safe_landing_states',{})
    # Once dispatch grants the free dock, departure promises its actual goal
    # rather than parking in its outbound corridor.
    executor.dispatch.holders['W3-loading']='truck'
    assert executor._departure_guard('truck','Q4','W3-loading',route,False)
    assert executor.safe_landing_states['truck',plan.edges]==plan.length
    # A parked body at the destination must still reject the departure.
    executor.latest_states['other']=VehicleState('other',*plan.point(plan.length),plan.arrival_heading)
    assert not executor._departure_guard('truck','Q4','W3-loading',route,False)
