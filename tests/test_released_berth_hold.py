"""A released destination promise must not retain permission to its endpoint."""
from types import SimpleNamespace
import pytest
from logistics_park.common.messages import VehicleState
from logistics_park.runtime.task_execution import ContinuousTaskExecutor
from logistics_park.scheduling.efficiency_policy import EfficiencyPolicy


@pytest.mark.parametrize('cached,expected',[(70.,12.),(8.,8.)])
def test_lost_berth_keeps_current_or_earlier_hold_when_no_new_landing_exists(monkeypatch,cached,expected):
    name='incoming';plan=SimpleNamespace(edges=('road','dock'),edge_starts=(0.,20.),length=70.)
    execution=SimpleNamespace(plan=plan,finished=False,progress=.08)
    move=SimpleNamespace(vehicle_id=name,command_id=1,destination='dock')
    dispatch=SimpleNamespace(efficiency_policy=EfficiencyPolicy(proactive_admission=True),
        _time=lambda now:None,departures={1},traffic_routing=None,offroad_busy_wait=False,
        tick=lambda now:[],promote_approaches=lambda priorities:None,holders={},moves={name:move})
    local=SimpleNamespace(eta_speed_scales={},last_targets={},timing=SimpleNamespace(windows={}))
    controller=SimpleNamespace(retain_finished_vehicles=True,local_traffic=local,
        executions={name:execution},tick=lambda states,now,**kw:{name:(0.,0.)})
    geometry=SimpleNamespace(model=SimpleNamespace(edges={'road':{'kind':'road'},'dock':{'kind':'service_access'}}))
    executor=ContinuousTaskExecutor.__new__(ContinuousTaskExecutor)
    executor.dispatch=dispatch;executor.controller=controller;executor.geometry=geometry
    executor.moves={name:move};executor.eta_log_at=0.;executor.sources={}
    executor.travel_time=SimpleNamespace(scale={},observe=lambda *args:None)
    executor.safe_landing_states={(name,plan.edges):cached}
    executor._berth_priorities=lambda states:{};executor._queue_gate=lambda *args:12.
    monkeypatch.setattr('logistics_park.runtime.queue_clearance.refresh_requirements',lambda *args:None)
    monkeypatch.setattr('logistics_park.runtime.queue_clearance.clear_stopped_queues',lambda *args:None)
    monkeypatch.setattr('logistics_park.runtime.conflict_detour.try_conflict_detours',lambda *args:None)
    monkeypatch.setattr('logistics_park.runtime.safe_landing.safe_landing',lambda *args,**kw:None)
    monkeypatch.setattr('logistics_park.scheduling.wait_dependencies.wait_snapshot',lambda *args:{'dependencies':[]})
    monkeypatch.setattr('logistics_park.runtime.wait_cycle_recovery.recover_wait_cycles',lambda *args:None)
    executor.tick({name:VehicleState(name,0,0,0)},0.)
    assert controller.destination_holds[name]==expected
    assert not dispatch.holders


@pytest.mark.parametrize('progress,permit,protected',[(.08,True,True),(0.,True,False),(.08,False,False)])
def test_recorded_q2_to_w6_action_retains_berth_after_entering_its_admission_boundary(progress,permit,protected):
    import json
    from pathlib import Path
    from logistics_park.routing.service_network import ServiceNetwork
    from logistics_park.runtime.transport_routes import TransportMap
    from logistics_park.runtime.task_execution import ServiceRouteGeometry
    root=Path(__file__).resolve().parents[1]
    fixture=json.loads((root/'tests/fixtures/ue_20261006_released_berth_hold.json').read_text())
    network=ServiceNetwork(root/'config/topology/industrialarea_ue_internal_waiting_network.json',blocked_edges=['access_133','access_134','access_167'])
    geometry=ServiceRouteGeometry(network,TransportMap(root/'config/topology/industrialarea_ue_internal_waiting_transport.json'))
    plan=geometry.plan(SimpleNamespace(**{k:fixture[k] for k in ('origin','destination','route')}))
    e=ContinuousTaskExecutor.__new__(ContinuousTaskExecutor)
    e.geometry=geometry;e.moves={'incoming':SimpleNamespace(destination=fixture['destination'],route=fixture['route'])}
    e.dispatch=SimpleNamespace(efficiency_policy=EfficiencyPolicy(proactive_admission=True),offroad_busy_wait=False,
        network=network,holders={fixture['destination']:'incoming'},now=0.,approach_intents={})
    ex=SimpleNamespace(plan=plan,progress=progress,finished=False,checkpoint=0)
    local=SimpleNamespace(passages={('incoming',0):dict(plan=plan,end=43.92)} if permit else {})
    e.controller=SimpleNamespace(executions={'incoming':ex},local_traffic=local,destination_holds={})
    state=VehicleState('incoming',*plan.point(progress),plan.heading(progress))
    assert e._berth_gate(plan)==0.
    assert e._approach_priorities({'incoming':state})['incoming'][1] is protected
