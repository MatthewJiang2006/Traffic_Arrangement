import json
from pathlib import Path
from types import SimpleNamespace

from logistics_park.common.messages import VehicleState
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.runtime.fleet_progress import LocalProgressWatch,local_progress
from logistics_park.runtime.task_execution import ServiceRouteGeometry
from logistics_park.runtime.transport_control import Execution
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.scheduling.wait_dependencies import wait_snapshot


def recorded_wait():
    root=Path(__file__).resolve().parents[1]
    frame=json.loads((root/'tests/fixtures/ue_20261005_predicted_wait_cycle.json').read_text())
    spec=json.loads((root/'config/runtime/industrialarea_ue_full_acceptance_20261005_230447.json').read_text())
    network=ServiceNetwork(root/spec['network'],blocked_edges=spec['blocked_candidate_edges'])
    geometry=ServiceRouteGeometry(network,TransportMap(root/spec['map']))
    executions={}
    for name,detail in frame['execution_details'].items():
        keys=list(dict.fromkeys(e.rsplit('__test',1)[0] for e in detail['route_edges']))
        plan=geometry.plan(SimpleNamespace(origin=network.edges[keys[0]]['from_node'],
            destination=detail['destination'],route=dict(edge_ids=keys)))
        executions[name]=Execution(plan,progress=detail['progress_m'],checkpoint=detail['checkpoint_index'])
    dispatch=SimpleNamespace(assignments={},moves={n:SimpleNamespace(destination=d['destination'])
        for n,d in frame['execution_details'].items()},approach_intents={},
        _blockers=lambda target,vehicle:set(frame['berth_blockers'][target]),efficiency_policy=True)
    local=SimpleNamespace(paused_by={},passage_denials=frame['passage_denials'],last_denials={})
    controller=SimpleNamespace(executions=executions,control_denials=frame['control_denials'],
        destination_holds=frame['destination_holds'],local_traffic=local,shared_scheduler=None)
    states={n:VehicleState(n,*pose) for n,pose in frame['positions'].items()}
    return SimpleNamespace(dispatch=dispatch,controller=controller),states


def test_recorded_ue_wait_is_for_moving_passage_owner_not_a_two_truck_cycle():
    executor,states=recorded_wait()
    snapshot=wait_snapshot(executor,states)
    assert snapshot['cycles']==[]
    assert any(e['vehicle']=='Car_0_0' and e['blocked_by']=='Car_1_0'
        and e['reason']=='entered_special_conflict' for e in snapshot['dependencies'])
    assert any(e['vehicle']=='Car_0_1' and e['blocked_by']=='Car_1_0'
        and e['reason']=='reserved_maneuver_clearance' for e in snapshot['dependencies'])
    assert not any(e['vehicle']=='Car_0_0' and e['blocked_by']=='Car_0_1'
        or e['vehicle']=='Car_0_1' and e['blocked_by']=='Car_0_0' for e in snapshot['active_dependencies'])
    # Admission still sees the future body and berth blockers: its safety
    # policy must not lose predictive evidence just to correct the monitor.
    assert any(e['vehicle']=='Car_0_0' and e['blocked_by']=='Car_0_1'
        for e in snapshot['dependencies'])
    assert any(e['vehicle']=='Car_0_1' and e['blocked_by']=='Car_0_0'
        for e in snapshot['dependencies'])
    watch=LocalProgressWatch()
    local_progress(executor,states,0,watch)
    states['Car_1_0'].x+=1
    result,_=local_progress(executor,states,90,watch)
    assert not result['stalled']
    # If the actual passage owner stops too, the unchanged waiting timeout fires.
    result,_=local_progress(executor,states,180,watch)
    assert result['stalled']


def test_reached_body_guard_and_berth_hold_still_form_a_real_deadlock():
    executor,states=recorded_wait()
    c=executor.controller
    c.control_denials['Car_0_0']=[dict(blocked_by='Car_0_1',reason='stationary_swept_body',distance_m=0.)]
    c.control_denials['Car_0_1']=[]
    c.local_traffic.passage_denials={}
    c.destination_holds['Car_0_1']=c.executions['Car_0_1'].progress+.25
    snapshot=wait_snapshot(executor,states)
    assert snapshot['cycles']==[['Car_0_0','Car_0_1']]
    watch=LocalProgressWatch()
    local_progress(executor,states,0,watch)
    states['Car_1_0'].x+=1
    result,_=local_progress(executor,states,20,watch)
    assert result['stalled'] and result['cycles']==snapshot['cycles']


def test_future_passage_denial_does_not_become_a_current_wait():
    executor,states=recorded_wait()
    c=executor.controller
    c.control_denials={};c.destination_holds={}
    ex=c.executions['Car_0_0']
    start=next(i for i,s in enumerate(ex.plan.edge_starts) if s-ex.progress>30)
    c.local_traffic.passage_denials={'Car_0_0':[dict(blocked_by='Car_0_1',
        reason='entered_special_conflict',start=start)]}
    assert not any(e['reason']=='entered_special_conflict'
        for e in wait_snapshot(executor,states)['active_dependencies'])
