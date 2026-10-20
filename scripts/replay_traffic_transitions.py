#!/usr/bin/env python3
"""Replay recorded unsafe transitions without starting UE or moving a vehicle."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from logistics_park.common.messages import VehicleState
from logistics_park.runtime.transport_routes import TransportMap, RouteGeometry
from logistics_park.runtime.transport_control import TransportController, envelope
from logistics_park.runtime.ue_transport import observed_boxes
from logistics_park.scheduling.local_traffic import LocalTraffic
from logistics_park.scheduling.conflict_timing import maneuver_groups


def recorded_frame(row):
    model = TransportMap(ROOT / 'config/topology/industrialarea_ue_internal_waiting_transport.json')
    plans, executions, states = {}, {}, {}
    for name, detail in row['execution_details'].items():
        edges = detail['route_edges']
        checkpoints = []
        for i, (a, b) in enumerate(zip(edges, edges[1:]), 1):
            if bool(model.edges[a].get('reverse')) != bool(model.edges[b].get('reverse')):
                checkpoints.append(dict(event='GEAR_CHANGE', after_edge_count=i,
                                        direction=-1 if model.edges[b].get('reverse') else 1))
        checkpoints.append(dict(event='ARRIVAL', after_edge_count=len(edges), leg=0,
                                site=detail['destination'], node=detail['destination']))
        plans[name] = RouteGeometry(model, edges, checkpoints, 'recorded', [detail['destination']])
        executions[name] = SimpleNamespace(plan=plans[name], progress=detail['progress_m'],
            checkpoint=detail['checkpoint_index'], finished=detail['finished'])
        states[name] = VehicleState(name, *row['positions'][name])
    local = LocalTraffic()
    local.compact_permissions = True
    local.proactive_admission = True
    local.action_groups = {n: maneuver_groups(p) for n, p in plans.items()}
    return plans, executions, states, local


def replay(fixture):
    row = fixture['round04']
    old, executions, _, _ = recorded_frame(row['before'])
    new, after, states, local = recorded_frame(row['after'])
    for name in executions:
        executions[name].progress = after[name].progress
    controller = TransportController.__new__(TransportController)
    controller.model = old['Car_5_0'].model
    controller.local_traffic = local
    controller.executions = executions
    controller.plans = old.copy()
    controller.states = states
    controller.boxes = {n: observed_boxes(dict(bodies=b)) for n, b in row['bodies'].items()}
    controller.capacity = SimpleNamespace(committed={})
    controller.events = []
    denied = False
    try:
        controller.reroute_stopped('Car_5_0', new['Car_5_0'], row['event']['time'], state=states['Car_5_0'])
    except ValueError:
        denied = True
    result4 = dict(unsafe_diversion_rejected=denied,
        original_route_retained=controller.plans['Car_5_0'] is old['Car_5_0'],
        decision=local.events[-1] if local.events else None)
    row = fixture['round07']
    plans, executions, states, local = recorded_frame(row['after'])
    states.update({n: VehicleState(n, *p) for n, p in row['release_positions'].items()})
    local.passages['Car_0_1', 20] = dict(plan=plans['Car_0_1'], end=plans['Car_0_1'].length,
                                       created_at=row['grant']['time'])
    local.prepare(states, executions, {}, envelope, row['event']['time'])
    result7 = dict(committed_permission_retained=('Car_0_1', 20) in local.passages,
        no_retry_penalty=('Car_0_1', 20) not in local.retry_after,
        decisions=[e for e in local.events if e['event'] == 'TRANSITION_REJECTED'])
    passed = (result4['unsafe_diversion_rejected'] and result4['original_route_retained']
              and result7['committed_permission_retained'] and result7['no_retry_penalty'])
    handoff=None
    if 'trial01_handoff' in fixture:
        row=fixture['trial01_handoff']
        old,executions,_,local=recorded_frame(row['before'])
        new,_,states,_=recorded_frame(row['after'])
        name='Car_3_0';executions[name].finished=True
        local.passages[name,row['old_start']]=dict(plan=old[name],end=row['old_end_m'],sweep=[])
        controller=TransportController.__new__(TransportController)
        controller.retain_finished_vehicles=True;controller.executions=executions
        controller.plans=old.copy();controller.shared_scheduler=None
        controller.local_traffic=local;controller.states=states;controller.boxes={}
        controller.events=[]
        controller.continue_route(name,new[name],row['event']['time'],state=states[name])
        handoff=dict(old_permission_retired=(name,row['old_start']) not in local.passages,
                     next_route_installed=controller.executions[name].plan is new[name],
                     stopped_body_preserved=controller.states[name] is states[name])
        passed=passed and all(handoff.values())
    stopped_before_line=None
    if 'trial02_stopped_before_line' in fixture:
        row=fixture['trial02_stopped_before_line']
        plans,executions,states,local=recorded_frame(row['frame'])
        name='Car_1_1';start=79
        local.passages[name,start]=dict(plan=plans[name],end=row['end_m'],created_at=row['grant']['time'])
        local.prepare(states,executions,{},envelope,row['event']['time'])
        stopped_before_line=dict(safely_released=(name,start) not in local.passages,
            holding_room_m=plans[name].edge_starts[start]-7.8-executions[name].progress)
        passed=passed and stopped_before_line['safely_released']
    return dict(status='PASS' if passed else 'FAIL',round04=result4,round07=result7,
                trial01_handoff=handoff,trial02_stopped_before_line=stopped_before_line,limits=fixture['limits'])



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', type=Path, default=ROOT / 'tests/fixtures/ue_transition_failures_20261020.json')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = replay(json.loads(args.fixture.read_text()))
    text = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    print(text, end='')
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
