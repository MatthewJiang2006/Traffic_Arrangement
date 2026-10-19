#!/usr/bin/env python3
"""Classify a stalled wait cycle into physical jams and promise-only edges.

Reads a completed run directory (``physical_acceptance.json`` plus the last
``physical_control_diagnostics.jsonl`` tick) and answers one question: at the
moment the cycle detector fired, were the members actually blocking each
other's bodies, or only holding permits and reservations?

Usage:
    python scripts/analyze_wait_cycle.py <run_dir> [<run_dir> ...]
    python scripts/analyze_wait_cycle.py <run_dir> --json out.json

A "physical" edge means the blocker's body is at the blocked truck's stopping
point (the leader is within one braking gap of the nose). A "promise" edge
means the blocker only owns a lease, a queue slot or a reserved footprint: the
two bodies may be tens of metres apart, so no amount of reversing frees it.
Reversing a truck helps only when the cycle is cut by physical edges.
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from logistics_park.common.messages import VehicleState  # noqa: E402
from logistics_park.runtime.transport_control import envelope  # noqa: E402
from logistics_park.safety.collision import separating_gap  # noqa: E402
from logistics_park.scheduling.wait_dependencies import strongly_connected_cycles  # noqa: E402

# Reasons emitted by wait_snapshot(): the blocker's body is in the way.
PHYSICAL_REASONS = {
    'physical_leader', 'stationary_swept_body', 'passage_occupied_body',
    'predicted_body', 'occupied_landing_pose', 'occupied_source_exit',
    'occupied_berth_pose', 'braking_body', 'leader_stopped',
}
# Reasons that survive a reversing move: the blocker owns a permit, slot or
# reserved future footprint, not a piece of pavement.
PROMISE_REASONS = {
    'destination_queue_intent', 'approach_berth', 'passage_capacity',
    'occupied_exit_landing', 'route_clearance', 'destination_or_access',
    'reserved_landing_pose', 'paused_for_clear_passage', 'source_clearance',
    'departure_queue_intent', 'berth_reservation',
}
# A body gap at or below this is a direct block; beyond it the edge is only a
# declared stopping dependency or a queue leader further up the line.
JAM_GAP_M = 2.5
SAME_LANE_ANGLE_RAD = math.radians(30.)
SAME_LANE_LATERAL_M = 3.0
AT_RESOURCE_RADIUS_M = 6.0


def read_last_tick(path):
    last = None
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if line:
                last = json.loads(line)
    return last or {}


def read_nodes(path):
    data = json.loads(Path(path).read_text())
    nodes = {}
    raw = data.get('service_nodes', data.get('nodes', []))
    for node in (raw.values() if isinstance(raw, dict) else raw):
        name = node.get('node_id') or node.get('id')
        if name:
            nodes[name] = node
    return nodes


def pose_of(tick, name):
    row = tick.get('positions', {}).get(name)
    if row is None:
        return None
    return VehicleState(name, *row)


def same_lane(a, b):
    """True when b sits on a's lane, ahead or behind within one corridor."""
    if a is None or b is None:
        return False
    delta = b.heading - a.heading
    delta = abs((delta + math.pi) % (2 * math.pi) - math.pi)
    if delta > SAME_LANE_ANGLE_RAD:
        return False
    dx, dy = b.x - a.x, b.y - a.y
    across = abs(-dx * math.sin(a.heading) + dy * math.cos(a.heading))
    return across <= SAME_LANE_LATERAL_M


def body_gap(a, b):
    if a is None or b is None:
        return None
    return separating_gap(envelope(a), envelope(b), 0.)


def at_berth(pose, node, radius=6.0):
    if pose is None or not node:
        return False
    rear = node.get('rear_xy_m') or node.get('center_xy_m')
    if not rear:
        return False
    return math.dist((pose.x, pose.y), tuple(rear)) <= radius


def blocker_at_resource(pose, resources, nodes):
    """True when the blocker's body already sits on one of its resources."""
    if pose is None:
        return None
    seen = False
    for resource in resources:
        node = nodes.get(str(resource))
        if not node:
            continue
        seen = True
        if at_berth(pose, node, AT_RESOURCE_RADIUS_M):
            return True
    return False if seen else None


def classify(reason):
    if reason in PHYSICAL_REASONS:
        return 'physical'
    if reason in PROMISE_REASONS:
        return 'promise'
    return 'other'


def analyze_run(run_dir):
    run_dir = Path(run_dir)
    acceptance = json.loads((run_dir / 'physical_acceptance.json').read_text())
    dependencies = acceptance.get('wait_dependencies', {})
    tick = read_last_tick(run_dir / 'physical_control_diagnostics.jsonl')
    nodes = read_nodes(ROOT / 'config/topology/industrialarea_ue_internal_waiting_network.json')

    assignments = acceptance.get('task_assignments', {})
    intents = acceptance.get('berth_queue_intents', {})
    positions_by_node = {}
    for node, vehicle in intents.items():
        positions_by_node[node] = vehicle

    report = {
        'run_dir': str(run_dir),
        'stall_t_s': tick.get('t_s'),
        'elapsed_s': acceptance.get('elapsed_s'),
        'completed_tasks': acceptance.get('completed_tasks'),
        'cycles': [],
        'hold_and_wait': [],
    }

    states = {name: pose_of(tick, name) for name in tick.get('positions', {})}
    for members in dependencies.get('cycles', []):
        members = list(members)
        edges = []
        for row in dependencies.get('dependencies', []):
            if row['vehicle'] in members and row['blocked_by'] in members:
                a = states.get(row['vehicle'])
                b = states.get(row['blocked_by'])
                gap = body_gap(a, b)
                kind = classify(row['reason'])
                edge = dict(
                    vehicle=row['vehicle'],
                    blocked_by=row['blocked_by'],
                    reason=row['reason'],
                    resource=row['resource'],
                    kind=kind,
                    body_gap_m=None if gap is None else round(gap, 3),
                    same_lane=same_lane(a, b),
                )
                if kind == 'physical':
                    edge['confirmed_jam'] = bool(gap is not None and gap <= JAM_GAP_M)
                    edge['same_lane'] = same_lane(a, b)
                edges.append(edge)

        physical = [e for e in edges if e['kind'] == 'physical']
        confirmed = [e for e in physical if e.get('confirmed_jam')]
        promises = [e for e in edges if e['kind'] == 'promise']

        # Group repeated wait-for edges between the same pair. A promise edge
        # only cuts the loop if no body edge backs the same pair.
        pairs = {}
        for edge in edges:
            key = (edge['vehicle'], edge['blocked_by'])
            row = pairs.setdefault(key, dict(vehicle=key[0], blocked_by=key[1],
                                             reasons=[], resources=[], kinds=set(),
                                             direct_block=False, body_gap_m=edge['body_gap_m'],
                                             same_lane=edge['same_lane']))
            row['reasons'].append(edge['reason'])
            row['resources'].append(edge['resource'])
            row['kinds'].add(edge['kind'])
            if edge['kind'] == 'physical' and edge.get('confirmed_jam'):
                row['direct_block'] = True
        for row in pairs.values():
            row['pair_kind'] = 'physical' if row['direct_block'] else (
                'physical_queue' if 'physical' in row['kinds'] else 'promise')
            row['physically_coupled'] = bool(row['same_lane'] and row['body_gap_m'] is not None
                                             and row['body_gap_m'] <= JAM_GAP_M)
            row['reasons'] = sorted(set(row['reasons']))
            row['resources'] = sorted(set(map(str, row['resources'])))
            row['kinds'] = sorted(row['kinds'])

        # Cut analysis: remove one pair at a time and see if the members stop
        # being mutually reachable. A promise-only cut can be resolved by
        # withdrawing a lease; a physical cut needs a body to move.
        cut_edges = []
        all_pairs = [dict(vehicle=v, blocked_by=b) for v, b in pairs]
        for key, row in pairs.items():
            remaining = [dict(vehicle=p['vehicle'], blocked_by=p['blocked_by'])
                         for p in all_pairs if (p['vehicle'], p['blocked_by']) != key]
            if not strongly_connected_cycles(remaining):
                entry = dict(row)
                entry['cuttable'] = True
                entry['blocker_at_resource'] = blocker_at_resource(
                    states.get(key[1]), row['resources'], nodes)
                cut_edges.append(entry)
        cut_edges.sort(key=lambda r: (r['pair_kind'] != 'promise', r['vehicle']))

        member_state = {}
        for name in members:
            assignment = assignments.get(name, {})
            pose = states.get(name)
            target = assignment.get('target')
            source = assignment.get('source')
            node = nodes.get(target, {})
            member_state[name] = dict(
                task=assignment.get('task_id'),
                operation=assignment.get('operation'),
                state=assignment.get('state'),
                source=source,
                target=target,
                target_kind=node.get('kind'),
                distance_to_target_m=None if (pose is None or not node.get('rear_xy_m'))
                else round(math.dist((pose.x, pose.y), tuple(node['rear_xy_m'])), 2),
                inside_source=at_berth(pose, nodes.get(source, {})),
                speed=None if pose is None else round(pose.speed, 2),
            )

        cycle = dict(
            members=members,
            edges=edges,
            pairs=sorted(pairs.values(), key=lambda r: (r['vehicle'], r['blocked_by'])),
            cut_edges=cut_edges,
            physical_edges=len(physical),
            confirmed_jam_edges=len(confirmed),
            promise_edges=len(promises),
            member_state=member_state,
        )
        report['cycles'].append(cycle)

    # Hold-and-wait: a truck that already owns a berth slot while every one of
    # its blocks is a permit it cannot use is the classic cycle ingredient.
    holds = {v: n for n, v in intents.items()}
    for name, held_node in holds.items():
        rows = [r for r in dependencies.get('dependencies', []) if r['vehicle'] == name]
        if not rows:
            continue
        kinds = {classify(r['reason']) for r in rows}
        if kinds == {'promise'}:
            report['hold_and_wait'].append(dict(
                vehicle=name,
                holds=held_node,
                blocked_by=sorted({r['blocked_by'] for r in rows}),
                reasons=sorted({r['reason'] for r in rows}),
            ))

    report['verdict'] = verdict(report)
    return report


def verdict(report):
    if not report['cycles']:
        return 'no cycle in snapshot'
    for cycle in report['cycles']:
        cuts = cycle['cut_edges']
        if not cuts:
            continue
        promise_cuts = [c for c in cuts if c['pair_kind'] == 'promise'
                        and c.get('blocker_at_resource') is False]
        if promise_cuts:
            return ('promise-cuttable: the loop closes on reservations whose owner '
                    'has not entered them; withdrawing that promise ends the cycle '
                    'without moving any body (reversing is unnecessary)')
        if any(c['pair_kind'] == 'promise' for c in cuts):
            return ('promise-cuttable-but-owner-present: a permit closes the loop, '
                    'but its holder already sits on the resource')
        if any(c['pair_kind'] == 'physical' for c in cuts):
            return 'body-cuttable: a direct body block closes the loop; reversing can cut it'
        return 'queue-cuttable: a same-lane queue closes the loop; leader must advance'
    return 'undetermined'


def describe(report):
    lines = []
    lines.append(f"run: {report['run_dir']}")
    lines.append(f"stall at t={report['stall_t_s']} s, completed "
                 f"{report['completed_tasks']} tasks")
    for cycle in report['cycles']:
        lines.append('')
        lines.append('cycle ' + ' / '.join(cycle['members']))
        lines.append(f"  {cycle['physical_edges']} physical edges "
                     f"({cycle['confirmed_jam_edges']} direct blocks), "
                     f"{cycle['promise_edges']} promise edges")
        lines.append('  wait-for pairs')
        for pair in cycle['pairs']:
            lines.append(f"    {pair['vehicle']:>7} -> {pair['blocked_by']:<7} "
                         f"{pair['pair_kind']:<14} gap={pair['body_gap_m']:<8} "
                         f"coupled={pair['physically_coupled']} "
                         f"{','.join(pair['reasons'])}")
        lines.append('  cut edges (removing one breaks the loop)')
        for cut in cycle['cut_edges']:
            lines.append(f"    {cut['vehicle']:>7} -> {cut['blocked_by']:<7} "
                         f"{cut['pair_kind']:<14} at_resource={cut['blocker_at_resource']} "
                         f"{cut['reasons']}")
        lines.append('  member state')
        for name, state in cycle['member_state'].items():
            lines.append(f"    {name:>7} {str(state['state']):<7} {str(state['operation']):<9} "
                         f"-> {str(state['target']):<14} dist={state['distance_to_target_m']} "
                         f"inside_source={state['inside_source']}")
    if report['hold_and_wait']:
        lines.append('')
        lines.append('hold-and-wait trucks (own a slot, blocked only by promises)')
        for row in report['hold_and_wait']:
            lines.append(f"  {row['vehicle']} holds {row['holds']}, waits on "
                         f"{row['blocked_by']} via {row['reasons']}")
    lines.append('')
    lines.append('verdict: ' + report['verdict'])
    return '\n'.join(lines)


def main(argv):
    args = []
    out_path = None
    rest = list(argv[1:])
    while rest:
        item = rest.pop(0)
        if item == '--json':
            if not rest:
                raise SystemExit('--json needs an output path')
            out_path = Path(rest.pop(0))
        elif item.startswith('--'):
            continue
        else:
            args.append(item)
    if not args:
        raise SystemExit(__doc__)
    reports = [analyze_run(path) for path in args]
    text = '\n\n'.join(describe(report) for report in reports)
    print(text)
    if out_path:
        out_path.write_text(json.dumps(reports, ensure_ascii=False, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
