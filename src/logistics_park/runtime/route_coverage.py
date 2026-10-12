"""Deduplicate physical routes without inventing end-to-end UE evidence."""
import hashlib
import json
import math


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def coverage_sets(topology, jobs):
    edges = {e['edge_id']: e for e in topology['edges']}
    channels = {key: [] for key in edges}
    for name, region in topology.get('shared_resources', {}).items():
        for key in region['edge_ids']:
            channels[key].append(name)
    # Exact geometry: even millimetre endpoint snaps remain distinct. Vehicle
    # direction, speed, curvature delay and resource semantics also matter.
    signatures = {}
    for key, edge in edges.items():
        value = {k: edge.get(k) for k in (
            'centerline', 'kind', 'maneuver', 'speed_limit',
            'maximum_path_error_m', 'capacity_alias', 'required_resources')}
        value.update(reverse=bool(edge.get('reverse')),
                     straight_until_m=edge.get('straight_until_m', 0),
                     channels=sorted(channels[key]))
        signatures[key] = digest(value)
    requirements = {}
    for job in jobs:
        path = [signatures[key] for key in job['edges']]
        tokens = {'EDGE:' + key for key in path}
        tokens.update('JOIN:' + digest(pair) for pair in zip(path, path[1:]))
        tokens.update('CHAIN3:' + digest(triple) for triple in zip(path, path[1:], path[2:]))
        tokens.add('DEPARTURE:' + digest([job['origin'], path[0]]))
        for cp in job['checkpoints']:
            index = cp['after_edge_count'] - 1
            if cp['event'] == 'GEAR_CHANGE':
                tokens.add('GEAR_CHANGE:' + digest([path[index], path[index + 1], cp['direction']]))
            elif cp['event'] == 'ARRIVAL':
                tokens.add('ARRIVAL:' + digest([job['destination'], path[index]]))
            else:
                raise ValueError('Unsupported scan checkpoint: ' + cp['event'])
        requirements[job['pair_id']] = tokens
    return requirements


def select_scan(topology, scenario):
    jobs = scenario['jobs']
    if len({j['pair_id'] for j in jobs}) != len(jobs):
        raise ValueError('Duplicate route identifiers')
    required = coverage_sets(topology, jobs)
    universe = set().union(*required.values())
    edges = {e['edge_id']: e for e in topology['edges']}
    distances = {j['pair_id']: sum(math.dist(a, b) for key in j['edges']
                 for a, b in zip(edges[key]['centerline'], edges[key]['centerline'][1:])) for j in jobs}
    # Weighted greedy set cover, followed by removal of redundant routes.
    costs = {j['pair_id']: 15 + sum(math.dist(a, b) / max(.1, edges[key]['speed_limit'])
             for key in j['edges'] for a, b in zip(edges[key]['centerline'], edges[key]['centerline'][1:]))
             for j in jobs}
    uncovered = set(universe)
    selected = []
    while uncovered:
        key = min(required, key=lambda key: (-len(required[key] & uncovered) / costs[key], key))
        if not required[key] & uncovered:
            raise ValueError('Uncoverable road requirements')
        selected.append(key)
        uncovered -= required[key]
    for key in list(reversed(selected)):
        others = [required[k] for k in selected if k != key]
        if set().union(*others) == universe:
            selected.remove(key)
    channels = {j['pair_id']: sorted(name for name, region in topology.get('shared_resources', {}).items()
                if set(j['edges']) & set(region['edge_ids'])) for j in jobs}
    # Separate routes with common critical areas. Runtime still checks actual
    # vehicle envelopes before staging; these waves are planning hints only.
    remaining = set(selected)
    waves = []
    while remaining:
        wave, occupied = [], set()
        for key in sorted(remaining, key=lambda key: (-len(channels[key]), key)):
            if not occupied.intersection(channels[key]):
                wave.append(key)
                occupied.update(channels[key])
        waves.append(wave)
        remaining.difference_update(wave)
    selected_distance = sum(distances[key] for key in selected)
    total_distance = sum(distances.values())
    return dict(status='PLANNED',scope='Distinct components used by the loading/unloading node-pair routes; waiting nodes excluded as endpoints',
                source_route_count=len(jobs), selected_route_count=len(selected),
                selected_pair_ids=selected, parallel_waves=waves,
                maximum_wave_size=max(map(len, waves), default=0),
                source_distance_m=total_distance, scan_distance_m=selected_distance,
                distance_reduction_fraction=1-selected_distance/total_distance,
                requirement_count=len(universe),
                requirement_counts={kind: sum(t.startswith(kind+':') for t in universe) for kind in
                    ('EDGE','JOIN','CHAIN3','DEPARTURE','ARRIVAL','GEAR_CHANGE')},
                requirements_by_pair={key: sorted(tokens) for key,tokens in required.items()},
                regions_by_pair={key: channels[key] for key in selected},
                ue_physics_validated=False, end_to_end_132_physically_validated=False,
                algorithm='Weighted greedy set cover; exact geometry, direction and resource identity; not a minimum-cover proof')


def coverage_result(manifest, results):
    expected=set(manifest['selected_pair_ids'])
    # Only stopped, completed physical runs contribute evidence. Failed routes
    # do not count as covering their untraversed suffixes.
    passed={r['pair_id'] for r in results if r['pair_id'] in expected and r['status']=='PASS'
            and r.get('ue_physics_validated') and r.get('body_obstacle_contacts')==0 and r.get('vehicle_collisions')==0}
    covered=set().union(*(set(manifest['requirements_by_pair'][key]) for key in passed))
    universe=set().union(*(set(v) for v in manifest['requirements_by_pair'].values()))
    missing=universe-covered
    pair_status={key: 'COMPONENTS_COVERED' if set(v)<=covered else 'INCOMPLETE'
                 for key,v in manifest['requirements_by_pair'].items()}
    return dict(coverage_status='COMPLETE' if not missing else 'INCOMPLETE',
                covered_requirement_count=len(covered), total_requirement_count=len(universe),
                missing_requirements=sorted(missing), components_by_pair=pair_status,
                end_to_end_132_physically_validated=False)
