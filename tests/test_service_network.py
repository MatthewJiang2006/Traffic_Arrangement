"""Protect road permissions, incomplete access and queue occupancy semantics."""
import hashlib
import json
from pathlib import Path

import pytest

from logistics_park.routing.service_network import ServiceNetwork


ROOT = Path(__file__).resolve().parents[1]
GRAPH = ROOT / 'config/topology/industrialarea_ue_service_network.json'


@pytest.fixture(scope='module')
def network():
    return ServiceNetwork(GRAPH)


def test_service_nodes_are_separate_and_retired_waiting_excluded(network):
    expected = {f'W{i}-{role}' for i in range(1, 7) for role in ('loading', 'unloading')}
    expected |= {f'Q{i}' for i in range(1, 13)}
    assert set(network.services) == expected
    assert all(n['capacity'] == 1 for n in network.services.values())
    assert all(n['entry_road_ports'] and n['exit_road_ports'] for n in network.services.values())


def test_road_splitting_preserves_geometry_and_every_legal_turn(network):
    base = json.loads((ROOT / 'config/topology/industrialarea_ue_transport.json').read_text())
    parts = network.raw['source_road_edge_parts']
    transitions = {(m['incoming_edge'], m['outgoing_edge']) for m in network.raw['movements']}
    permitted = set()
    for edge in base['edges']:
        children = [network.edges[k] for k in parts[edge['edge_id']]]
        assert children[0]['centerline'][0] == edge['centerline'][0]
        assert children[-1]['centerline'][-1] == edge['centerline'][-1]
        # Inserting ports may subdivide segments, never relocate the original line.
        from logistics_park.map.geometry import Polyline
        line = Polyline(tuple(map(tuple, edge['centerline'])))
        assert sum(c['length_m'] for c in children) == pytest.approx(line.length)
        assert all(line.project(*p).distance < 1e-7 for c in children for p in c['centerline'])
        permitted.update(zip(parts[edge['edge_id']], parts[edge['edge_id']][1:]))
    permitted.update((parts[m['incoming_edge']][-1], parts[m['outgoing_edge']][0]) for m in base['movements'])
    actual = {(a, b) for a, b in transitions
              if network.edges[a]['kind'] != 'service_access' and network.edges[b]['kind'] != 'service_access'}
    assert actual == permitted


def test_catalog_paths_are_directed_and_inputs_current(network):
    catalog = json.loads((ROOT / 'config/planning/ue_service_connectivity.json').read_text())
    assert catalog['graph_sha256'] == hashlib.sha256(GRAPH.read_bytes()).hexdigest()
    for name, digest in network.raw['source_sha256'].items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest
    assert len(catalog['pairs']) == 24 * 23
    assert sum(p['reachable_without_pending_access'] for p in catalog['pairs']) == catalog['strict_reachable_pair_count']
    for pair in catalog['pairs']:
        route = pair['best_route']['edge_ids']
        assert network.edges[route[0]]['from_node'] == pair['origin']
        assert network.edges[route[-1]]['to_node'] == pair['destination']
        assert all(b in network.successors[a] for a, b in zip(route, route[1:]))
        assert set(route) <= set(pair['legal_corridor_edge_ids'])
        assert pair['end_to_end_ue_validated'] is False
        strict = pair['route_without_pending_access']
        if strict:
            assert not strict['attachment_validation_required'] and not strict['access_validation_required']


def test_pending_reverse_connections_are_excluded_by_default(network, tmp_path):
    # Published local validation now opens the four original access gaps.
    for a, b in [('W5-loading', 'W3-loading'), ('W4-unloading', 'W5-loading')]:
        route = network.route(a, b)
        assert route and not route['access_validation_required']
    # A subsequently added unvalidated arrival must still fail closed.
    raw = json.loads(GRAPH.read_text())
    for edge in raw['edges']:
        if edge['to_node'] == 'W3-loading':edge['access_validation_required'] = True
    pending = tmp_path / 'pending.json'
    pending.write_text(json.dumps(raw))
    blocked = ServiceNetwork(pending)
    assert blocked.route('W5-loading', 'W3-loading') is None
    assert blocked.route('W5-loading', 'W3-loading', allow_pending_access=True)['access_validation_required']


def test_multiple_approaches_and_direct_staging_connection(network):
    assert network.services['W5-loading']['entry_modes'] == ['forward', 'reverse']
    assert len(network.services['W5-loading']['entry_road_ports']) > 1
    route = network.route('W5-loading', 'W6-unloading')
    assert route['distance_m'] < 100  # Avoid an unnecessary park-wide loop.
    assert not route['attachment_validation_required']
    assert route['gear_change_nodes']
    reverse_departure = network.route('Q12', 'W5-loading', allow_pending_access=True)
    assert reverse_departure['initial_direction'] == -1
    assert reverse_departure['gear_change_nodes'][0] == 'Q12'


def test_waiting_departure_and_occupied_destination_block_route(network):
    assert network.route('Q1', 'W5-loading') is not None
    assert network.route('Q1', 'W5-loading', occupied_nodes={'Q2'}, allow_pending_access=True) is None
    assert network.route('Q1', 'W5-loading', occupied_nodes={'W5-loading'}) is None
    assert network.route('Q1', 'W5-loading', occupied_nodes={'W6-unloading'}) is None
    queues = network.queue_options('W1-loading', 'W5-loading', occupied_nodes={'Q2'}, allow_pending_access=True)
    assert all(q['waiting_node'] != 'Q2' for q in queues)


def test_queue_cannot_fill_through_busy_destination_shared_access(network):
    queues = {q['waiting_node']: q for q in network.queue_options(
        'W1-loading', 'W5-loading', allow_pending_access=True)}
    assert queues['Q1']['capacity'] == 1
    assert queues['Q1']['departure_requires_empty'] == ['Q2']
    assert 'access:W5_W6_merge' in queues['Q2']['destination_resource_conflicts']
    assert not queues['Q2']['usable_while_destination_occupied']
    assert queues['Q10']['usable_while_destination_occupied']
    for q in queues.values():
        assert network.edges[q['approach']['edge_ids'][-1]]['to_node'] == q['waiting_node']
        assert network.edges[q['onward']['edge_ids'][0]]['from_node'] == q['waiting_node']


def test_congestion_detour_cannot_use_waiting_bays_as_transit():
    network = ServiceNetwork(ROOT / 'config/topology/industrialarea_ue_parking24_network.json',
                             blocked_edges={'access_133', 'access_134', 'access_167'})
    # Round 10: the Car_2_0 detour entered Q10 en route to Q11, then stopped
    # inside the lower junction when Q10 acquired a parked occupant.
    for occupied in (frozenset(), frozenset({'Q10'})):
        route = network.route('access_point_309', 'Q11',
                              incoming_edge='road_j_0_1_j_1_1__service_0',
                              occupied_nodes=occupied)
        assert route is not None
        assert all(network.services.get(network.edges[k]['to_node'], {}).get('kind') != 'waiting'
                   for k in route['edge_ids'][:-1])
    # Removing shortcuts must retain every warehouse transport pair.
    bays = [n for n, site in network.services.items() if site['kind'] in ('loading', 'unloading')]
    assert all(network.route(a, b) for a in bays for b in bays if a != b)


def test_declared_tandem_waiting_access_retains_empty_dependency(network):
    for a, b in [('W1-loading', 'Q1'), ('Q1', 'W5-loading')]:
        route = network.route(a, b)
        assert route and 'Q2' in route['must_be_empty_nodes']
        assert network.route(a, b, occupied_nodes={'Q2'}) is None
