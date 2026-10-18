"""Unsafe physical access remains closed across routing and parking callers."""
import json
from pathlib import Path
import pytest
from logistics_park.routing.service_network import ServiceNetwork
ROOT=Path(__file__).resolve().parents[1]
NETWORK=ROOT/'config/topology/industrialarea_ue_internal_waiting_network.json'


@pytest.mark.parametrize('pending',[False,True])
@pytest.mark.parametrize('extra',[frozenset(),frozenset({'access_133'})])
def test_q33_closure_cannot_be_overridden_by_routing_options(pending,extra,tmp_path):
    # Even accidental republication of the bay cannot bypass its physical
    # access closure. Only this isolated test copy republishes the definition.
    raw=json.loads(NETWORK.read_text())
    raw['service_nodes']['Q33']=raw['temporarily_unavailable_service_nodes']['Q33']['definition']
    p=tmp_path/'republished.json';p.write_text(json.dumps(raw))
    network=ServiceNetwork(p,blocked_edges=extra)
    assert {'Q33_entry','Q33_exit'}|extra <= network.blocked_edges
    assert network.route('W1-loading','Q33',allow_pending_access=pending) is None
    assert network.route('Q33','W1-unloading',allow_pending_access=pending) is None
    assert network.route('W1-loading','W2-unloading',allow_pending_access=pending)
    assert 'Q33' not in {o['waiting_node'] for o in network.queue_options('W1-loading','W2-unloading',allow_pending_access=pending)}


def test_unknown_topology_closure_is_rejected(tmp_path):
    raw=json.loads(NETWORK.read_text());raw['disabled_edges']=['missing-access']
    p=tmp_path/'network.json';p.write_text(json.dumps(raw))
    with pytest.raises(ValueError,match='Unknown blocked service edge'):ServiceNetwork(p)


def test_fixed_demo_keeps_tasks_and_initial_fleet_outside_closed_bay():
    spec=json.loads((ROOT/'config/runtime/industrialarea_ue_full_verified_20261007.json').read_text())
    assert len(spec['tasks'])==32 and len(spec['positions'])==16
    assert 'Q33' in spec['efficiency_policy']['unavailable_waiting']
    assert {'Q33_entry','Q33_exit'} <= set(spec['blocked_candidate_edges'])
    assert 'Q33' not in spec['positions'].values()
    assert all('Q33' not in task for task in spec['tasks'])


def test_closed_q33_is_not_a_published_service():
    network=ServiceNetwork(NETWORK)
    assert 'Q33' not in network.services
    assert 'Q33' in network.raw['temporarily_unavailable_service_nodes']
    with pytest.raises(ValueError,match='distinct known service nodes'):
        network.route('W1-loading','Q33')
