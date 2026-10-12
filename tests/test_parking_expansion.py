"""Parking capacity and legal gate connections, without all-pairs UE tests."""
import json
import math
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from build_ue_parking_expansion import paths
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.runtime.transport_routes import TransportMap


def test_capacity_excludes_retired_slot_and_has_independent_bays():
    plan=json.loads((ROOT/'config/planning/ue_parking_expansion.json').read_text())
    assert len(plan['existing_waiting_nodes'])+sum(s['capacity'] for s in plan['sites'])==24
    assert 'Q3' not in plan['existing_waiting_nodes']
    assert len({tuple(s['xy_m']) for s in plan['sites']})==13
    assert all(s['entry_mode']==s['exit_mode']=='forward' for s in plan['sites'])


def test_north_left_turn_uses_left_lane_south_right_turn_uses_right_lane():
    plan=json.loads((ROOT/'config/planning/ue_parking_expansion.json').read_text())
    network=ServiceNetwork(ROOT/plan['source_network'])
    for s in plan['sites']:
        expected='gate_north_0_out' if s['region']=='north_outer' else 'gate_south_0_out_right'
        assert s['entry_gate']==expected
        assert math.dist(s['entry_rear_path'][0],network.nodes[expected]['position'])<1e-6
        assert math.dist(s['exit_rear_path'][-1],network.nodes[s['exit_gate']]['position'])<1e-6
        assert math.dist(s['entry_rear_path'][-1],s['exit_rear_path'][0])<1e-6


def test_parking_curves_obey_minimum_turn_radius():
    for north in (False,True):
        for path in paths(-134,north):
            for a,b,c in zip(path,path[1:],path[2:]):
                ab,bc,ac=math.dist(a,b),math.dist(b,c),math.dist(a,c)
                cross=abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))
                if cross>1e-8:
                    radius=ab*bc*ac/(2*cross)
                    assert radius>=3.9


def test_candidate_parking_has_bidirectional_reachability_but_cannot_bypass_validation():
    network=ServiceNetwork(ROOT/'config/topology/industrialarea_ue_parking24_network.json',blocked_edges={'access_133','access_134','access_167'})
    for q in (f'Q{i}' for i in range(13,26)):
        assert network.route('W3-loading',q,allow_pending_access=True)
        assert network.route(q,'W4-unloading',allow_pending_access=True)
        if network.services[q].get('validation')=='STATIC_PASS_UE_ENTRY_EXIT_PENDING':
            assert network.route('W3-loading',q) is None
            assert network.route(q,'W4-unloading') is None
    # Loading the real map validates every edge/movement including subdivided road joins.
    TransportMap(ROOT/'config/topology/industrialarea_ue_parking24_transport.json')


def test_transport_executor_can_compose_new_parking_entry_and_exit():
    from types import SimpleNamespace
    from logistics_park.runtime.task_execution import ServiceRouteGeometry
    network=ServiceNetwork(ROOT/'config/topology/industrialarea_ue_parking24_network.json',blocked_edges={'access_133','access_134','access_167'})
    # Exercise geometry composition separately from the physical approval gate.
    for e in network.edges.values():
        if e.get('site') in {f'Q{i}' for i in range(13,26)}:e['access_validation_required']=False
    model=TransportMap(ROOT/'config/topology/industrialarea_ue_parking24_transport.json')
    geometry=ServiceRouteGeometry(network,model)
    for q in ('Q19','Q25'):
        for origin,destination in (('W3-loading',q),(q,'W4-unloading')):
            route=network.route(origin,destination)
            plan=geometry.plan(SimpleNamespace(origin=origin,destination=destination,route=route))
            assert math.dist(plan.point(0),network.services[origin]['rear_xy_m'])<1e-6
            assert math.dist(plan.point(plan.length),network.services[destination]['rear_xy_m'])<1e-6
