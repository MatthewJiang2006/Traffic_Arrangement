import json
from pathlib import Path
import pytest
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.task_execution import ValidatedServiceRoutes
from logistics_park.scheduling.task_dispatch import DispatchMove

ROOT=Path(__file__).resolve().parents[1]

def inputs():
    network=ServiceNetwork(ROOT/'config/topology/industrialarea_ue_service_network.json')
    spec=json.loads((ROOT/'config/runtime/industrialarea_ue_node_pair_tests.json').read_text())
    return network,TransportMap(ROOT/spec['map']),spec['jobs']


def test_bindings_keep_actual_route_and_gear_changes_and_reject_detour():
    network,model,jobs=inputs();bridge=ValidatedServiceRoutes(network,model,jobs)
    a,b='W1-loading','W3-unloading';route=network.route(a,b)
    move=DispatchMove(1,'t','car','visit',a,b,route,False)
    plan=bridge.plan(move)
    assert plan.origin==a and plan.stops==[b]
    for x,y in zip(plan.edges,plan.edges[1:]):
        assert model.edges[x]['to_node']==model.edges[y]['from_node']
        assert model.edges[x]['centerline'][-1]==pytest.approx(model.edges[y]['centerline'][0],abs=1e-8)
    route['edge_ids']=route['edge_ids'][1:]
    with pytest.raises(ValueError,match='no validated UE geometry'):bridge.plan(move)


def test_geometry_change_rejects_stale_manifest():
    network,model,jobs=inputs()
    job=next(j for j in jobs if j['origin']=='W1-loading' and j['destination']=='W3-unloading')
    model.edges[job['edges'][0]]['centerline'][0][0]+=1
    with pytest.raises(ValueError,match='Stale'):ValidatedServiceRoutes(network,model,[job])
