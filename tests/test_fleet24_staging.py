"""Road staging must preserve checked geometry and physical separation."""
import json,math
from pathlib import Path
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.task_execution import ServiceRouteGeometry
from logistics_park.runtime.transport_control import envelope,zone_box
from logistics_park.common.messages import VehicleState
from logistics_park.safety.collision import overlaps
from logistics_park.scheduling.task_dispatch import DispatchMove
ROOT=Path(__file__).resolve().parents[1]

def test_24_starts_leave_bays_and_critical_areas_clear_and_reuse_source_slices():
    spec=json.loads((ROOT/'config/runtime/industrialarea_ue_fleet24_tasks.json').read_text())
    n=ServiceNetwork(ROOT/spec['network'],blocked_edges=spec['blocked_candidate_edges'])
    m=TransportMap(ROOT/spec['map']);g=ServiceRouteGeometry(n,m)
    assert len(spec['positions'])==len(set(spec['positions'].values()))==24
    bodies=[]
    bays=[envelope(VehicleState(k,*v['rear_xy_m'],math.radians(v['heading_deg']))) for k,v in n.services.items() if v['kind']!='staging']
    for start in spec['staging']:
        body=envelope(VehicleState(start['vehicle'],*start['rear_xy_m'],math.radians(start['heading_deg'])))
        assert all(not overlaps(body,b,.5) for b in bays)
        assert all(not overlaps(body,zone_box(z),.5) for z in m.shared_resources.values())
        assert all(not overlaps(body,b,1.) for b in bodies);bodies.append(body)
        route=n.route(start['node'],'W1-loading')
        assert route is not None
        p=g.plan(DispatchMove(1,'t',start['vehicle'],'visit',start['node'],'W1-loading',route,False))
        assert math.dist(p.point(0),start['rear_xy_m'])<1e-6
        for index in range(len(g.parts[route['edge_ids'][0]])):
            new=m.edges[g.parts[route['edge_ids'][0]][index]]
            old=m.edges[start['source_edge']+'__test'+str(start['source_slice']+index)]
            assert new['centerline']==old['centerline'] and new['capacity_alias']==old['capacity_alias']

def test_leader_distance_filter_cannot_skip_a_valid_projected_leader():
    # Arc length bounds Euclidean distance even around a corner. Match the
    # actual following projection, including its sampled upper segment.
    spec=json.loads((ROOT/'config/runtime/industrialarea_ue_continuous_tasks.json').read_text())
    n=ServiceNetwork(ROOT/'config/topology/industrialarea_ue_service_network.json',blocked_edges=spec['blocked_candidate_edges'])
    g=ServiceRouteGeometry(n,TransportMap(ROOT/spec['map']))
    r=n.route('W1-loading','W3-unloading');p=g.plan(DispatchMove(1,'t','v','visit','W1-loading','W3-unloading',r,False))
    for s in range(0,int(p.length),10):
        for advance in (1,15,35,35.2):
            q=p.point(min(p.length,s+advance));h=p.heading(min(p.length,s+advance))
            other=(q[0]-1.39*math.sin(h),q[1]+1.39*math.cos(h))
            error,leader=p.project(other,s,behind=0,ahead=35)
            if error<1.4 and leader>s+.1:assert math.dist(p.point(s),other)<38.
