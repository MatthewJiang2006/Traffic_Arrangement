"""Round40 corner with the measured steering response and actuator lag."""
import json,math
import pytest
from pathlib import Path
from logistics_park.runtime.transport_routes import TransportMap,RouteGeometry
ROOT=Path(__file__).resolve().parents[1]
KEY='turn_j_3_0_N_E_right_single'

def plan(key=KEY):
 m=TransportMap(ROOT/'config/topology/industrialarea_ue_internal_waiting_transport.json')
 keys=sorted([k for k,e in m.edges.items() if e['capacity_alias']==key],key=lambda k:int(k.rsplit('test',1)[1]))
 return RouteGeometry(m,keys,[],key,[])

def test_repaired_corner_preserves_endpoints_and_matches_service_network():
 p=plan();net=json.loads((ROOT/'config/topology/industrialarea_ue_internal_waiting_network.json').read_text())
 e=next(e for e in net['edges'] if e['edge_id']==KEY)
 assert p.point(0)==(-86.4937,273.0451)
 assert math.dist(p.point(p.length),(-80.1878,266.9264))<1e-8
 assert p.model.physical_paths[KEY].points==tuple(map(tuple,e['centerline']))
 assert all(math.dist(p.model.edges[a]['centerline'][-1],p.model.edges[b]['centerline'][0])<1e-8 for a,b in zip(p.edges,p.edges[1:]))

def test_delayed_turn_still_corrects_approach_error_without_early_yaw_on_path():
 p=plan();s=.5;point=p.point(s);h=p.heading(s)
 assert abs(p.steering(point,h,s))<1e-8
 assert p.steering((point[0]-.2,point[1]),h,s)>0


@pytest.mark.parametrize('key',[KEY,'turn_j_1_0_N_E_right_single','turn_j_0_0_N_E_right_single'])
def test_measured_steering_efficiency_with_lag_can_follow_repaired_corner(key):
 p=plan(key);s=0.;x,y=p.point(0);h=p.heading(0);actual_delta=0.;peak=0.
 for _ in range(220):
  error,station=p.project((x,y),s,ahead=3);s=max(s,station);peak=max(peak,error)
  if s>=p.length-.25:break
  delta=p.steering((x,y),h,s,speed=1.5)
  # Measured max wheel response is ~33 degrees at a 40-degree command.
  actual_delta+=(.82*delta-actual_delta)*.08/.2
  omega=1.5*math.tan(actual_delta)/2.3285;mid=h+.5*omega*.08
  x+=1.5*math.cos(mid)*.08;y+=1.5*math.sin(mid)*.08;h+=omega*.08
 assert s>=p.length-.25
 assert peak<.7


def test_formal_service_task_uses_calibrated_corner_without_stale_slices():
 from types import SimpleNamespace
 from logistics_park.routing.service_network import ServiceNetwork
 from logistics_park.runtime.task_execution import ServiceRouteGeometry
 n=ServiceNetwork(ROOT/'config/topology/industrialarea_ue_internal_waiting_network.json')
 m=TransportMap(ROOT/'config/topology/industrialarea_ue_internal_waiting_transport.json')
 route=n.route('W4-loading','W2-unloading')
 assert KEY in route['edge_ids']
 p=ServiceRouteGeometry(n,m).plan(SimpleNamespace(origin='W4-loading',destination='W2-unloading',route=route))
 repaired=[e for e in p.edges if m.edges[e]['capacity_alias']==KEY]
 assert len(repaired)==3
 assert all(m.edges[e]['right_turn_tracking_gain']==1.2 for e in repaired)


def test_round43_failed_corner_preserves_endpoints_and_formal_route():
 from types import SimpleNamespace
 from logistics_park.routing.service_network import ServiceNetwork
 from logistics_park.runtime.task_execution import ServiceRouteGeometry
 key='turn_j_1_0_N_E_right_single';p=plan(key)
 assert math.dist(p.point(0),(-86.4936,145.0211))<1e-8
 assert math.dist(p.point(p.length),(-80.1877,138.9024))<1e-8
 n=ServiceNetwork(ROOT/'config/topology/industrialarea_ue_internal_waiting_network.json')
 edge=n.edges[key]
 assert p.model.physical_paths[key].points==tuple(map(tuple,edge['centerline']))
 route=n.route('W4-loading','W5-unloading')
 assert key in route['edge_ids']
 joined=ServiceRouteGeometry(n,p.model).plan(SimpleNamespace(origin='W4-loading',destination='W5-unloading',route=route))
 slices=[e for e in joined.edges if p.model.aliases[e]==key]
 assert len(slices)==3 and all(p.model.edges[e]['right_turn_tracking_gain']==1.2 for e in slices)


def test_recorded_j_0_0_deviation_route_uses_calibrated_continuous_geometry():
 from types import SimpleNamespace
 from logistics_park.routing.service_network import ServiceNetwork
 from logistics_park.runtime.task_execution import ServiceRouteGeometry
 fixture=json.loads((ROOT/'tests/fixtures/ue_20261006_right_turn_deviation.json').read_text())
 key='turn_j_0_0_N_E_right_single';p=plan(key)
 assert math.dist(p.point(0),(-86.4936,97.2211))<1e-8
 assert math.dist(p.point(p.length),(-80.1877,91.1024))<1e-8
 network=ServiceNetwork(ROOT/'config/topology/industrialarea_ue_internal_waiting_network.json')
 edge=network.edges[key]
 assert p.model.physical_paths[key].points==tuple(map(tuple,edge['centerline']))
 assert all(math.dist(p.model.edges[a]['centerline'][-1],p.model.edges[b]['centerline'][0])<1e-8
            for a,b in zip(p.edges,p.edges[1:]))
 assert all(p.model.edges[e]['maximum_path_error_m']==1.2 for e in p.edges)
 route=fixture['route'];origin=network.edges[route['edge_ids'][0]]['from_node']
 joined=ServiceRouteGeometry(network,p.model).plan(SimpleNamespace(origin=origin,
        destination=fixture['destination'],route=route))
 slices=[e for e in joined.edges if joined.model.aliases[e]==key]
 assert len(slices)==3 and all(joined.model.edges[e]['right_turn_tracking_gain']==1.2 for e in slices)
