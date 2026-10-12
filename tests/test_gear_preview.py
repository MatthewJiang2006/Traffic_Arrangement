"""Forward steering must never preview a reverse leg across road slices."""
from types import SimpleNamespace
import pytest
from logistics_park.map.geometry import Polyline
from logistics_park.runtime.transport_routes import RouteGeometry


def plan():
    edges={
        'a':dict(centerline=[[0,0],[5,0]],reverse=False,kind='road'),
        'b':dict(centerline=[[5,0],[10,0]],reverse=False,kind='road'),
        'c':dict(centerline=[[10,0],[9,-1],[8,-2]],reverse=True,kind='service_access')}
    for key,row in edges.items():row.update(speed_limit=8 if key!='c' else 1,maneuver='STRAIGHT',resource=None,capacity_alias=key)
    model=SimpleNamespace(edges=edges,aliases={k:k for k in edges},zones={},shared_resources={},physical_paths={k:Polyline(tuple(map(tuple,e['centerline']))) for k,e in edges.items()},raw={'speed_adaptive_steering':True})
    return RouteGeometry(model,list(edges),[dict(event='GEAR_CHANGE',after_edge_count=2,direction=-1)],'start',['end'])


@pytest.mark.parametrize('station',[4.,4.8,8.])
def test_forward_preview_stays_on_forward_lane_before_reverse(station):
    p=plan()
    # At 8 m/s the 6.4 m preview crosses multiple 5 m road slices.
    assert p.steering((station,0),0,station,speed=8)==pytest.approx(0)


def test_reverse_control_uses_reverse_geometry_after_the_gear_boundary():
    p=plan();station=10.1;point=p.point(station);heading=p.heading(station)
    assert p.reverse(station)
    assert abs(p.steering(point,heading,station,speed=-.5))<.01


def test_reverse_preview_is_independent_of_storage_slices():
    whole=plan()
    model=whole.model
    model.edges['d']={**model.edges['c'],'centerline':[[9,-1],[8,-2]]}
    model.edges['c']={**model.edges['c'],'centerline':[[10,0],[9,-1]]}
    model.aliases['d']='d'
    model.physical_paths['d']=Polyline(((9,-1),(8,-2)))
    sliced=RouteGeometry(model,['a','b','c','d'],[dict(event='GEAR_CHANGE',after_edge_count=2,direction=-1)],'start',['end'])
    station=10.8
    # A lateral error close to a slice boundary used to produce a much sharper
    # correction than the same continuous reverse geometry without that slice.
    point=whole.point(station);point=(point[0]+.15,point[1]-.1)
    assert sliced.steering(point,whole.heading(station),station,-.7)==pytest.approx(whole.steering(point,whole.heading(station),station,-.7))
