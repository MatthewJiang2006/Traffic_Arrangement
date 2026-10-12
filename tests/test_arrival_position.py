"""A stopped station projection alone is insufficient to release a vehicle."""
import pytest
import math
from logistics_park.common.messages import VehicleState
from logistics_park.runtime.transport_control import stopped_at_point


@pytest.mark.parametrize('degrees,arrived',[(0,True),(4.9,True),(14.7,False),(-11.5,False),(360,True)])
def test_service_pose_requires_body_heading(degrees,arrived):
    assert stopped_at_point(VehicleState('truck',0,0,math.radians(degrees)),(0,0),0) is arrived


@pytest.mark.parametrize('node',['Q30','Q32','Q33','Q34','Q36','Q5','Q8'])
def test_parking_alignment_with_measured_wheel_response_and_lag(node):
    import json
    from pathlib import Path
    from logistics_park.runtime.transport_routes import TransportMap,RouteGeometry
    root=Path(__file__).resolve().parents[1]
    model=TransportMap(root/'config/topology/industrialarea_ue_internal_waiting_transport.json')
    network=json.loads((root/'config/topology/industrialarea_ue_internal_waiting_network.json').read_text())
    edge=next(e for e in network['edges'] if e['to_node']==node)
    keys=sorted([k for k in model.edges if k.startswith(edge['edge_id']+'__test')],key=lambda k:int(k.rsplit('test',1)[1]))
    plan=RouteGeometry(model,keys,[],edge['from_node'],[node])
    plan.arrival_heading=math.radians(network['service_nodes'][node]['heading_deg'])
    s=max(0,plan.length-12);x,y=plan.point(s);heading=plan.heading(s);delta=0.;peak=0.
    for _ in range(2000):
        error,station=plan.project((x,y),s,ahead=3);s=max(s,station);peak=max(peak,error)
        if plan.length-s<.1:break
        command=plan.steering((x,y),heading,s,speed=1.)
        delta+=(.82*command-delta)*.08/.2
        speed=-1. if plan.reverse(s) else 1.
        omega=speed*math.tan(delta)/2.3285;mid=heading+.5*omega*.08
        x+=speed*math.cos(mid)*.08;y+=speed*math.sin(mid)*.08;heading+=omega*.08
    assert plan.length-s<.1
    assert math.dist((x,y),plan.point(plan.length))<.35
    assert abs((heading-plan.arrival_heading+math.pi)%(2*math.pi)-math.pi)<math.radians(5)
    assert peak<.7


@pytest.mark.parametrize('xy,speed,arrived',[
    ((-24.8956500813,275.6560375491),-.001757,False),
    ((-25.16425,275),.2,False),
    ((-25.16425,275),0,True),
    ((-24.96425,275.2),.01,True),
])
def test_arrival_requires_measured_position_and_stopped_speed(xy,speed,arrived):
    assert stopped_at_point(VehicleState('truck',*xy,0,speed=speed),(-25.16425,275)) is arrived


@pytest.mark.parametrize('offset,yaw_error',[(.656,0), (0,math.radians(14.7))])
def test_controller_does_not_service_or_collect_a_missed_waiting_pose(offset,yaw_error):
    from pathlib import Path
    from types import SimpleNamespace
    from logistics_park.routing.service_network import ServiceNetwork
    from logistics_park.runtime.transport_routes import TransportMap
    from logistics_park.runtime.task_execution import ServiceRouteGeometry
    from logistics_park.runtime.transport_control import TransportController
    root=Path(__file__).resolve().parents[1]
    model=TransportMap(root/'config/topology/industrialarea_ue_internal_waiting_transport.json')
    network=ServiceNetwork(root/'config/topology/industrialarea_ue_internal_waiting_network.json')
    route=network.route('fleet24-start-06','Q29')
    plan=ServiceRouteGeometry(network,model).plan(SimpleNamespace(origin='fleet24-start-06',destination='Q29',route=route))
    controller=TransportController(model,{'truck':plan},service_s=0,collect_at_destination=True)
    controller.activate('truck',0)
    ex=controller.executions['truck'];ex.checkpoint=len(plan.checkpoints)-1;ex.progress=plan.length-.1
    x,y=plan.point(plan.length)
    # Along-route remaining distance is within the old arrival threshold,
    # while the measured lateral miss is the one observed in UE round20.
    state=VehicleState('truck',x+.1,y+offset,plan.heading(plan.length)+yaw_error,speed=0)
    controller.tick({'truck':state},1)
    controller.tick({'truck':state},2)
    assert ex.service_until is None and not ex.finished
    assert not any(e['event'] in ('ARRIVED','TASK_COMPLETE','COLLECTED') for e in controller.events)
    state=VehicleState('truck',x,y,plan.heading(plan.length),speed=0)
    controller.tick({'truck':state},3)
    controller.tick({'truck':state},4)
    assert ex.finished
    assert [e['event'] for e in controller.events if e['event'] in ('ARRIVED','TASK_COMPLETE','COLLECTED')]==['ARRIVED','TASK_COMPLETE','COLLECTED']
