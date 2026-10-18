"""Vehicle feedback must remain independent when route geometry is cached."""
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from logistics_park.common.messages import VehicleState
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.runtime.task_execution import ServiceRouteGeometry
from logistics_park.runtime.transport_control import TransportController
from logistics_park.runtime.transport_routes import TransportMap


ROOT=Path(__file__).resolve().parents[1]


def shared_plan():
    network=ServiceNetwork(ROOT/'config/topology/industrialarea_ue_internal_waiting_network.json')
    model=TransportMap(ROOT/'config/topology/industrialarea_ue_internal_waiting_transport.json')
    geometry=ServiceRouteGeometry(network,model)
    move=SimpleNamespace(origin='W4-unloading',destination='W2-loading',
                         route=network.route('W4-unloading','W2-loading'))
    plan=geometry.plan(move)
    assert geometry.plan(move) is plan
    return model,plan


@pytest.mark.parametrize('order',[('waiting','moving'),('moving','waiting')])
def test_shared_cached_route_keeps_each_vehicle_steering_equal_to_solo_execution(order):
    model,plan=shared_plan()
    controller=TransportController(model,dict.fromkeys(order,plan),service_s=0)
    solo=TransportController(model,{'moving':plan},service_s=0)
    for c in [controller,solo]:
        for name in c.plans:c.activate(name,0)
        ex=c.executions['moving'];ex.progress=53.
        ex.checkpoint=next(i for i,cp in enumerate(plan.checkpoints) if cp['s']>ex.progress)
    x,y=plan.point(53.)
    states={'waiting':VehicleState('waiting',*plan.point(0),plan.heading(0)),
            'moving':VehicleState('moving',x+.6,y,plan.heading(53.)+.09,speed=.8)}
    result=controller.tick(states,.2)['moving']
    independent=solo.tick({'moving':states['moving']},.2)['moving']
    assert result==pytest.approx(independent)
    assert result[1]<0  # Correct the lateral offset instead of increasing it.


@pytest.mark.parametrize('dt',[.13,.25,.5])
def test_shared_route_corrects_offset_with_measured_steering_gain_and_lag(dt):
    model,plan=shared_plan()
    c=TransportController(model,{'waiting':plan,'moving':plan},service_s=0)
    for name in c.plans:c.activate(name,0)
    ex=c.executions['moving'];ex.progress=53.
    ex.checkpoint=next(i for i,cp in enumerate(plan.checkpoints) if cp['s']>ex.progress)
    x,y=plan.point(53.)
    state=VehicleState('moving',x+.3,y,plan.heading(53.)+.09,speed=.8)
    waiting=VehicleState('waiting',*plan.point(0),plan.heading(0))
    wheel=0.;peak=0.
    for i in range(int(20/dt)):
        _,delta=c.tick({'waiting':waiting,'moving':state},(i+1)*dt)['moving']
        wheel+=(.82*delta-wheel)*(1-math.exp(-dt/.2))
        yaw=state.speed*math.tan(wheel)/2.3285*dt
        state.x+=state.speed*math.cos(state.heading+yaw/2)*dt
        state.y+=state.speed*math.sin(state.heading+yaw/2)*dt
        state.heading+=yaw
        error,_=plan.project((state.x,state.y),ex.progress)
        peak=max(peak,error)
    assert peak<.7
    assert error<.3


def test_next_execution_starts_with_fresh_feedback_on_reused_geometry():
    model,plan=shared_plan()
    c=TransportController(model,{'moving':plan},service_s=0,retain_finished_vehicles=True)
    c.activate('moving',0)
    old=c.executions['moving'];old.steering_feedback.observe((10,20),1.,1.)
    old.finished=True
    state=VehicleState('moving',*plan.point(0),plan.heading(0))
    c.continue_route('moving',plan,1.,state=state)
    assert c.executions['moving'].steering_feedback.pose is None
    assert c.executions['moving'].steering_feedback is not old.steering_feedback
