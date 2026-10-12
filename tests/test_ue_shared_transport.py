import json
from pathlib import Path
import pytest
from logistics_park.common.messages import VehicleState
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.transport_control import TransportController
from logistics_park.runtime.ue_transport import configured_plans
ROOT=Path(__file__).resolve().parents[1]


def fixture():
    scenario=json.loads((ROOT/'config/runtime/industrialarea_ue_shared_transport.json').read_text())
    model=TransportMap(ROOT/scenario['map'])
    return model,configured_plans(model,scenario)


def advance(controller,name,x):
    ex=controller.executions[name]
    y=91.0989 if name=='Car_0_0' else 87.2989
    error,s=ex.plan.project((x,y),0,behind=0,ahead=ex.plan.length)
    assert error<.01
    ex.progress=s;ex.cleared=set(range(len(ex.plan.visits)))
    return VehicleState(name,x,y,ex.plan.heading(s),speed=0)


def test_shared_channel_holds_actual_tail_after_axle_crosses_boundary():
    model,plans=fixture();c=TransportController(model,plans,shared_resources=True)
    for n in plans:c.activate(n,0)
    channel='shared_corridor_j0';owner='Car_0_0';follower='Car_0_1'
    c.shared_scheduler.request(channel,owner,0);c.shared_scheduler.request(channel,follower,0)
    state=advance(c,owner,4)
    ex=c.executions[owner];ex.plan.shared_visits[0].end=ex.progress
    ex.shared_entered.add(0);ex.shared_granted.add(0);ex.shared_requested.add(0)
    other=advance(c,follower,24)
    c.tick({owner:state,follower:other},1)
    assert c.shared_scheduler.allows(channel,owner)
    assert not ex.shared_cleared
    c.tick({owner:advance(c,owner,9),follower:other},2)
    assert c.shared_scheduler.allows(channel,follower)
    assert ex.shared_cleared=={0}
    assert any(e['event']=='SHARED_RESOURCE_RELEASED' and e['tail_clear'] for e in c.events)


def test_shared_physical_front_cannot_enter_without_grant():
    model,plans=fixture();c=TransportController(model,plans,shared_resources=True)
    c.activate('Car_0_0',0)
    c.shared_scheduler.request('shared_corridor_j0','other_truck',0)
    state=advance(c,'Car_0_0',0)
    # Axle is west of the boundary, but the actual front envelope is inside.
    with pytest.raises(RuntimeError,match='unauthorized shared entry'):
        c.tick({'Car_0_0':state},1)
    assert c.shared_unauthorized_count==1


def test_upstream_junction_does_not_grant_into_someone_elses_shared_channel():
    model,plans=fixture();c=TransportController(model,plans,shared_resources=True)
    c.activate('Car_0_0',0)
    ex=c.executions['Car_0_0'];ex.requested.add(0)
    c.shared_scheduler.request('shared_corridor_j0','Car_0_1',0)
    assert not c._outlet_clear(ex.plan.visits[0].outgoing,'Car_0_0')


def test_terminal_service_must_finish_before_vehicle_is_collected():
    model,plans=fixture();c=TransportController(model,plans,service_s=2)
    n='Car_0_0';c.activate(n,0)
    state=advance(c,n,12)
    c.tick({n:state},1)
    assert not c.executions[n].finished
    c.tick({n:state},3.1)
    assert c.executions[n].finished and c.executions[n].completed_legs=={0}


def test_two_opposing_trucks_finish_after_real_fifo_wait():
    import math
    model,plans=fixture();c=TransportController(model,plans,shared_resources=True)
    states={n:VehicleState(n,*p.point(0),p.heading(0),speed=0) for n,p in plans.items()}
    for n in plans:c.activate(n,0)
    dt=.02;commands={}
    for tick in range(6000):
        if tick%5==0:
            commands=c.tick(states,tick*dt)
            if all(ex.finished for ex in c.executions.values()):break
        for n,s in states.items():
            target,delta=commands.get(n,(0,0))
            speed=max(0,s.speed+max(-.6*dt,min(.5*dt,target-s.speed)))
            middle=s.heading+.5*speed*math.tan(delta)/2.3285*dt
            s.x+=speed*math.cos(middle)*dt;s.y+=speed*math.sin(middle)*dt
            s.heading+=speed*math.tan(delta)/2.3285*dt;s.speed=speed
    assert c.report()['status']=='PASS'
    assert c.shared_scheduler.owners=={}
    assert c.shared_unauthorized_count==c.collision_count==0
    assert [e['vehicle_id'] for e in c.events if e['event']=='SHARED_RESOURCE_ENTERED']==list(plans)
    assert c.executions['Car_0_1'].waiting_s>2
    assert all(e.get('tail_clear') for e in c.events if e['event']=='SHARED_RESOURCE_RELEASED')


def test_terminal_speed_remains_positive_before_arrival_tolerance():
    _,plans=fixture();p=plans['Car_0_0']
    # The physical adapter brakes fully on zero; a sampled final zero must
    # not stop the truck 25 cm short of a 22 cm arrival tolerance.
    assert p.speed_cap(p.length-.24)>0
    assert p.speed_cap(p.length)==pytest.approx(0)
