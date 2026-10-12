"""Cross-warehouse cargo, reverse transitions, repeated reservation and return."""
import json
import math
from pathlib import Path
import pytest
from logistics_park.common.messages import VehicleState
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.transport_control import TransportController
from logistics_park.runtime.ue_transport import configured_plans
ROOT=Path(__file__).resolve().parents[1]

@pytest.mark.parametrize('vehicles,cycles',[(1,1),(2,1),(1,2)])
def test_transport_delivers_between_distinct_warehouses_and_returns(vehicles,cycles):
    s=json.loads((ROOT/f'config/runtime/industrialarea_ue_interwarehouse_{vehicles}v_{cycles}cycles.json').read_text())
    plans=configured_plans(TransportMap(ROOT/s['map']),s)
    c=TransportController(next(iter(plans.values())).model,plans,service_s=3,shared_resources=True)
    states={n:VehicleState(n,*p.point(0),p.heading(0)) for n,p in plans.items()}
    for n in plans:c.activate(n,0)
    dt=.05;commands={}
    for tick in range(18000):
        if tick%2==0:
            commands=c.tick(states,tick*dt)
            if all(e.finished for e in c.executions.values()):break
        for n,state in states.items():
            target,delta=commands.get(n,(0,0))
            v=state.speed+max(-.6*dt,min(.6*dt,target-state.speed))
            h=state.heading+.5*v*math.tan(delta)/2.3285*dt
            state.x+=v*math.cos(h)*dt;state.y+=v*math.sin(h)*dt
            state.heading+=v*math.tan(delta)/2.3285*dt;state.speed=v
    else:pytest.fail('cross-warehouse cycle timed out')
    r=c.report();assert r['status']=='PASS'
    assert r['collision_count']==r['unauthorized_resource_entry_count']==r['unauthorized_shared_entry_count']==0
    assert not c.shared_scheduler.owners and not any(station.occupants for station in c.services.values())
    for n,ex in c.executions.items():
        operations=[e for e in c.events if e.get('vehicle_id')==n and e['event'] in ('LOADING_COMPLETE','UNLOADING_COMPLETE')]
        assert [(e['site'],e['cargo_units']) for e in operations]==[('W5-loading',1),('W6-unloading',0)]*cycles
        assert ex.cargo_units==0
        assert ex.started_legs==set(range(len(plans[n].stops)))
        assert math.dist((states[n].x,states[n].y),plans[n].point(plans[n].length))<.25
        changes=[e for e in c.events if e.get('vehicle_id')==n and e['event']=='GEAR_CHANGE']
        assert len(changes)==cycles*3+cycles-1
        assert all(abs(e['speed_m_s'])<.08 for e in changes)
        assert len([e for e in c.events if e.get('vehicle_id')==n and e['event']=='SHARED_RESOURCE_RELEASED'])==cycles*3
        releases=[e for e in c.events if e.get('vehicle_id')==n and e['event']=='TRANSPORT_TASK_RELEASED']
        returns=[e for e in c.events if e.get('vehicle_id')==n and e['event']=='TRANSPORT_CYCLE_RETURNED']
        assert [e['cycle'] for e in releases]==list(range(cycles))
        assert [e['cycle'] for e in returns]==list(range(cycles))
        if cycles>1:assert releases[1]['time']>=returns[0]['time']
    if vehicles==2:assert r['maximum_moving_vehicles']==2


def test_projection_cannot_jump_to_overlapping_reverse_entry_before_stopping():
    s=json.loads((ROOT/'config/runtime/industrialarea_ue_interwarehouse_1v_1cycles.json').read_text())
    p=configured_plans(TransportMap(ROOT/s['map']),s)['Car_0_0']
    stop=next(cp['s'] for cp in p.checkpoints if cp['event']=='GEAR_CHANGE')
    error,station=p.project(p.point(stop-.3),stop-.4,stop_s=stop)
    assert error<1e-6 and station==pytest.approx(stop-.3)
