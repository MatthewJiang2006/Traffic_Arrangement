import json
import math
from pathlib import Path
import pytest
from logistics_park.common.messages import VehicleState
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.transport_control import TransportController
from logistics_park.runtime.ue_transport import configured_plans
ROOT=Path(__file__).resolve().parents[1]


def setup():
    s=json.loads((ROOT/'config/runtime/industrialarea_ue_w5_missions.json').read_text())
    m=TransportMap(ROOT/s['map']);p=configured_plans(m,s)
    return TransportController(m,p,shared_resources=True),p


def simulate():
    c,plans=setup();states={n:VehicleState(n,*p.point(0),p.heading(0)) for n,p in plans.items()}
    for n in plans:c.activate(n,0)
    dt=.02;commands={}
    for tick in range(12000):
        if tick%5==0:
            commands=c.tick(states,tick*dt)
            if all(ex.finished for ex in c.executions.values()):return c
        for n,s in states.items():
            v,delta=commands.get(n,(0,0));speed=max(0,s.speed+max(-.6*dt,min(.5*dt,v-s.speed)))
            h=s.heading+.5*speed*math.tan(delta)/2.3285*dt
            s.x+=speed*math.cos(h)*dt;s.y+=speed*math.sin(h)*dt
            s.heading+=speed*math.tan(delta)/2.3285*dt;s.speed=speed
    pytest.fail('warehouse kinematic mission timed out')


def test_two_w5_missions_load_unload_and_physically_release_berths():
    c=simulate();r=c.report()
    assert r['status']=='PASS' and r['completed_tasks']==4
    assert r['collision_count']==r['unauthorized_shared_entry_count']==0
    assert r['maximum_moving_vehicles']==2
    assert all(s.max_reserved==1 and not s.occupants for s in c.services.values())
    assert all(ex.cargo_units==0 for ex in c.executions.values())
    assert [e['vehicle_id'] for e in c.events if e['event']=='SHARED_RESOURCE_ENTERED']==['Car_0_0','Car_0_1']
    for n in c.executions:
        e=[e for e in c.events if e.get('vehicle_id')==n]
        assert [v['leg'] for v in e if v['event']=='STARTED']==[0,1]
        assert [v['event'] for v in e if v['event'] in ('LOADING_COMPLETE','UNLOADING_COMPLETE')]==['LOADING_COMPLETE','UNLOADING_COMPLETE']
        assert len([v for v in e if v['event']=='BERTH_RELEASED' and v['tail_clear']])==2
        for started,done in zip([v for v in e if v['event']=='SERVICING'],[v for v in e if v['event'] in ('LOADING_COMPLETE','UNLOADING_COMPLETE')]):
            assert done['time']-started['time']>=5
            assert not started['test_station']


def test_w5_loading_requires_reserved_single_capacity_berth():
    c,plans=setup();n='Car_0_0';c.activate(n,0)
    p=plans[n];cp=p.checkpoints[0];ex=c.executions[n];ex.progress=cp['s']
    c.shared_scheduler.request('W5_work_access',n,0)
    ex.shared_requested.add(0);ex.shared_granted.add(0);ex.shared_entered.add(0)
    c.services['W5-loading'].reserve('other','busy')
    state=VehicleState(n,*p.point(cp['s']),0)
    with pytest.raises(RuntimeError,match='unreserved warehouse arrival'):
        c.tick({n:state},1)
    assert c.services['W5-loading'].max_reserved==1


def test_w5_unloading_rejects_missing_cargo():
    c,plans=setup();n='Car_0_0';c.activate(n,0)
    ex=c.executions[n];ex.checkpoint=1;cp=plans[n].checkpoints[1];ex.progress=cp['s']
    c.shared_scheduler.request('W5_work_access',n,0);ex.shared_requested.add(0);ex.shared_granted.add(0);ex.shared_entered.add(0)
    state=VehicleState(n,*plans[n].point(cp['s']),0)
    c.tick({n:state},1)
    with pytest.raises(RuntimeError,match='unloading without cargo'):
        c.tick({n:state},6.1)
