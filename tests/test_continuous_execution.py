import json,math,sys
from pathlib import Path
import pytest
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.runtime.task_execution import ServiceRouteGeometry
from logistics_park.scheduling.task_dispatch import DispatchMove
from logistics_park.safety.collision import OBB
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from simulate_ue_continuous_tasks import setup as setup_runtime


@pytest.fixture(autouse=True)
def small_scenario(monkeypatch):
    import simulate_ue_continuous_tasks as sim
    spec=json.loads((ROOT/'config/runtime/industrialarea_ue_continuous_tasks.json').read_text())
    monkeypatch.setattr(sim,'SPEC',spec)


def setup():
    return setup_runtime(positions={'Car_0_0':'W1-loading','Car_0_1':'W2-loading'},
                         tasks=[('W1-loading','W3-unloading'),('W2-loading','W3-unloading'),('W1-loading','W4-unloading')])


def geometry():
    cfg=json.loads((ROOT/'config/runtime/industrialarea_ue_continuous_tasks.json').read_text())
    n=ServiceNetwork(ROOT/'config/topology/industrialarea_ue_service_network.json',blocked_edges=cfg['blocked_candidate_edges'])
    m=TransportMap(ROOT/'config/topology/industrialarea_ue_continuous_transport.json')
    return n,m,ServiceRouteGeometry(n,m)


def test_waiting_arrival_departure_and_reverse_geometry_compose_without_pair_manifest():
    n,m,g=geometry()
    for a,b in [('W2-loading','Q3'),('Q4','W3-unloading'),('W3-unloading','W1-loading'),('W2-loading','Q12'),('Q12','W4-unloading')]:
        route=n.route(a,b);move=DispatchMove(1,'t','v','visit',a,b,route,False)
        plan=g.plan(move)
        assert plan.point(0)==pytest.approx(n.services[a]['rear_xy_m'],abs=.15)
        assert plan.point(plan.length)==pytest.approx(n.services[b]['rear_xy_m'],abs=.15)
        for x,y in zip(plan.edges,plan.edges[1:]):
            assert m.edges[x]['centerline'][-1]==pytest.approx(m.edges[y]['centerline'][0],abs=1e-7)
        if a=='Q12':assert any(c['event']=='GEAR_CHANGE' for c in plan.checkpoints)


def test_illegal_route_and_pending_access_cannot_be_executed():
    n,m,g=geometry();r=n.route('W2-loading','Q3')
    bad={**r,'edge_ids':r['edge_ids'][1:]}
    with pytest.raises(ValueError,match='Illegal'):g.plan(DispatchMove(1,'t','v','visit','W2-loading','Q3',bad,True))
    n.edges[r['edge_ids'][0]]['access_validation_required']=True
    with pytest.raises(ValueError,match='Pending'):g.plan(DispatchMove(1,'t','v','visit','W2-loading','Q3',r,True))


def test_retained_parked_vehicles_are_observed_and_braked():
    e,states=setup();c=e.controller
    commands=c.tick(states,0)
    assert set(c.states)==set(states)
    assert all(ex.finished for ex in c.executions.values())
    assert all(command==(0.,0.) for command in commands.values())
    assert not any(event['event']=='COLLECTED' for event in c.events)


def test_collision_check_includes_retained_finished_trucks():
    e,states=setup();same=OBB(0,0,0,8,2.6)
    with pytest.raises(RuntimeError,match='vehicle envelope overlap'):
        e.controller.tick(states,0,body_boxes={n:[same] for n in states})


def test_next_leg_preserves_source_lease_and_owned_road_area():
    e,states=setup();d=e.dispatch;c=e.controller
    owned=dict(c.shared_scheduler.owners)
    e.tick(states,0);e.tick(states,5)
    assert all(c.shared_scheduler.owners[k]==v for k,v in owned.items())
    assert d.holders['W1-loading']=='Car_0_0' and d.holders['W2-loading']=='Car_0_1'
    assert d.holders['W3-unloading']=='Car_0_0'
    waiting=e.moves['Car_0_1'].destination
    assert d.network.services[waiting]['kind']=='waiting' and d.holders[waiting]=='Car_0_1'
    assert any(ev['event']=='LEG_CONTINUED' and ev['retained_resources'] for ev in c.events)
    assert not d.departures


def test_incomplete_observations_rejected_before_dispatch():
    e,states=setup()
    with pytest.raises(ValueError,match='fresh observations'):e.tick({'Car_0_0':states['Car_0_0']},0)
    assert not e.dispatch.assignments



def test_candidate_exit_is_excluded_from_operational_routes():
    n,m,g=geometry()
    # A safe forward departure cannot reach this northern destination using
    # today's lane-change ports; do not substitute the failed reversal.
    assert n.route('Q3','W3-unloading') is None
    r=n.route('Q4','W3-unloading')
    assert r['edge_ids'][0]=='access_138'
    assert not set(r['edge_ids']) & n.blocked_edges


def test_source_geometry_change_rejects_old_slices():
    n,m,g=geometry();r=n.route('W1-loading','W3-unloading')
    n.edges[r['edge_ids'][0]]['centerline'][0][0]+=.3
    with pytest.raises(ValueError,match='Stale'):
        g.plan(DispatchMove(1,'t','v','visit','W1-loading','W3-unloading',r,False))


def test_moving_vehicle_cannot_switch_legs():
    from dataclasses import replace
    e,states=setup();n=e.dispatch.network;r=n.route('W1-loading','W3-unloading')
    p=e.geometry.plan(DispatchMove(1,'t','Car_0_0','visit','W1-loading','W3-unloading',r,False))
    with pytest.raises(ValueError,match='actual stopped pose'):
        e.controller.continue_route('Car_0_0',p,0,state=replace(states['Car_0_0'],speed=1))
    assert e.controller.executions['Car_0_0'].finished


def test_handoff_cannot_forget_old_road_ownership():
    e,states=setup();n=e.dispatch.network;r=n.route('W1-loading','W3-unloading')
    p=e.geometry.plan(DispatchMove(1,'t','Car_0_0','visit','W1-loading','W3-unloading',r,False))
    p.shared_visits=[]
    owners=dict(e.controller.shared_scheduler.owners)
    with pytest.raises(ValueError,match='omits retained'):
        e.controller.continue_route('Car_0_0',p,0,state=states['Car_0_0'])
    assert e.controller.shared_scheduler.owners==owners


def test_gate_resource_snapshot_includes_physical_occupancy():
    e,states=setup();e.controller.tick(states,0)
    resources=e.resource_snapshot('Car_0_0')
    assert 'berth:W1-loading' in resources
    assert 'road:berth:W1-loading' in resources
    assert any(r.startswith('shared:') for r in resources)


def test_arrival_keeps_destination_lease_during_a_settling_impulse(monkeypatch):
    from dataclasses import replace
    e,states=setup();e.tick(states,0);e.tick(states,5)
    name='Car_0_0';move=e.moves[name];ex=e.controller.executions[name]
    ex.finished=True
    states[name]=replace(states[name],x=ex.plan.point(ex.plan.length)[0],y=ex.plan.point(ex.plan.length)[1],speed=.12)
    monkeypatch.setattr(e.controller,'tick',lambda states,now,**kw:{n:(0.,0.) for n in states})
    monkeypatch.setattr(e.dispatch,'tick',lambda now:[])
    e.tick(states,6)
    assert name in e.moves and e.dispatch.holders[move.destination]==name
    assert not any(x['event']=='NODE_ARRIVED' and x.get('vehicle_id')==name for x in e.dispatch.events)
    states[name].speed=0
    e.tick(states,7)
    assert name not in e.moves and e.dispatch.positions[name]==move.destination
