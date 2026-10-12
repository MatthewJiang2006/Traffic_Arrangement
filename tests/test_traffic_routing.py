"""Task endpoints stay fixed while traffic changes routes and parking choices."""
import json,math
from types import SimpleNamespace
import pytest
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.scheduling.traffic_routing import TrafficRouting
from logistics_park.scheduling.task_dispatch import TaskDispatcher,TransportWork
from logistics_park.scheduling.fleet_lifecycle import FleetRegistry
from logistics_park.common.messages import VehicleState
from logistics_park.map.geometry import Polyline

def network(tmp_path,alternatives=True):
    edges=[]
    for key,a,b,cost,alias in [('direct','L','U',4,'main'),('alt1','L','X',3,'other1'),('alt2','X','U',3,'other2'),('inQ','L','Q',3,'inQ'),('outQ','Q','U',4,'outQ')]:
        edges.append(dict(edge_id=key,from_node=a,to_node=b,kind='road',length_m=cost*8,estimated_travel_time_s=cost,capacity_alias=alias))
    raw=dict(nodes=[dict(node_id=k) for k in ['L','U','X','Q']],edges=edges,
        service_nodes={'L':dict(kind='loading',capacity=1,warehouse='W1'),'U':dict(kind='unloading',capacity=1,warehouse='W2'),'Q':dict(kind='waiting',capacity=1)},
        movements=[dict(incoming_edge='alt1',outgoing_edge='alt2')])
    path=tmp_path/'network.json';path.write_text(json.dumps(raw))
    return ServiceNetwork(path,blocked_edges=frozenset() if alternatives else {'alt1','alt2'})

def policy(n):
    paths={e['capacity_alias']:Polyline(((0,0),(e['length_m'],0))) for e in n.edges.values()}
    model=SimpleNamespace(physical_paths=paths,raw={'physical_capacities':{a:4 for a in paths}},edges={})
    return TrafficRouting(n,model)

def test_weighted_route_avoids_busy_road_and_keeps_same_destination(tmp_path):
    n=network(tmp_path)
    assert n.route('L','U')['edge_ids']==['direct']
    route=n.route('L','U',edge_costs={'direct':20})
    assert route['edge_ids']==['alt1','alt2'] and route['freeflow_travel_time_s']==6
    assert route['congestion_delay_s']==0
    assert n.route('L','U',occupied_nodes={'U'},edge_costs={'direct':20}) is None

@pytest.mark.parametrize('costs',[{'unknown':1},{'direct':-1},{'direct':float('nan')}])
def test_invalid_traffic_costs_rejected(tmp_path,costs):
    with pytest.raises(ValueError):network(tmp_path).route('L','U',edge_costs=costs)

def test_dispatch_selects_offroad_wait_then_resumes_after_queue_clears(tmp_path):
    n=network(tmp_path,alternatives=False);t=policy(n);f=FleetRegistry()
    d=TaskDispatcher(n,f,service_s=0,traffic_routing=t)
    v=f.adopt_present_vehicle('truck',0,fully_inside=True,resources=[]);d.place_vehicle('truck',v.visit_id,'L',0,stopped=True)
    t.observed={'main':{'lead1':0,'lead2':0}};d.submit(TransportWork('task','L','U'))
    assert not d.tick(0)  # load
    move=d.tick(1)[0]
    assert move.destination=='Q' and move.waiting and d.assignments['truck'].work.unloading_node=='U'
    d.departed(move.command_id,'truck',move.visit_id,2,tail_clear=True)
    d.arrived(move.command_id,'truck',move.visit_id,10,stopped=True)
    # Minimum waiting starts from actual stopped arrival, not departure.
    assert not d.tick(12)
    t.observed={};t.cache.clear()
    resumed=d.tick(15)[0]
    assert resumed.origin=='Q' and resumed.destination=='U' and not resumed.waiting
    assert any(e['event']=='TRAFFIC_WAIT_SELECTED' for e in d.events)
    assert any(e['event']=='TRAFFIC_WAIT_RELEASED' for e in d.events)

def test_short_queue_follows_without_offroad_detour(tmp_path):
    n=network(tmp_path,alternatives=False);t=policy(n);t.observed={'main':{'leader':6.}}
    f=FleetRegistry();d=TaskDispatcher(n,f,service_s=0,traffic_routing=t)
    v=f.adopt_present_vehicle('truck',0,fully_inside=True,resources=[]);d.place_vehicle('truck',v.visit_id,'L',0,stopped=True)
    d.submit(TransportWork('task','L','U'));d.tick(0);move=d.tick(1)[0]
    assert move.destination=='U' and not move.waiting

def test_own_occupancy_is_excluded_and_road_demand_influences_route(tmp_path):
    n=network(tmp_path);t=policy(n);t.observed={'main':{'ego':0}}
    assert t.costs_for('ego',{})=={}
    t.cache.clear();t.observed['main']['other']=0
    assert n.route('L','U',edge_costs=t.costs_for('ego',{}))['edge_ids']==['alt1','alt2']

def test_forecast_uses_remaining_route_not_already_traversed_origin(tmp_path):
    n=network(tmp_path);t=policy(n)
    move=SimpleNamespace(command_id=1,origin='L',destination='U',route={'edge_ids':['direct','alt1','alt2']})
    t.forecasts={'other':('L',('U',),{'other2'},1)}
    costs=t.costs_for('ego',{'other':move})
    assert 'direct' not in costs and 'alt1' not in costs and costs['alt2']>0

def test_waiting_hysteresis_and_bounded_wait(tmp_path):
    t=policy(network(tmp_path))
    assert not t.may_resume({'congestion_delay_s':10},10)
    assert t.may_resume({'congestion_delay_s':7},10)
    assert not t.may_resume({'congestion_delay_s':0},4)
    assert t.may_resume({'congestion_delay_s':40},60)

def test_foreign_service_bay_is_not_a_congestion_shortcut(tmp_path):
    n=network(tmp_path);n.services['X']=dict(kind='loading',warehouse='W3')
    assert n.route('L','U',edge_costs={'direct':20})['edge_ids']==['alt1','alt2']
    r=n.route('L','U',edge_costs={'direct':20},transit_warehouses={'W1','W2'})
    assert r['edge_ids']==['direct']

def test_observed_staging_truck_counts_on_real_road_and_snapshots_refresh(tmp_path):
    n=network(tmp_path);n.services['L']['kind']='staging';t=policy(n)
    controller=SimpleNamespace(executions={'ego':SimpleNamespace(finished=True)})
    state=VehicleState('ego',0,0,0,speed=0)
    t.observe({'ego':state},controller,{'ego':'L'},0)
    assert t.observed['main']['ego']==0
    state.speed=6;t.observe({'ego':state},controller,{'ego':'L'},.2)
    assert t.observed['main']['ego']==0
    t.observe({'ego':state},controller,{'ego':'L'},.5)
    assert t.observed['main']['ego']==6
    with pytest.raises(ValueError):t.observe({'ego':state},controller,{'ego':'L'},.4)


def test_new_command_does_not_reuse_previous_leg_forecast(tmp_path):
    n=network(tmp_path);t=policy(n)
    t.forecasts={'other':('L',('U',),{'other2'},1)}
    move=SimpleNamespace(command_id=2,origin='L',destination='U',route={'edge_ids':['direct']})
    assert 'direct' in t.costs_for('ego',{'other':move})


def test_offroad_wait_is_rejected_when_detour_erases_benefit(tmp_path):
    t=policy(network(tmp_path))
    direct={'estimated_travel_time_s':30,'congestion_delay_s':25}
    options=[(40,'Q',{'estimated_travel_time_s':20,'waiting_onward_freeflow_s':10})]
    assert t.waiting_choice(direct,options) is None


def test_occupied_waiting_point_is_never_selected_for_traffic_wait(tmp_path):
    n=network(tmp_path,alternatives=False);t=policy(n);t.observed={'main':{'leader':0,'leader2':0}}
    f=FleetRegistry();d=TaskDispatcher(n,f,service_s=0,traffic_routing=t)
    for name,node in [('ego','L'),('parked','Q')]:
        v=f.adopt_present_vehicle(name,0,fully_inside=True,resources=[]);d.place_vehicle(name,v.visit_id,node,0,stopped=True)
    d.submit(TransportWork('task','L','U'));d.tick(0)
    move=d.tick(1)[0]
    assert move.destination=='U' and not move.waiting and d.holders['Q']=='parked'


def test_live_safe_approach_can_replace_static_queue_access(tmp_path):
    n=network(tmp_path,alternatives=False);t=policy(n);f=FleetRegistry();d=TaskDispatcher(n,f,traffic_routing=t)
    for name,node in [('ego','L'),('busy','U')]:
        v=f.adopt_present_vehicle(name,0,fully_inside=True,resources=[]);d.place_vehicle(name,v.visit_id,node,0,stopped=True)
    original=n.queue_options
    n.queue_options=lambda a,b:[{**option,'usable_while_destination_occupied':False} for option in original(a,b)]
    options=d._waiting_options('L','U','ego')
    assert options and options[0][1]=='Q'  # Actual approach does not touch U.


def test_real_service_route_alternatives_keep_checked_geometry():
    from pathlib import Path
    from logistics_park.runtime.transport_routes import TransportMap
    from logistics_park.runtime.task_execution import ServiceRouteGeometry
    from logistics_park.scheduling.task_dispatch import DispatchMove
    root=Path(__file__).resolve().parents[1]
    spec=json.loads((root/'config/runtime/industrialarea_ue_continuous_tasks.json').read_text())
    n=ServiceNetwork(root/'config/topology/industrialarea_ue_service_network.json',blocked_edges=spec['blocked_candidate_edges'])
    model=TransportMap(root/spec['map']);g=ServiceRouteGeometry(n,model)
    for origin,target in [('W1-loading','W3-unloading'),('W2-loading','W4-unloading'),('W3-loading','W1-unloading'),('W4-loading','W2-unloading'),('W5-loading','W6-unloading'),('W6-loading','W5-unloading')]:
        baseline=n.route(origin,target)
        costs={key:20. for key in baseline['edge_ids'] if n.edges[key]['kind']=='road'}
        route=n.route(origin,target,edge_costs=costs,transit_warehouses={n.services[origin]['warehouse'],n.services[target]['warehouse']})
        assert route is not None and not set(route['edge_ids']).intersection(n.blocked_edges)
        plan=g.plan(DispatchMove(1,'task','truck','visit',origin,target,route,False))
        assert math.dist(plan.point(0),n.services[origin]['rear_xy_m'])<=.15
        assert math.dist(plan.point(plan.length),n.services[target]['rear_xy_m'])<=.15


def test_turning_wait_entry_checks_body_even_when_leader_axle_is_off_centerline():
    from pathlib import Path
    from logistics_park.runtime.transport_routes import TransportMap
    from logistics_park.runtime.task_execution import ServiceRouteGeometry
    from logistics_park.runtime.transport_control import TransportController,Execution,envelope
    from logistics_park.scheduling.task_dispatch import DispatchMove
    root=Path(__file__).resolve().parents[1]
    spec=json.loads((root/'config/runtime/industrialarea_ue_continuous_tasks.json').read_text())
    n=ServiceNetwork(root/'config/topology/industrialarea_ue_service_network.json',blocked_edges=spec['blocked_candidate_edges'])
    model=TransportMap(root/spec['map']);g=ServiceRouteGeometry(n,model)
    r=n.route('W1-loading','Q10');p=g.plan(DispatchMove(1,'t','ego','visit','W1-loading','Q10',r,True))
    # Reproduce the Q10 nose overlap from the 24-car lightweight trace.
    nearest=min(range(len(p.points)),key=lambda i:math.dist(p.points[i],(-12.1,110.)))
    progress=p.stations[nearest];ego=VehicleState('ego',*p.point(progress),p.heading(progress),speed=1.5)
    leader=VehicleState('leader',-12.102789,119.986544,math.pi/2,speed=0)
    c=TransportController(model,{});c.states={'ego':ego,'leader':leader};c.boxes={k:[envelope(v)] for k,v in c.states.items()}
    ex=Execution(p,progress=progress)
    error,_=p.project((leader.x,leader.y),progress,behind=0,ahead=35)
    assert error>1.4  # Old centerline-only following did not detect this leader.
    distance=c._stationary_obstacle_distance('ego',ego,ex)
    assert distance is not None and distance<8


def test_stationary_body_guard_allows_parallel_opposite_lane():
    from logistics_park.runtime.transport_control import TransportController,Execution,envelope
    class Plan:
        length=100
        def point(self,s):return (s,0)
        def heading(self,s):return 0
    c=TransportController.__new__(TransportController)
    ego=VehicleState('ego',0,0,0,speed=8)
    other=VehicleState('other',20,3.8,math.pi,speed=0)
    c.states={'ego':ego,'other':other};c.boxes={k:[envelope(v)] for k,v in c.states.items()}
    assert c._stationary_obstacle_distance('ego',ego,Execution(Plan())) is None
    # A high-speed truck must see a stopped vehicle beyond the old 35 m window.
    c.states['other'].x=60;c.states['other'].y=0;c.states['other'].heading=0
    c.boxes['other']=[envelope(c.states['other'])]
    distance=c._stationary_obstacle_distance('ego',ego,Execution(Plan()))
    assert distance is not None and distance>35


def test_following_requests_next_free_region_before_tail_leaves_current_one(tmp_path):
    from logistics_park.runtime.transport_routes import TransportMap,RouteGeometry
    from logistics_park.runtime.transport_control import TransportController
    nodes=[dict(node_id='n'+str(i),node_type='WAYPOINT',position=[10*i,0]) for i in range(3)]
    edges=[dict(edge_id='e'+str(i),from_node='n'+str(i),to_node='n'+str(i+1),centerline=[[10*i,0],[10*(i+1),0]],width=3.8,speed_limit=2,storage_capacity=2,capacity_alias='road',kind='road',maneuver='STRAIGHT') for i in range(2)]
    raw=dict(nodes=nodes,edges=edges,movements=[dict(movement_id='m',incoming_edge='e0',outgoing_edge='e1',maneuver='STRAIGHT',reference_path=[[10,0],[11,0]],conflict_zones=[],speed_limit=2)],
             physical_capacities={'road':2},physical_centerlines={'road':[[0,0],[20,0]]},sites={},control_zones={},
             shared_resources={'A':dict(bounds=[0,-2,11,2],edge_ids=['e0']),'B':dict(bounds=[13,-2,20,2],edge_ids=['e1'])})
    path=tmp_path/'map.json';path.write_text(json.dumps(raw));model=TransportMap(path)
    plan=RouteGeometry(model,['e0','e1'],[dict(event='ARRIVAL',after_edge_count=2,leg=0,site='end',node='n2')],'begin',['end'])
    for pipeline in (False,True):
        c=TransportController(model,{'ego':plan},service_s=0,shared_resources=True,collect_at_destination=True,pipeline_shared=pipeline)
        c.activate('ego',0);c.executions['ego'].progress=6
        c.shared_scheduler.request('A','ego',0)
        commands=c.tick({'ego':VehicleState('ego',6,0,0)},0)
        assert c.shared_scheduler.owners['A']=='ego'
        if pipeline:
            assert c.shared_scheduler.owners['B']=='ego' and commands['ego'][0]>0
        else:
            assert 'B' not in c.shared_scheduler.owners and commands['ego'][0]==0


def test_shared_request_uses_physical_leader_not_registration_order():
    from logistics_park.runtime.transport_control import TransportController,Execution
    from logistics_park.runtime.transport_routes import SharedVisit
    class Plan:
        def point(self,s):return (s,0)
        def heading(self,s):return 0
        def project(self,p,previous,**kw):return abs(p[1]),p[0]
    c=TransportController.__new__(TransportController)
    c.states={'rear':VehicleState('rear',0,0,0),'front':VehicleState('front',15,0,0),'other_lane':VehicleState('other_lane',5,3.8,0)}
    visit=SharedVisit('junction',30,40,0,1)
    assert c._leader_before_shared('rear',Execution(Plan()),visit)=='front'
    assert c._leader_before_shared('front',Execution(Plan(),progress=15),visit) is None


def test_deferring_shared_request_cannot_cancel_physical_owner():
    from logistics_park.scheduling.shared_resource import SharedResourceReservations
    r=SharedResourceReservations(['entry']);r.request('entry','inside',0)
    r.request('entry','rear',1)
    assert r.cancel_request('entry','rear',2) and r.queued('entry')==()
    with pytest.raises(ValueError):r.cancel_request('entry','inside',2)
    assert r.owners['entry']=='inside'
