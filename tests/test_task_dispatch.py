import pytest
from logistics_park.scheduling.fleet_lifecycle import FleetRegistry,FleetPhase
from logistics_park.scheduling.task_dispatch import TaskDispatcher,TransportWork,WorkStatus


class Network:
    services={k:dict(kind=kind,capacity=1) for k,kind in
              [('L','loading'),('U','unloading'),('Q','waiting'),('A','waiting'),('B','waiting')]}
    def route(self,a,b,*,occupied_nodes=frozenset()):
        if b in occupied_nodes:return None
        return dict(edge_ids=[a+'-'+b],distance_m=10,estimated_travel_time_s=1 if a=='A' else 4)
    def queue_options(self,a,b):
        return [dict(waiting_node='Q',usable_while_destination_occupied=True,
                     attachment_validation_required=False,access_validation_required=False,departure_requires_empty=[])]


def setup(positions):
    f=FleetRegistry();d=TaskDispatcher(Network(),f,service_s=1)
    for car,node in positions.items():
        v=f.adopt_present_vehicle(car,0,fully_inside=True,resources=[])
        d.place_vehicle(car,v.visit_id,node,0,stopped=True)
    return f,d


def arrive(d,m,depart_time,arrival_time):
    d.departed(m.command_id,m.vehicle_id,m.visit_id,depart_time,tail_clear=True)
    d.arrived(m.command_id,m.vehicle_id,m.visit_id,arrival_time,stopped=True)


def test_eta_dispatch_chooses_closer_vehicle_and_reserves_approach():
    f,d=setup({'far':'B','near':'A'})
    d.submit(TransportWork('t','L','U'))
    m=d.tick(0)[0]
    assert m.vehicle_id=='near' and d.holders['L']=='near'
    assert d.holders['A']=='near'  # Source cannot be freed by merely planning.
    assert f.vehicles['far'].phase==FleetPhase.IDLE
    with pytest.raises(ValueError):d.arrived(m.command_id,'near',m.visit_id,0,stopped=True)
    with pytest.raises(ValueError):d.departed(m.command_id,'near',m.visit_id,0,tail_clear=False)
    arrive(d,m,0,1)
    assert 'A' not in d.holders


def test_remote_admission_owner_is_replaceable_and_selected_owner_can_depart():
    from logistics_park.scheduling.task_dispatch import Assignment
    f,d=setup({'far':'B','near':'A'})
    d.follow_to_busy_destination=True;d.offroad_busy_wait=True
    for name,queued in [('far',0.),('near',1.)]:
        work=TransportWork('work-'+name,'L','U');d.submit(work)
        visit=f.vehicles[name].visit_id;f.assign_task(name,visit,work.task_id,0)
        d.status[work.task_id]=WorkStatus.ACTIVE
        d.assignments[name]=Assignment(work,name,visit,state='WAITING',queued_at=queued)
    d.holders['L']='far'
    # A stopped remote holder is a revisable promise, not physical occupancy.
    d.promote_approaches({'far':(100.,False),'near':(10.,False)})
    assert d.holders['L']=='near'
    assert d._destination_queue('L')[0].vehicle_id=='far'
    # Dispatch obeys the same admission choice, even if another car queued first.
    moves=d.tick(2)
    assert [(m.vehicle_id,m.destination) for m in moves]==[('near','L')]
    assert d.holders['A']=='near' and d.holders['B']=='far'


def test_remote_promise_cannot_override_actual_dock_occupancy():
    from logistics_park.scheduling.task_dispatch import Assignment
    f,d=setup({'occupant':'L','near':'A'})
    d.follow_to_busy_destination=True;d.offroad_busy_wait=True
    work=TransportWork('work','L','U');visit=f.vehicles['near'].visit_id
    d.assignments['near']=Assignment(work,'near',visit,state='WAITING',queued_at=0.)
    d.promote_approaches({'near':(1.,False)})
    assert d.holders['L']=='occupant' and d.positions['occupant']=='L'


def test_busy_berth_uses_waiting_and_promotes_after_tail_clear():
    f,d=setup({'first':'L','second':'A'})
    d.submit(TransportWork('t1','L','U'));d.submit(TransportWork('t2','L','U'))
    move=d.tick(0)[0]
    assert move.vehicle_id=='second' and move.destination=='Q' and move.waiting
    arrive(d,move,0,1)
    moves=d.tick(1)
    assert len(moves)==1 and moves[0].vehicle_id=='first'
    assert d.holders['L']=='first'  # Service completion is not tail clearance.
    assert not d.tick(1)
    d.departed(moves[0].command_id,'first',moves[0].visit_id,1,tail_clear=True)
    promoted=d.tick(1)[0]
    assert promoted.vehicle_id=='second' and promoted.destination=='L'


def test_continuous_assignment_finishes_load_and_unload_then_next_task():
    f,d=setup({'car':'L'})
    d.submit(TransportWork('t1','L','U'));d.submit(TransportWork('t2','L','U'))
    assert not d.tick(0)
    haul=d.tick(1)[0];assert d.assignments['car'].cargo_units==1
    arrive(d,haul,1,2);assert not d.tick(2)
    pickup=d.tick(3)[0]
    assert d.status['t1']==WorkStatus.COMPLETE and d.status['t2']==WorkStatus.ACTIVE
    assert pickup.destination=='L' and len([e for e in d.events if e['event']=='TASK_DISPATCHED'])==2


def test_departure_drains_task_and_excludes_next_assignment():
    f,d=setup({'car':'L'})
    d.submit(TransportWork('t1','L','U'));d.submit(TransportWork('t2','L','U'))
    d.tick(0);v=f.vehicles['car'];f.request_exit('car',v.visit_id,'gate',0)
    haul=d.tick(1)[0];arrive(d,haul,1,2);d.tick(2)
    assert not d.tick(3)
    assert f.vehicles['car'].phase==FleetPhase.EXIT_REQUESTED
    assert d.status['t1']==WorkStatus.COMPLETE and d.status['t2']==WorkStatus.PENDING


def test_outside_unknown_or_stale_vehicle_cannot_receive_task():
    f,d=setup({});v=f.register_vehicle('new',0)
    with pytest.raises(ValueError):d.place_vehicle('new',v.visit_id,'A',0,stopped=True)
    d.submit(TransportWork('t','L','U'));assert not d.tick(0)
    f.adopt_present_vehicle('present',0,fully_inside=True,resources=[])
    assert not d.tick(0)  # No confirmed node observation.


def test_invalid_task_and_double_occupancy_rejected():
    f,d=setup({'a':'A'})
    with pytest.raises(ValueError):d.submit(TransportWork('t','Q','U'))
    d.submit(TransportWork('t','L','U'))
    with pytest.raises(ValueError):d.submit(TransportWork('t','L','U'))
    v=f.adopt_present_vehicle('b',0,fully_inside=True,resources=[])
    with pytest.raises(ValueError):d.place_vehicle('b',v.visit_id,'A',0,stopped=True)


def test_release_time_priority_and_no_safe_queue_defer_dispatch():
    f,d=setup({'car':'A'})
    d.submit(TransportWork('future','L','U',release_time=10,priority=9))
    d.submit(TransportWork('low','L','U'));d.submit(TransportWork('high','L','U',priority=1))
    assert d.tick(0)[0].task_id=='high'
    assert d.status['future']==d.status['low']==WorkStatus.PENDING


def test_shared_access_is_reserved_for_approaching_vehicle():
    f,d=setup({'a':'A','b':'B'})
    d.network.services={**d.network.services,'L2':dict(kind='loading',capacity=1,occupation_resources=['corridor'])}
    d.network.services['L']={**d.network.services['L'], 'occupation_resources':['corridor']}
    d.submit(TransportWork('t1','L','U'));d.submit(TransportWork('t2','L2','U'))
    moves=d.tick(0)
    assert len(moves)==2
    assert moves[0].destination=='L' and moves[1].destination=='Q'
    assert d._blockers('L2',moves[1].vehicle_id)=={moves[0].vehicle_id}


def test_idle_reposition_preserves_berth_until_actual_departure():
    f,d=setup({'idle':'U','busy':'A'})
    d.submit(TransportWork('t','L','U'))
    moves=d.tick(0)
    reposition=next(m for m in moves if m.purpose=='REPOSITION')
    assert reposition.vehicle_id=='idle' and d.holders['U']=='idle'
    assert d.resource_snapshot('idle')=={'berth:U','berth:'+reposition.destination}
    arrive(d,reposition,0,1)
    assert 'U' not in d.holders and f.vehicles['idle'].phase==FleetPhase.IDLE



def test_idle_vehicle_does_not_reposition_from_unneeded_berth():
    f,d=setup({'idle':'U','busy':'A'})
    d.network.services={**d.network.services,'U2':dict(kind='unloading',capacity=1)}
    d.submit(TransportWork('t','L','U2'))
    moves=d.tick(0)
    assert len(moves)==1 and moves[0].vehicle_id=='busy'
    assert d.holders['U']=='idle' and f.vehicles['idle'].phase==FleetPhase.IDLE


def test_idle_reposition_clears_an_intermediate_access_blocker():
    f,d=setup({'idle':'U','busy':'A'})
    d.network.services={**d.network.services,'U2':dict(kind='unloading',capacity=1)}
    original=d.network.route
    def route(a,b,*,occupied_nodes=frozenset()):
        if b=='U2' and 'U' in occupied_nodes:return None
        return original(a,b,occupied_nodes=occupied_nodes)
    d.network.route=route
    d.submit(TransportWork('t','L','U2'))
    pickup=d.tick(0)[0];arrive(d,pickup,0,1);d.tick(1)
    moves=d.tick(2)
    assert any(m.purpose=='REPOSITION' and m.vehicle_id=='idle' for m in moves)
    assert d.holders['U']=='idle'


def test_unassigned_task_can_clear_idle_intermediate_blocker():
    f,d=setup({'idle':'U','candidate':'A'})
    d.network.services={**d.network.services,'U2':dict(kind='unloading',capacity=1)}
    original=d.network.route
    def route(a,b,*,occupied_nodes=frozenset()):
        if b=='L' and (a=='U' or 'U' in occupied_nodes):return None
        return original(a,b,occupied_nodes=occupied_nodes)
    d.network.route=route
    d.submit(TransportWork('t','L','U2'))
    moves=d.tick(0)
    assert d.status['t']==WorkStatus.PENDING
    assert len(moves)==1 and moves[0].purpose=='REPOSITION' and moves[0].vehicle_id=='idle'
    arrive(d,moves[0],0,1)
    assert any(m.task_id=='t' for m in d.tick(1))


def test_completed_vehicle_parks_before_run_can_finish():
    f,d=setup({'car':'L'})
    d.park_idle_vehicles=True
    d.submit(TransportWork('last','L','U'))
    assert not d.tick(0)
    haul=d.tick(1)[0];arrive(d,haul,1,2)
    assert not d.tick(2)
    park=d.tick(3)[0]
    assert d.status['last']==WorkStatus.COMPLETE
    assert park.purpose=='REPOSITION' and park.waiting
    assert park.destination in ('Q','A','B')
    assert not d.ready_to_finish()
    assert d.holders['U']=='car'  # Body/source lease retained until actual exit.
    arrive(d,park,3,4)
    assert d.ready_to_finish()
    assert not d.tick(4)


def test_no_free_parking_never_reports_transport_run_finished():
    f,d=setup({'car':'U','q':'Q','a':'A','b':'B'})
    d.park_idle_vehicles=True
    assert not d.tick(0)
    assert not d.ready_to_finish()
    assert d.positions['car']=='U' and d.holders['U']=='car'


def admission_setup():
    from logistics_park.scheduling.task_dispatch import Assignment
    f,d=setup({'far':'B','near':'A'})
    d.follow_to_busy_destination=True;d.offroad_busy_wait=True
    for name in ('far','near'):
        work=TransportWork('work-'+name,'L','U');d.submit(work)
        visit=f.vehicles[name].visit_id;f.assign_task(name,visit,work.task_id,0)
        d.status[work.task_id]=WorkStatus.ACTIVE
        d.assignments[name]=Assignment(work,name,visit,state='WAITING',queued_at=0.)
    return d


def test_wait_age_can_override_a_small_eta_advantage_and_depart_consistently():
    d=admission_setup();d._time(60)
    d.assignments['near'].queued_at=60
    d.holders['L']='near'
    d.promote_approaches({'far':(25.,False),'near':(23.,False)})
    assert d.holders['L']=='far'
    moves=d.tick(60)
    assert [(m.vehicle_id,m.destination) for m in moves]==[('far','L')]


def test_same_lane_leader_wins_even_when_follower_has_better_eta():
    d=admission_setup();d.holders['L']='near'
    d.promote_approaches({'far':(40.,False,frozenset()),'near':(10.,False,frozenset({'far'}))})
    assert d.holders['L']=='far'
    assert [m.vehicle_id for m in d.tick(0)]==['far']


def test_missing_remote_owner_priority_does_not_make_its_eta_zero():
    d=admission_setup();d.holders['L']='far'
    d.promote_approaches({'near':(1.,False)})
    assert d.holders['L']=='near'


def test_committed_final_approach_cannot_be_preempted_by_earlier_eta():
    d=admission_setup();d.holders['L']='far'
    d.promote_approaches({'far':(40.,True),'near':(1.,False)})
    assert d.holders['L']=='far'


def test_subsecond_eta_fluctuations_do_not_churn_the_remote_promise():
    d=admission_setup();d.holders['L']='far'
    d.promote_approaches({'far':(10.,False),'near':(9.8,False)})
    assert d.holders['L']=='far'
    d.promote_approaches({'far':(10.,False),'near':(9.,False)})
    assert d.holders['L']=='near'


def test_exit_guard_withdraws_unentered_promise_but_retains_actual_occupancy():
    from logistics_park.scheduling.task_dispatch import Assignment
    f,d=setup({'near':'A'})
    d.follow_to_busy_destination=True;d.offroad_busy_wait=True
    work=TransportWork('work','L','U');visit=f.vehicles['near'].visit_id
    d.assignments['near']=Assignment(work,'near',visit,state='WAITING',queued_at=0.)
    d.berth_exit_blockers=lambda target,vehicle:{'departing'} if target=='L' else set()
    d.holders['L']='near'
    d.promote_approaches({'near':(1.,False)})
    assert 'L' not in d.holders and d.holders['A']=='near'
    assert d._blockers('L','near')=={'departing'}
    d.holders['L']='near';d.positions['near']='L'
    d.promote_approaches({'near':(1.,True)})
    assert d.holders['L']=='near'
