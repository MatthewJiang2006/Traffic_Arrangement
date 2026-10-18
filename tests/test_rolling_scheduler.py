import math
from types import SimpleNamespace as NS
import pytest
from logistics_park.scheduling.resource_calendar import ResourceCalendar
from logistics_park.scheduling.traffic_forecast import TrafficForecast,OccupancyInterval
from logistics_park.scheduling.global_scheduler import RollingScheduler
from logistics_park.scheduling.task_dispatch import TransportWork,WorkStatus
from logistics_park.scheduling.optimization import SchedulingOptions
from test_scheduling_optimization import dispatcher


def test_calendar_joint_conflicts_require_both_resources_clear():
    c=ResourceCalendar();c.reserve(['exit'],1,9,'one');c.reserve(['dock'],8,12,'two')
    assert c.earliest(['exit','dock'],0,5)==20
    assert c.earliest(['exit'],10,2)==10
    assert c.earliest(['exit'],0,5,ignore='one')==0
    copy=c.copy();copy.reserve(['exit'],20,5,'third')
    assert len(c.intervals['exit'])==1


def test_capacity_counts_distinct_vehicles_not_duplicate_route_slices():
    c=ResourceCalendar({'road':2});c.reserve(['road'],0,10,'one');c.reserve(['road'],1,9,'one')
    assert c.earliest(['road'],2,5)==2
    c.reserve(['road'],0,10,'two')
    assert c.earliest(['road'],2,5)==10


def test_whole_route_shift_rechecks_prefix_without_waiting_inside_junction():
    c=ResourceCalendar();c.reserve(['junction'],8,10,'other');c.reserve(['exit'],18,10,'third')
    spans=((0.,('exit',),5.),(5.,('junction',),5.))
    departure=c.trajectory(spans,0.,'ego',latest_start=60.)
    assert departure==13.
    assert c.intervals['exit'][-1].start==13.
    assert c.intervals['junction'][-1].start==18.


@pytest.mark.parametrize('start,duration',[(math.nan,1),(0,-1),(0,math.inf),(-1,2)])
def test_invalid_calendar_inputs_rejected(start,duration):
    with pytest.raises(ValueError):ResourceCalendar().reserve(['dock'],start,duration,'one')


def test_forecast_keeps_different_arrival_times_and_deduplicates_alias_slices():
    f=TrafficForecast();f.now=0
    f.intervals=[OccupancyInterval('a','road',0,10),OccupancyInterval('a','road',5,15),
                 OccupancyInterval('b','road',20,30)]
    assert f.occupancy('road',7)['count']==1
    assert f.occupancy('road',25)['count']==1
    assert f.occupancy('road',16)['count']==0
    assert f.occupancy('road',7,exclude='a')['count']==0
    assert f.occupancy('road',61) is None


def test_forecast_blocked_road_has_unknown_departure_and_no_freeflow_exit():
    f=TrafficForecast();edge=dict(kind='road',capacity_alias='R',speed_limit=8.)
    model=NS(edges={'e':edge},raw={'physical_capacities':{'R':2}})
    plan=NS(model=model,edges=['e'],edge_ends=[8.],edge_starts=[0.],edge_index=lambda s:0)
    f.update({'a':NS(speed=0.)},{'a':NS(plan=plan,progress=0.,finished=False)},0.,blocked_vehicles={'a'})
    assert f.occupancy('R',30)=={'count':1,'unknown_count':1}
    f.update({'a':NS(speed=2.)},{'a':NS(plan=plan,progress=0.,finished=False)},1.)
    assert f.occupancy('R',30)=={'count':0,'unknown_count':0}
    with pytest.raises(ValueError):f.update({}, {}, .5)


def test_forecast_penalty_uses_capacity_and_excludes_own_demand():
    f=TrafficForecast();f.now=0.;f.capacities={'R':2}
    f.intervals=[OccupancyInterval('a','R',0,30),OccupancyInterval('b','R',0,30)]
    n=NS(edges={'e':dict(kind='road',capacity_alias='R',estimated_travel_time_s=8.)})
    assert f.route_delay(n,{'edge_ids':['e']},'a',0.)==pytest.approx(1.)


def test_rolling_plan_is_advisory_and_does_not_book_real_nodes():
    d=dispatcher({'a':'A','b':'B'},{'rolling_scheduler':True,'planning_budget_s':1.})
    d.submit(TransportWork('one','L0','U0'));d.submit(TransportWork('two','L1','U1'))
    before=dict(d.holders);pairs=d.global_scheduler.plan(d,list(d.tasks.values()))
    assert pairs and len({p[1] for p in pairs})==len(pairs)
    assert d.holders==before and not d.assignments and not d.moves
    assert all(s==WorkStatus.PENDING for s in d.status.values())
    assert d.global_scheduler.last_plan['physical_leases_changed'] is False


def test_rolling_shared_dock_defers_departure_without_claiming_future_lease():
    d=dispatcher({'a':'A','b':'B'},{'rolling_scheduler':True,'planning_budget_s':1.})
    d.submit(TransportWork('one','L0','U0'));d.submit(TransportWork('two','L0','U0'))
    moves=d.tick(0.)
    assert len(moves)==1 and len(d.assignments)==1
    assert sum(s==WorkStatus.PENDING for s in d.status.values())==1
    assert d.global_scheduler.last_plan['deferred_tasks']
    assert d.positions=={'a':'A','b':'B'} and d.holders['A']=='a' and d.holders['B']=='b'


def test_rolling_timeout_falls_back_to_existing_dispatch():
    d=dispatcher({'a':'A'},{'rolling_scheduler':True,'planning_budget_s':1e-12})
    d.submit(TransportWork('one','L0','U0'))
    assert d.tick(0.)[0].task_id=='one'
    assert d.global_scheduler.last_plan['status']=='BUDGET_EXCEEDED'


def test_unreachable_high_priority_does_not_starve_feasible_lower_task():
    d=dispatcher({'a':'A'},{'rolling_scheduler':True,'planning_budget_s':1.})
    d.submit(TransportWork('blocked','L2','U2',priority=5));d.submit(TransportWork('ready','L0','U0'))
    assert d.tick(0.)[0].task_id=='ready'
    assert d.status['blocked']==WorkStatus.PENDING


def test_source_wait_advice_never_releases_holder_and_has_fixed_twenty_second_budget():
    d=dispatcher({'a':'A'},{'rolling_scheduler':True,'planning_budget_s':1.})
    work=TransportWork('one','L0','U0');d.submit(work)
    from logistics_park.scheduling.task_dispatch import Assignment
    visit=d.fleet.vehicles['a'].visit_id
    d.fleet.assign_task('a',visit,work.task_id,0.)
    a=Assignment(work,'a',visit);d.assignments['a']=a;d.status['one']=WorkStatus.ACTIVE
    d.departure_not_before={'a':('one','LOADING','A',50.)};d.departure_plan_at=0.
    route=d._route('A','L0','a');before=dict(d.holders)
    assert d._checked_issue(a,'L0',route,False)==[] and d.holders==before and not d.moves
    d.now=19.;d.departure_plan_at=19.
    assert d._checked_issue(a,'L0',route,False)==[]
    d.now=20.;d.departure_plan_at=20.
    move=d._checked_issue(a,'L0',route,False)[0]
    assert move.destination=='L0' and d.holders['A']=='a'


def test_source_wait_cannot_bypass_actual_departure_rejection_after_expiry():
    d=dispatcher({'a':'A'},{'rolling_scheduler':True,'planning_budget_s':1.})
    work=TransportWork('one','L0','U0');d.submit(work)
    from logistics_park.scheduling.task_dispatch import Assignment
    a=Assignment(work,'a',d.fleet.vehicles['a'].visit_id)
    d.departure_guard=lambda *args:False
    d.departure_not_before={'a':('one','LOADING','A',50.)};d.departure_plan_at=0.
    route=d._route('A','L0','a');assert not d._checked_issue(a,'L0',route,False)
    d.now=20.;d.departure_plan_at=20.
    assert not d._checked_issue(a,'L0',route,False) and 'a' not in d.moves


def test_rolling_parking_accounts_for_late_transport_finish_without_physical_leases():
    from logistics_park.scheduling.task_dispatch import Assignment
    d=dispatcher({'idle':'U2','late':'A','early':'B'},
                 {'rolling_scheduler':True,'parking_lookahead':True,'parking_tail_reserve':True})
    d.network.services={k:dict(v) for k,v in d.network.services.items()};d.network.times=dict(d.network.times)
    d.network.services['Q3']={'kind':'waiting','capacity':1}
    d.network.times.update({('U2','Q3'):5,('U0','Q1'):20,('U0','Q2'):30,('U1','Q1'):1,('U1','Q2'):100})
    for name,target in [('late','U0'),('early','U1')]:
        work=TransportWork(name,'L0',target);d.submit(work);visit=d.fleet.vehicles[name].visit_id
        d.fleet.assign_task(name,visit,name,0);d.status[name]=WorkStatus.ACTIVE
        d.assignments[name]=Assignment(work,name,visit,operation='UNLOADING',state='MOVING')
    d._admission_priority=lambda a,*args:(100. if a.vehicle_id=='late' else 0.,False)
    before=dict(d.holders)
    assert d._parking_plan()=={'idle':'Q3'}
    assert d.parking_future_targets=={'late':'Q1','early':'Q2'}
    assert d.holders==before and not d.moves


@pytest.mark.parametrize('physical_owner',[False,True])
def test_actual_blocked_source_releases_only_future_promise_without_needing_inbound_move(physical_owner):
    from logistics_park.scheduling.task_dispatch import Assignment
    from logistics_park.scheduling.efficiency_policy import EfficiencyPolicy
    d=dispatcher({'a':'L0' if physical_owner else 'A','b':'B'},{'rolling_scheduler':True,'planning_budget_s':1.})
    d.follow_to_busy_destination=True;d.offroad_busy_wait=True
    d.efficiency_policy=EfficiencyPolicy(proactive_admission=True)
    for name in ['a','b']:
        work=TransportWork(name,'L0','U0');d.submit(work)
        d.assignments[name]=Assignment(work,name,d.fleet.vehicles[name].visit_id)
    d.holders['L0']='a'
    d.actual_departure_blockers['a','L0']=dict(time=0.,source=d.positions['a'],blocked_by=['b'])
    d.departure_rejected_at['a','L0']=0.
    before=d.positions.copy();d.promote_approaches({'a':(.5,False),'b':(5.,False)})
    assert d.holders['L0']==('a' if physical_owner else 'b')
    assert d.positions==before and not d.moves
    if not physical_owner:assert d.holders['A']=='a'


@pytest.mark.parametrize('options',[{'traffic_forecast':1},{'rolling_scheduler':'yes'},
                                 {'planning_budget_s':0},{'planning_interval_s':math.inf}])
def test_planning_options_validated(options):
    with pytest.raises(ValueError):SchedulingOptions(**options)
