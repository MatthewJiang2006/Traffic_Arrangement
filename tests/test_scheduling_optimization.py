"""Scheduling decisions compared with exhaustive matching and lease invariants."""
import itertools,math,random
from types import SimpleNamespace
import pytest
from logistics_park.scheduling.optimization import bottleneck_assignment,SchedulingOptions
from logistics_park.scheduling.task_dispatch import TaskDispatcher,TransportWork,Assignment,WorkStatus
from logistics_park.scheduling.fleet_lifecycle import FleetRegistry
from logistics_park.scheduling.travel_time import TravelTimeEstimator


def test_bottleneck_matching_agrees_with_exhaustive_oracle():
    rng=random.Random(19)
    for _ in range(70):
        n,m=rng.randint(1,4),rng.randint(1,4)
        costs=[[math.inf if rng.random()<.25 else rng.randint(0,20) for _ in range(m)] for _ in range(n)]
        def score(pairs):
            values=[costs[i][j] for i,j in pairs]
            return -len(pairs),max(values,default=0),sum(values)
        possible=[]
        for assignment in itertools.product(range(-1,m),repeat=n):
            cols=[j for j in assignment if j>=0]
            if len(cols)!=len(set(cols)):continue
            pairs=[(i,j) for i,j in enumerate(assignment) if j>=0]
            if all(math.isfinite(costs[i][j]) for i,j in pairs):possible.append(score(pairs))
        result=bottleneck_assignment(costs)
        assert score(result)==min(possible)


class Network:
    services={n:dict(kind=k,capacity=1) for n,k in [('A','waiting'),('B','waiting'),('Q1','waiting'),('Q2','waiting'),
        ('L0','loading'),('L1','loading'),('L2','loading'),('U0','unloading'),('U1','unloading'),('U2','unloading')]}
    times={('A','L0'):1,('A','L1'):6,('B','L0'):5,('B','L1'):9,
        ('L0','U0'):1,('L1','U1'):1,('L2','U2'):1,('U0','Q1'):1,('U0','Q2'):10,('U1','Q1'):1,('U1','Q2'):90}
    def route(self,a,b,*,occupied_nodes=frozenset(),**kwargs):
        if b in occupied_nodes or (a,b) not in self.times:return None
        t=self.times[a,b]
        return dict(edge_ids=[a+'-'+b],distance_m=t*5,estimated_travel_time_s=t)
    def queue_options(self,*args):return []


def dispatcher(positions,options):
    f=FleetRegistry();d=TaskDispatcher(Network(),f,service_s=1,scheduling_options=options)
    for car,node in positions.items():
        v=f.adopt_present_vehicle(car,0,fully_inside=True,resources=[])
        d.place_vehicle(car,v.visit_id,node,0,stopped=True)
    return d


def test_batch_avoids_greedy_tail_and_keeps_source_leases():
    d=dispatcher({'a':'A','b':'B'},{'batch_dispatch':True})
    d.submit(TransportWork('first','L0','U0'));d.submit(TransportWork('second','L1','U1'))
    moves=d.tick(0)
    assert {(m.vehicle_id,m.destination) for m in moves}=={('a','L1'),('b','L0')}
    assert d.holders['A']=='a' and d.holders['B']=='b'
    assert d.positions=={'a':'A','b':'B'}


def test_unreachable_front_task_does_not_starve_reachable_task():
    d=dispatcher({'a':'A'},{'batch_dispatch':True})
    d.submit(TransportWork('old','L2','U2'));d.submit(TransportWork('new','L1','U1'))
    assert d.tick(0)[0].task_id=='new'
    assert d.status['old']==WorkStatus.PENDING


def test_equal_release_uses_submission_order_not_lexical_identifier():
    d=dispatcher({'a':'A'},{'batch_dispatch':True})
    d.submit(TransportWork('z-first','L1','U1'));d.submit(TransportWork('a-later','L0','U0'))
    assert d.tick(0)[0].task_id=='z-first'


@pytest.mark.parametrize('batch',[False,True])
def test_loading_inflight_limit_leaves_extra_work_pending(batch):
    d=dispatcher({'a':'A','b':'B'},{'batch_dispatch':batch,'max_loading_inflight':1})
    d.submit(TransportWork('one','L0','U0'));d.submit(TransportWork('two','L0','U0'))
    assert len(d.tick(0))==1 and len(d.assignments)==1
    assert d.status['two']==WorkStatus.PENDING


def test_eta_learns_queue_separately_and_does_not_learn_detour_as_slow_physics():
    t=TravelTimeEstimator();route=dict(estimated_travel_time_s=10,freeflow_travel_time_s=10,congestion_delay_s=2,edge_ids=[])
    t.record_leg('L0',10,50,30,unchanged_route=True)
    estimate=t.estimate_route(Network(),route,'A','L0')
    assert estimate['moving_s']==12.5 and estimate['queue_s']==7.5
    before=estimate.copy()
    t.record_leg('L0',10,900,700,unchanged_route=False)
    assert t.estimate_route(Network(),route,'A','L0')==before
    assert t.leg_samples['L0']==1


def test_calibrated_pickup_accounts_for_busy_source_and_inflight_loading():
    d=dispatcher({'a':'A','b':'L0'},{'calibrated_eta':True})
    work=TransportWork('one','L0','U0');d.submit(work)
    d.assignments['b']=Assignment(work,'b',d.fleet.vehicles['b'].visit_id,state='SERVICING',service_until=20)
    route=d._route('A','L0','a',future=True)
    d.route_eta_predictor=lambda *args:dict(total_s=10.)
    assert d._pickup_eta(work,'a',route)==40.
    assert d._pickup_eta(work,'b',dict(estimated_travel_time_s=0))==0.
    assert d.holders['L0']=='b'


def test_parking_lookahead_preserves_close_space_without_issuing_virtual_leases():
    d=dispatcher({'idle':'U0','active':'B'},{'parking_lookahead':True})
    work=TransportWork('active-work','L1','U1');d.submit(work)
    visit=d.fleet.vehicles['active'].visit_id;d.fleet.assign_task('active',visit,work.task_id,0)
    d.assignments['active']=Assignment(work,'active',visit,state='MOVING')
    d._admission_priority=lambda *args:(500.,False)
    before=dict(d.holders)
    assert d._parking_plan()=={'idle':'Q2'}
    assert d.holders==before and 'Q1' not in d.holders and 'Q2' not in d.holders


@pytest.mark.parametrize('options',[{'batch_dispatch':1},{'max_loading_inflight':-1},{'decision_interval_s':float('nan')}])
def test_invalid_scheduling_options_are_rejected(options):
    with pytest.raises(ValueError):SchedulingOptions(**options)
