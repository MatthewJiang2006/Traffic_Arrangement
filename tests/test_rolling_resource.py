import pytest
from logistics_park.scheduling.rolling_resource import RollingResourceReservations

def test_joint_admission_never_holds_half_of_a_blocked_bundle():
    r=RollingResourceReservations(['a','b'])
    assert r.request('b','owner',0)
    assert not r.request_many(['a','b'],'truck',1,predicted_s=10)
    assert 'a' not in r.owners
    r.release('b','owner',2,tail_clear=True)
    assert 'a' not in r.owners and 'b' not in r.owners
    r.advance(2)
    assert r.owners=={'a':'truck','b':'truck'}

def test_blocked_exit_is_not_granted_and_does_not_block_safe_other_request():
    r=RollingResourceReservations(['a']);blocked={'first'}
    r.guard=lambda request:request.vehicle_id not in blocked
    assert not r.request('a','first',0)
    assert r.request('a','second',1)
    r.release('a','second',2,tail_clear=True)
    blocked.clear();r.advance(2)
    assert r.owners['a']=='first'

def test_wait_aging_eventually_beats_shorter_duration():
    r=RollingResourceReservations(['a']);r.guard=lambda request:False
    assert not r.request_many(['a'],'long',0,predicted_s=20)
    assert not r.request_many(['a'],'short',50,predicted_s=1)
    r.guard=lambda request:True;r.advance(50)
    assert r.owners['a']=='long'

def test_expired_forecast_does_not_release_a_physically_occupied_region():
    r=RollingResourceReservations(['a'])
    assert r.request('a','inside',0)
    assert not r.request('a','next',100)
    assert r.windows['a']['predicted_end']==1
    r.advance(100)
    assert r.owners['a']=='inside'
    assert not r.release('a','inside',100,tail_clear=False)
    assert r.owners['a']=='inside'
    r.release('a','inside',101,tail_clear=True);r.advance(101)
    assert r.owners['a']=='next'

def test_disjoint_safe_corridors_can_be_admitted_in_parallel():
    r=RollingResourceReservations(['a','b','c','d'])
    assert r.request_many(['a','b'],'one',0,predicted_s=10)
    assert r.request_many(['c','d'],'two',0,predicted_s=10)
    assert len(r.owners)==4

@pytest.mark.parametrize('duration',[0,-1,float('nan')])
def test_invalid_duration_rejected_before_reserving(duration):
    r=RollingResourceReservations(['a'])
    with pytest.raises(ValueError):r.request_many(['a'],'one',0,predicted_s=duration)
    assert not r.owners and not r.pending

def test_committed_route_is_protected_from_new_waiting_occupant():
    from simulate_ue_continuous_tasks import setup
    e,states=setup(positions={'Car_0_0':'W1-loading','Car_0_1':'W2-loading'},tasks=[('W1-loading','W3-unloading')])
    d=e.dispatch;d.protect_committed_routes=True
    from logistics_park.scheduling.task_dispatch import DispatchMove
    d.moves['Car_0_0']=DispatchMove(1,'task','Car_0_0','visit','W1-loading','W3-unloading',dict(must_be_empty_nodes=['Q4'],required_resources=[]),False)
    assert d._blocks_committed_route('Q4','Car_0_1')
    assert not d._blocks_committed_route('Q4','Car_0_0')


def test_wait_graph_separates_cycles_from_simple_queues():
    from logistics_park.scheduling.wait_dependencies import strongly_connected_cycles
    rows=[dict(vehicle=a,blocked_by=b) for a,b in [('a','b'),('b','a'),('c','a'),('d','e')]]
    assert strongly_connected_cycles(rows)==[['a','b']]
    assert strongly_connected_cycles([])==[]

def test_existing_owner_replaces_obsolete_pending_bundle():
    r=RollingResourceReservations(['a','b']);r.request('a','one',0);r.request('b','two',0)
    assert not r.request_many(['a','b'],'one',1,predicted_s=10)
    assert r.request('a','one',2)
    assert 'one' not in r.pending
    r.release('b','two',3,tail_clear=True);r.advance(3)
    assert 'b' not in r.owners
