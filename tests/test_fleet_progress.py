from logistics_park.common.messages import VehicleState
from logistics_park.runtime.fleet_progress import FleetProgressWatch
from logistics_park.runtime.fleet_progress import LocalProgressWatch

def test_wheel_spin_does_not_hide_a_stalled_fleet():
    watch=FleetProgressWatch(15,.25);states={'a':VehicleState('a',0,0,0,speed=8)}
    assert not watch.observe(states,0,('task',))['stalled']
    assert watch.observe(states,15,('task',))['stalled']

def test_actual_motion_and_service_completion_reset_the_timer():
    watch=FleetProgressWatch(15,.25);state=VehicleState('a',0,0,0);states={'a':state}
    watch.observe(states,0,0);state.x=.3
    assert not watch.observe(states,14,0)['stalled']
    assert not watch.observe(states,28,1)['stalled']
    assert watch.observe(states,43,1)['stalled']

def test_planned_service_is_not_mistaken_for_deadlock():
    watch=FleetProgressWatch(15,.25);states={'a':VehicleState('a',0,0,0)}
    watch.observe(states,0,0)
    assert not watch.observe(states,20,0,scheduled_activity_until=30)['stalled']
    assert not watch.observe(states,30,1)['stalled']
    assert watch.observe(states,45,1)['stalled']


def test_local_cycle_is_detected_while_an_unrelated_truck_keeps_moving():
    watch=LocalProgressWatch(20,90,.25)
    states={n:VehicleState(n,0,0,0) for n in ['a','b','moving']}
    assert not watch.observe(states,0,[['a','b']],['a','b'])['stalled']
    states['moving'].x=100
    result=watch.observe(states,20,[['a','b']],['a','b'])
    assert result['stalled'] and result['cycles']==[['a','b']]


def test_local_watch_resets_for_actual_clearance_and_ignores_unblocked_parked_trucks():
    watch=LocalProgressWatch(20,90,.25)
    states={n:VehicleState(n,0,0,0) for n in ['a','b','parked']}
    watch.observe(states,0,[['a','b']],['a','b'])
    states['a'].x=.3
    assert not watch.observe(states,19,[['a','b']],['a','b'])['stalled']
    assert not watch.observe(states,30,[],[])['stalled']
    assert not watch.observe(states,300,[],[])['stalled']


def test_waiting_queue_retains_age_while_its_actual_downstream_chain_progresses():
    watch=LocalProgressWatch(20,90,.25)
    states={n:VehicleState(n,0,0,0) for n in ('queue','dock','downstream','unrelated')}
    deps=[dict(vehicle='queue',blocked_by='dock'),dict(vehicle='dock',blocked_by='downstream')]
    watch.observe(states,0,[],['queue'],dependencies=deps)
    states['downstream'].x=.3
    result=watch.observe(states,90,[],['queue'],dependencies=deps)
    assert not result['stalled'] and result['inactive_s']['queue']==90
    assert result['blocking_chain_inactive_s']['queue']==0
    states['unrelated'].x=100
    assert watch.observe(states,180,[],['queue'],dependencies=deps)['stalled']


def test_dependency_speed_without_displacement_cannot_hide_stagnation():
    watch=LocalProgressWatch(20,90,.25)
    states={n:VehicleState(n,0,0,0,speed=8) for n in ('a','b')}
    deps=[dict(vehicle='a',blocked_by='b')]
    watch.observe(states,0,[],['a'],dependencies=deps)
    assert watch.observe(states,90,[],['a'],dependencies=deps)['stalled']


def test_stationary_cycle_still_stops_even_with_a_moving_downstream_dependency():
    watch=LocalProgressWatch(20,90,.25)
    states={n:VehicleState(n,0,0,0) for n in ('a','b','c')}
    deps=[dict(vehicle='a',blocked_by='b'),dict(vehicle='b',blocked_by='a'),dict(vehicle='b',blocked_by='c')]
    watch.observe(states,0,[['a','b']],['a','b'],dependencies=deps)
    states['c'].x=.3
    assert watch.observe(states,20,[['a','b']],['a','b'],dependencies=deps)['cycles']==[['a','b']]


def test_small_actual_dock_departure_keeps_its_upstream_queue_alive():
    watch=LocalProgressWatch(20,90,.25)
    states={n:VehicleState(n,0,0,0) for n in ('queue','dock')}
    deps=[dict(vehicle='queue',blocked_by='dock')]
    watch.observe(states,0,[],['queue'],dependencies=deps)
    states['dock'].x=.134;states['dock'].speed=.36
    result=watch.observe(states,90,[],['queue'],dependencies=deps)
    assert not result['stalled']
    assert result['inactive_s']['queue']==90
    assert result['blocking_chain_inactive_s']['queue']==0
    # A departure which stops again must still be diagnosed.
    states['dock'].speed=0
    assert watch.observe(states,180,[],['queue'],dependencies=deps)['stalled']


def test_tiny_pose_jitter_or_stationary_drift_does_not_extend_local_wait():
    for displacement,speed in ((.01,8),(.134,0)):
        watch=LocalProgressWatch(20,90,.25)
        states={n:VehicleState(n,0,0,0) for n in ('queue','dock')}
        deps=[dict(vehicle='queue',blocked_by='dock')]
        watch.observe(states,0,[],['queue'],dependencies=deps)
        states['dock'].x=displacement;states['dock'].speed=speed
        assert watch.observe(states,90,[],['queue'],dependencies=deps)['stalled']


def test_departure_below_anchor_threshold_gets_only_a_bounded_startup_grace():
    watch=LocalProgressWatch(20,90,.25)
    states={n:VehicleState(n,0,0,0) for n in ('queue','dock')}
    deps=[dict(vehicle='queue',blocked_by='dock')]
    watch.observe(states,0,[],['queue'],dependencies=deps)
    watch.observe(states,89,[],['queue'],dependencies=deps)
    states['dock'].x=.04;states['dock'].speed=.18
    result=watch.observe(states,90,[],['queue'],dependencies=deps)
    assert not result['stalled'] and result['blocking_chain_inactive_s']['queue']==90
    states['dock'].speed=0
    assert not watch.observe(states,91,[],['queue'],dependencies=deps)['stalled']
    assert watch.observe(states,92,[],['queue'],dependencies=deps)['stalled']


def test_resolved_dependency_retains_its_observed_progress_but_not_future_unrelated_motion():
    watch=LocalProgressWatch(20,90,.25)
    states={n:VehicleState(n,0,0,0) for n in ('queue','dock','crossing')}
    deps=[dict(vehicle='queue',blocked_by='dock'),dict(vehicle='dock',blocked_by='crossing')]
    watch.observe(states,0,[],['queue','dock'],dependencies=deps)
    states['crossing'].x=.3
    watch.observe(states,89,[],['queue','dock'],dependencies=deps)
    # The dock's permit was granted: its old crossing dependency disappeared,
    # but it still needs time to physically start, so the queue stays blocked.
    remaining=deps[:1]
    result=watch.observe(states,90,[],['queue'],dependencies=remaining)
    assert not result['stalled'] and result['blocking_chain_inactive_s']['queue']==1
    states['crossing'].x=100
    assert watch.observe(states,179,[],['queue'],dependencies=remaining)['stalled']
