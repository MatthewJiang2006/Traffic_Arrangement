"""Boundary sequences and recorded failures for the shared transition contract."""
import itertools
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from logistics_park.common.messages import VehicleState
from logistics_park.runtime.transport_control import TransportController, envelope
from logistics_park.scheduling.local_traffic import LocalTraffic
from logistics_park.scheduling.traffic_transition import (
    TrafficTransition, TransitionValidator, holding_line)
from replay_traffic_transitions import replay

ROOT = Path(__file__).resolve().parents[1]


def line_plan(action_entry=30., with_action=True):
    edges = ('approach', 'junction', 'exit') if with_action else ('road',)
    model = SimpleNamespace(edges={k: dict(kind='junction' if k == 'junction' else 'road',
        capacity_alias=k, reverse=False) for k in edges}, aliases={k: k for k in edges}, raw={})
    return SimpleNamespace(model=model, edges=edges, edge_starts=(0., action_entry, action_entry + 5.) if with_action else (0.,),
        edge_ends=(action_entry, action_entry + 5., 100.) if with_action else (100.,), length=100.,
        point=lambda s: (s, 0.), heading=lambda s: 0., reverse=lambda s: False,
        edge_index=lambda s: 0 if not with_action or s < action_entry else (1 if s < action_entry + 5. else 2),
        speed_cap=lambda s: 8., checkpoints=[], stops=['destination'])


def fleet(names=('a',), progress=0., speed=0.):
    local = LocalTraffic()
    executions = {n: SimpleNamespace(plan=line_plan(), progress=progress, checkpoint=0, finished=False)
                  for n in names}
    states = {n: VehicleState(n, progress, 0., 0., speed=speed) for n in names}
    local._executions = executions
    return local, executions, states


def lease(local, ex, name='a', created=0.):
    right = dict(plan=ex[name].plan, end=35., created_at=created, sweep=[])
    local.passages[name, 1] = right
    return right


def validator(local, ex, states):
    return TransitionValidator(local, ex, states, {}, envelope)


def test_recorded_failures_do_not_reenter_the_unsafe_states():
    result = replay(json.loads((ROOT / 'tests/fixtures/ue_transition_failures_20261020.json').read_text()))
    assert result['status'] == 'PASS'
    assert result['round04']['decision']['reason'] == 'route_clearance_not_executable'
    assert result['round07']['decisions'][0]['reason'] == 'passage_stop_line_committed'


@pytest.mark.parametrize('kind', ['timeout_revocation', 'passage_admission', 'cycle_revocation'])
@pytest.mark.parametrize('offset', [-1.8, -.7, -.01, 0., .01, .574, .576, 1., 8.])
@pytest.mark.parametrize('speed', [-1e-7, 0., .079, 1., 4.])
def test_all_revocation_origins_obey_the_same_physical_boundary(kind, offset, speed):
    local, ex, states = fleet(progress=22.2-offset, speed=speed)
    right = lease(local, ex)
    proposal = TrafficTransition(kind, 'a', revoke=(('a', 1),))
    # Independent continuous braking bound + reaction and one control-step allowance.
    required = abs(speed)*1.2 + speed*speed/1.2 + .3*.5*.5
    expected = offset+1e-9 >= required
    published = []
    result = local.commit_transition(proposal, states, {}, envelope, lambda: published.append(True))
    assert result.allowed == expected
    assert bool(published) == expected
    assert local.passages['a', 1] is right
    assert local.transition_revision == int(expected)


@pytest.mark.parametrize('age', [7.999, 8., 8.001, 20.])
@pytest.mark.parametrize('progress', [20., 21.624, 21.626, 22.2, 22.21, 23.99])
def test_timeout_boundary_sequence_never_revokes_a_committed_permission(age, progress):
    local, ex, states = fleet(progress=progress)
    right = lease(local, ex)
    local.prepare(states, ex, {}, envelope, age)
    expected_release = age >= 8. and 22.2-progress >= .075-1e-6
    assert (('a', 1) not in local.passages) == expected_release
    assert (('a', 1) in local.retry_after) == expected_release
    if not expected_release:
        assert local.passages['a', 1] is right
    # Repeated timeout checks must not erode or steal the same committed right.
    local.prepare(states, ex, {}, envelope, age+.01)
    expected_second_release = age+.01 >= 8. and 22.2-progress >= .075-1e-6
    assert (('a', 1) not in local.passages) == expected_second_release


@pytest.mark.parametrize('moving', [False, True])
@pytest.mark.parametrize('kind', ['junction', 'uturn', 'lane_change', 'service_access'])
def test_stopped_and_moving_diversion_cannot_introduce_a_stop_line_behind_the_body(moving, kind):
    local, ex, states = fleet(progress=3.)
    old = ex['a'].plan = line_plan(with_action=False)
    new = line_plan(action_entry=10.)
    new.model.edges['junction']['kind'] = kind
    controller = TransportController.__new__(TransportController)
    controller.local_traffic = local
    controller.executions = ex
    controller.plans = {'a': old}
    controller.model = old.model
    controller.capacity = SimpleNamespace(committed={'road': {'a'}})
    controller.events = []
    call = controller.reroute_forward if moving else controller.reroute_stopped
    local.proactive_admission = True
    with pytest.raises(ValueError, match='action stop line'):
        call('a', new, 1., state=states['a'])
    assert ex['a'].plan is old and controller.plans['a'] is old
    assert controller.capacity.committed == {'road': {'a'}}
    assert not controller.events and local.transition_revision == 0


def test_grant_revoke_and_yield_commit_together_or_leave_all_existing_rights_intact():
    local, ex, states = fleet(('a', 'b'), progress=23.)
    right = lease(local, ex, 'b')
    new = dict(plan=ex['a'].plan, end=35., sweep=[], created_at=9.)
    proposal = TrafficTransition('passage_admission', 'a', revoke=(('b', 1),),
        grants=((('a', 1), new),), waits=(('b', 'a'),), holds=(('b', 23.),))
    def publish():
        del local.passages['b', 1]
        local.passages['a', 1] = new
        local.paused_by['b'] = {('a', 1)}
    result = local.commit_transition(proposal, states, {}, envelope, publish)
    assert not result.allowed
    assert local.passages == {('b', 1): right}
    assert not local.paused_by and not local.released and not local.retry_after
    ex['b'].progress = 20.
    states['b'].x = 20.
    assert local.commit_transition(proposal, states, {}, envelope, publish).allowed
    assert local.passages == {('a', 1): new}
    assert local.paused_by == {'b': {('a', 1)}}


def test_entered_and_stale_route_permissions_cannot_be_timed_out_or_retired():
    local, ex, states = fleet(progress=31.)
    right = lease(local, ex)
    for kind in ('revoke', 'retire'):
        decision = validator(local, ex, states).validate(TrafficTransition('test', 'a', **{kind: (('a', 1),)}))
        assert not decision.allowed
    ex['a'].plan = line_plan()
    assert validator(local, ex, states).lease_phase(('a', 1), right) == 'stale'
    assert not local.release_passages((('a', 1),), states, envelope, reason='timeout_revocation')


def test_retirement_clears_permission_but_keeps_the_finished_body_observable():
    local, ex, states = fleet(progress=42.)
    lease(local, ex)
    assert local.release_passages((('a', 1),), states, envelope, reason='passage_retirement', retire=True)
    assert not local.passages and states['a'].x == 42. and 'a' in ex


def test_shared_guard_exhausts_three_vehicle_wait_graphs_for_every_yield_origin():
    names = ('a', 'b', 'c')
    edges = list(itertools.permutations(names, 2))
    checked = 0
    for mask in range(1 << len(edges)):
        local, ex, states = fleet(names)
        adjacency = {n: set() for n in names}
        for index, (a, b) in enumerate(edges):
            if mask & (1 << index):
                adjacency[a].add(b)
        local.admission_dependencies = [dict(vehicle=a, blocked_by=b) for a in names for b in adjacency[a]]
        reachable = {'a'}
        pending = ['a']
        while pending:
            for b in adjacency[pending.pop()]:
                if b not in reachable:
                    reachable.add(b); pending.append(b)
        for yielding in ('b', 'c'):
            for kind in ('passage_admission', 'clearance_yield', 'stopped_diversion'):
                proposal = TrafficTransition(kind, 'a', waits=((yielding, 'a'),), holds=((yielding, 0.),))
                assert validator(local, ex, states).validate(proposal).allowed == (yielding not in reachable)
                checked += 1
    assert checked == 384


def test_only_the_rechecked_request_is_removed_from_the_prospective_graph():
    local, ex, states = fleet(('a', 'b'))
    local.admission_dependencies = [dict(vehicle='a', blocked_by='b', resource='passage:1')]
    proposal = TrafficTransition('passage_admission', 'a', waits=(('b', 'a'),), resolved=(('a', 1),))
    assert validator(local, ex, states).validate(proposal).allowed
    local.admission_dependencies.append(dict(vehicle='a', blocked_by='b', resource='route'))
    assert not validator(local, ex, states).validate(proposal).allowed


def test_new_yield_cannot_require_a_moving_owner_to_stop_immediately():
    local, ex, states = fleet(('a', 'b'), progress=10., speed=2.)
    proposal = TrafficTransition('clearance_yield', 'a', waits=(('b', 'a'),), holds=(('b', 10.),))
    result = validator(local, ex, states).validate(proposal)
    assert not result.allowed and result.reason == 'yield_stop_not_executable'


def test_grant_checks_measured_body_even_when_time_windows_claim_compatibility():
    local, ex, states = fleet(('a', 'b'))
    proposal = TrafficTransition('passage_admission', 'a', compatible=('b',),
        grants=((('a', 1), dict(plan=ex['a'].plan, end=35., sweep=[envelope(states['b'])])),))
    result = TransitionValidator(local, ex, states, {'b': [envelope(states['b'])]}, envelope).validate(proposal)
    assert not result.allowed and result.reason == 'passage_occupied_body'


def test_failed_publication_rolls_back_permissions_precedence_and_revision():
    local, ex, states = fleet(progress=20.)
    original = lease(local, ex)
    proposal = TrafficTransition('cycle_revocation', 'a', revoke=(('a', 1),))
    def broken_publish():
        del local.passages['a', 1]
        local.paused_by['a'] = {('b', 0)}
        local.events.append(dict(event='partial'))
        raise RuntimeError('injected publication failure')
    with pytest.raises(RuntimeError, match='publication failure'):
        local.commit_transition(proposal, states, {}, envelope, broken_publish)
    assert local.passages['a', 1]['plan'] is original['plan']
    assert local.passages['a', 1] == original
    assert not local.paused_by and not local.events and local.transition_revision == 0


def test_frame_change_between_validation_and_publication_rejects_stale_decision(monkeypatch):
    local, ex, states = fleet(progress=20.)
    lease(local, ex)
    original = TransitionValidator.validate
    def concurrent_frame(self, proposal):
        decision = original(self, proposal)
        states['a'].speed = 1.
        return decision
    monkeypatch.setattr(TransitionValidator, 'validate', concurrent_frame)
    published = []
    decision = local.commit_transition(TrafficTransition('cycle_revocation', 'a', revoke=(('a', 1),)),
        states, {}, envelope, lambda: published.append(True))
    assert not decision.allowed and decision.reason == 'transition_frame_changed'
    assert not published and ('a', 1) in local.passages


def test_finished_leg_handoff_retires_only_old_version_promises_atomically():
    local, ex, states = fleet(progress=35.)
    old = ex['a'].plan
    old.point = lambda station: (station-35., 0.)
    ex['a'].finished = True
    states['a'].x = 0.
    old_right = lease(local, ex)
    next_plan = line_plan()
    next_plan.project = lambda point, station, **kwargs: (0., max(0., point[0]))
    next_plan.origin = 'source'
    next_plan.shared_visits = []
    controller = TransportController.__new__(TransportController)
    controller.retain_finished_vehicles = True
    controller.executions = ex
    controller.plans = {'a': old}
    controller.shared_scheduler = None
    controller.local_traffic = local
    controller.states = states
    controller.boxes = {'a': [envelope(states['a'])]}
    controller.events = []
    controller.continue_route('a', next_plan, 9., state=states['a'])
    assert controller.executions['a'].plan is next_plan
    assert controller.plans['a'] is next_plan
    assert not local.passages
    assert local.released == [(('a', 1), old_right)]
    assert states['a'].x == 0. and not controller.executions['a'].finished
    local.prepare(states, controller.executions, {}, envelope, 10.)
    assert not any(e.get('reason') == 'retiring_uncleared_passage' for e in local.events)


def test_next_leg_cannot_reuse_a_live_old_route_permission():
    local, ex, states = fleet(progress=0.)
    ex['a'].finished = True
    lease(local, ex)
    new = line_plan()
    result = validator(local, ex, states).validate(TrafficTransition('leg_handoff', 'a', handoffs=(('a', new),)))
    assert not result.allowed and result.reason == 'next_leg_retains_stale_permission'


@pytest.mark.parametrize('follower_speed', [0., .2, .8])
def test_changed_action_checks_the_current_frame_and_moving_followers(follower_speed):
    local, ex, states = fleet(('a', 'b'))
    old = line_plan(with_action=False)
    ex['a'].plan = old; ex['a'].progress = 30.
    ex['b'].plan = line_plan(with_action=False); ex['b'].progress = 18.
    states['a'] = VehicleState('a', 30., 0., 0.)
    states['b'] = VehicleState('b', 18., 0., 0., speed=follower_speed)
    new = line_plan(45.)
    new.model.edges['junction']['kind'] = 'uturn'
    new.edge_starts = (0., 45., 75.)
    new.edge_ends = (45., 75., 100.)
    new.point = lambda s: (s if s <= 45. else (90.-s if s <= 75. else s-60.), 0.)
    new.heading = lambda s: math.pi if 45. < s < 75. else 0.
    controller = TransportController.__new__(TransportController)
    controller.model = old.model; controller.local_traffic = local
    controller.executions = ex; controller.plans = {'a': old, 'b': ex['b'].plan}
    # The previous frame put the follower elsewhere; validation must use this
    # supplied complete observation frame and its bodies for both vehicles.
    controller.states = {'a': states['a'], 'b': VehicleState('b', 18., 50., 0.)}
    controller.boxes = {n: [envelope(s)] for n, s in controller.states.items()}
    controller.capacity = SimpleNamespace(committed={}); controller.events = []
    with pytest.raises(ValueError, match='route_clearance_not_executable'):
        controller.reroute_stopped('a', new, 1., state=states['a'], observations=states,
                                  body_boxes={n: [envelope(s)] for n, s in states.items()})
    assert controller.plans['a'] is old and not controller.events


@pytest.mark.parametrize('road_gap', [13.799, 13.8, 13.801])
def test_connected_action_boundary_uses_the_whole_new_outlet(road_gap):
    local, ex, states = fleet(('a', 'b'))
    ex['a'].progress = 20.; states['a'].x = 20.
    ex['b'].progress = 60.; ex['b'].finished = True; states['b'].x = 60.
    new = line_plan()
    second = 35.+road_gap
    new.edges = ('approach', 'junction', 'link', 'change', 'exit')
    new.edge_starts = (0., 30., 35., second, second+5.)
    new.edge_ends = (30., 35., second, second+5., 100.)
    new.model.edges.update(link=dict(kind='road', capacity_alias='link'),
                           change=dict(kind='lane_change', capacity_alias='change'))
    new.edge_index = lambda s: next((i for i, end in enumerate(new.edge_ends) if s < end), 4)
    decision = TransitionValidator(local, ex, states,
        {n: [envelope(s)] for n, s in states.items()}, envelope).validate(
            TrafficTransition('stopped_diversion', 'a', routes=(('a', new),)))
    # A gap shorter than the indivisible action boundary expands the imminent
    # outlet over the retained parked body. At/above it the second action is
    # distant and independently admitted, preserving the earlier contract.
    # Classification uses the stored coordinates; 35+13.8 rounds slightly
    # below the decimal boundary on this platform.
    assert decision.allowed == (second-35. >= 13.8)
    if not decision.allowed:
        assert decision.reason == 'route_clearance_not_executable'
