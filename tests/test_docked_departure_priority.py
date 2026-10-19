"""A docked departure must outrank a remote inbound truck's exit reserve.

Replay of series08_normal_05: the truck in W5-loading had finished loading and
had to leave through the apron that W6-unloading reserves for its future exit,
while that inbound truck was still 68 m away and itself blocked by the W5
queue. The three promises closed a loop with no body in it, so no reversing
move could have cut it.
"""
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from logistics_park.common.messages import VehicleState
from logistics_park.runtime import safe_landing as landing
from logistics_park.routing.service_network import ServiceNetwork

ROOT = Path(__file__).resolve().parents[1]
NETWORK = ROOT / 'config/topology/industrialarea_ue_internal_waiting_network.json'
DOCKED = 'docked'
OWNER = 'owner'


def network():
    return ServiceNetwork(NETWORK)


def build(*, dock='W5-loading', target='W6-loading', owner_target='W6-unloading',
          vehicle_state='READY', owner_progress=0., owner_at_origin=True,
          owner_speed=0., vehicle_speed=0., owner_xy=None, owner_finished=False,
          origin='fleet24-start-02'):
    net = network()
    owner_pose = owner_xy or net.services[origin]['rear_xy_m']
    return SimpleNamespace(
        dispatch=SimpleNamespace(
            network=net,
            assignments={DOCKED: SimpleNamespace(state=vehicle_state),
                         OWNER: SimpleNamespace(state='MOVING')},
            positions={DOCKED: dock,
                       OWNER: origin if owner_at_origin else 'upstream-gate'},
            moves={DOCKED: SimpleNamespace(origin=dock, destination=target),
                   OWNER: SimpleNamespace(origin=origin, destination=owner_target)},
            holders={owner_target: OWNER},
        ),
        controller=SimpleNamespace(
            executions={OWNER: SimpleNamespace(progress=owner_progress,
                                               finished=owner_finished)},
            local_traffic=None,
        ),
        states={
            DOCKED: VehicleState(DOCKED, *net.services[dock]['rear_xy_m'],
                                 math.radians(net.services[dock]['heading_deg']),
                                 vehicle_speed),
            OWNER: VehicleState(OWNER, *owner_pose,
                                math.radians(net.services[origin]['heading_deg']),
                                owner_speed),
        },
    )


def has_priority(ex):
    return landing.docked_departure_has_priority(ex, DOCKED, OWNER, ex.states)


def test_inbound_for_the_same_dock_still_yields_to_the_departure():
    """The occupant leaving outranks the next truck's promise for that berth."""
    ex = build(owner_target='W6-unloading')
    assert has_priority(ex)


def test_recorded_series08_promise_is_deferred():
    # Car_0_1 was staged at fleet24-start-02, 68.8 m from W6-unloading.
    net = network()
    origin = net.services['fleet24-start-02']['rear_xy_m']
    berth_pose = net.services['W6-unloading']['rear_xy_m']
    assert 67. < math.dist(origin, berth_pose) < 70.
    assert has_priority(build())


@pytest.mark.parametrize('overrides', [
    dict(owner_speed=.5),
    dict(vehicle_speed=.5),
])
def test_priority_does_not_apply_outside_a_docked_departure(overrides):
    assert not has_priority(build(**overrides))


def test_priority_stops_at_the_approach_boundary():
    berth_pose = network().services['W6-unloading']['rear_xy_m']
    close = (berth_pose[0], berth_pose[1] - landing.REMOTE_INBOUND_M + 1.)
    assert not has_priority(build(owner_xy=close))


def test_docked_departure_can_land_on_the_reserved_apron(monkeypatch):
    """The wiring: dropping the reserve makes the docked truck's stop legal."""
    from logistics_park.scheduling import local_traffic

    class Plan:
        length = 40.
        edges = ('access_dock',)
        model = SimpleNamespace(edges={'access_dock': {'kind': 'service_access'}})

        def project(self, xy, progress, behind=0., ahead=80.):
            return 0., progress

        def point(self, station):
            return station, 0.

        def heading(self, station):
            return 0.

    apron = SimpleNamespace()
    monkeypatch.setattr(landing, 'maneuver_groups', lambda plan: ())
    monkeypatch.setattr(landing, 'braking_distance', lambda speed: 0.)
    monkeypatch.setattr(landing, 'overlaps', lambda a, b, margin=0.: False)
    monkeypatch.setattr(landing, 'service_exit_sweeps',
                        lambda executor, node, **kw: [apron] if node == 'W6-unloading' else [])
    monkeypatch.setattr(landing, 'occupied_exit_sweeps',
                        lambda executor, node, owner: [apron] if node == 'W6-unloading' else [])
    monkeypatch.setattr(local_traffic, 'sweeps_intersect', lambda a, b, margin=0.: bool(a and b))

    ex = build()
    ex.geometry = SimpleNamespace()
    ex.controller.executions[OWNER].plan = Plan()
    kwargs = dict(states=ex.states, progress=0., speed=0., target='W6-loading',
                  traffic=True, protect_following=False, blocker_evidence=[])
    assert landing.safe_landing(ex, DOCKED, Plan(), 30., **kwargs) == 30.

    # The same pose is refused once the departing truck is no longer standing
    # at its berth, so a live approach keeps its outlet.
    net = network()
    pose = net.services['W5-loading']['rear_xy_m']
    ex.states[DOCKED] = VehicleState(DOCKED, pose[0] + 20., pose[1],
                                     math.radians(net.services['W5-loading']['heading_deg']))
    assert landing.safe_landing(ex, DOCKED, Plan(), 30., **kwargs) is None
