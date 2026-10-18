"""Parking must not seal a staged truck's only exit.

series10_normal_05: the last task (Car_1_0, staged at Q9, bound for
W3-loading) could never start because a finished truck had parked at Q10, the
single spur cell between Q9 and the network. Every route from Q9 then failed,
the whole fleet stood still and the fleet watchdog stopped the run at 31/32.
Waiting slots share one access spur, so the parking choice has to check that
a pending task can still leave its source afterwards.
"""
import pytest

from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.scheduling.fleet_lifecycle import FleetRegistry
from logistics_park.scheduling.task_dispatch import Assignment, TaskDispatcher, TransportWork
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NETWORK = ROOT / 'config/topology/industrialarea_ue_internal_waiting_network.json'


def dispatcher():
    return TaskDispatcher(ServiceNetwork(NETWORK), FleetRegistry(),
                          protect_committed_routes=True)


def staged(dispatcher, *, vehicle='Car_1_0', source='Q9', target='W3-loading'):
    dispatcher.positions[vehicle] = source
    dispatcher.assignments[vehicle] = Assignment(
        work=TransportWork('transport-26', target, 'W1-unloading'),
        vehicle_id=vehicle, visit_id='leg-1')


def test_parking_across_a_staged_trucks_only_exit_is_blocked():
    d = dispatcher()
    staged(d)
    assert d._blocks_committed_route('Q10', 'Car_5_0') is True


def test_parking_elsewhere_is_still_allowed():
    d = dispatcher()
    staged(d)
    assert d._blocks_committed_route('Q2', 'Car_5_0') is False


def test_no_pending_task_means_no_extra_restriction():
    d = dispatcher()
    d.positions['Car_5_0'] = 'W3-unloading'
    assert d._blocks_committed_route('Q10', 'Car_5_0') is False


def test_unknown_pending_target_is_ignored_not_crashing():
    d = dispatcher()
    staged(d, source='Q9', target='no-such-target')
    assert d._blocks_committed_route('Q10', 'Car_5_0') is False
