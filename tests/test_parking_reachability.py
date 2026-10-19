"""Parking must not seal a staged truck's exit or another free waiting slot.

series10_normal_05: the last task could not start because a finished truck had
parked at Q10, the single spur cell between Q9 and the network.
aligned run 20261018: the last truck could not reach the two free slots Q9 and
Q1 because Q10 and Q2 were already parked in front of them.
"""
from pathlib import Path

from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.scheduling.fleet_lifecycle import FleetRegistry
from logistics_park.scheduling.task_dispatch import Assignment, TaskDispatcher, TransportWork

ROOT = Path(__file__).resolve().parents[1]
NETWORK = ROOT / 'config/topology/industrialarea_ue_internal_waiting_network.json'


def dispatcher():
    return TaskDispatcher(ServiceNetwork(NETWORK), FleetRegistry(),
                          protect_committed_routes=True)


def staged(d, *, vehicle='Car_1_0', source='Q9', target='W3-loading'):
    d.positions[vehicle] = source
    d.assignments[vehicle] = Assignment(
        work=TransportWork('transport-26', target, 'W1-unloading'),
        vehicle_id=vehicle, visit_id='leg-1')


def test_parking_across_a_staged_trucks_only_exit_is_blocked():
    d = dispatcher()
    staged(d)
    assert d._blocks_committed_route('Q10', 'Car_5_0') is True


def test_parking_elsewhere_is_still_allowed():
    d = dispatcher()
    staged(d)
    assert d._blocks_committed_route('Q6', 'Car_5_0') is False


def test_parking_in_front_of_an_empty_spur_slot_is_blocked():
    """Q10 in front of an empty Q9 would strand Q9 for the rest of the run."""
    d = dispatcher()
    assert d._blocks_committed_route('Q10', 'Car_5_0') is True
    assert d._blocks_committed_route('Q12', 'Car_5_0') is True


def test_the_inner_slot_itself_stays_available():
    d = dispatcher()
    assert d._blocks_committed_route('Q9', 'Car_5_0') is False
    assert d._blocks_committed_route('Q11', 'Car_5_0') is False


def test_occupied_inner_slot_no_longer_protects_the_front_slot():
    d = dispatcher()
    d.holders['Q9'] = 'Car_2_0'
    assert d._blocks_committed_route('Q10', 'Car_5_0') is False


def test_sealing_rule_holds_without_any_pending_task():
    d = dispatcher()
    assert d._blocks_committed_route('Q10', 'Car_5_0') is True   # Q9 still free
    assert d._blocks_committed_route('Q6', 'Car_5_0') is False


def test_unknown_pending_target_is_ignored_not_crashing():
    d = dispatcher()
    staged(d, source='Q9', target='no-such-target')
    assert d._blocks_committed_route('Q10', 'Car_5_0') is True
