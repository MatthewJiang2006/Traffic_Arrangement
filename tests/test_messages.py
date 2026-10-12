from dataclasses import asdict
import json
import pytest

from logistics_park.common.enums import Maneuver, ReservationStatus
from logistics_park.common.messages import (ConflictZone, ControlCommand, Movement, Reservation,
                                           RoadState, Task, VehicleState)
from logistics_park.common.time_utils import PeriodicTimer


def test_state_and_task_serialization():
    state = VehicleState("v", 1, 2, 0, route=("ab",))
    task = Task("t", "v", "a", "b")
    assert json.loads(json.dumps(asdict(state)))["route"] == ["ab"]
    assert json.loads(json.dumps(asdict(task)))["status"] == "PENDING"
    with pytest.raises(ValueError):
        VehicleState("v", float("nan"), 0, 0)
    with pytest.raises(ValueError):
        VehicleState("v", 0, 0, 0, route=("ab",), route_index=1)
    with pytest.raises(ValueError):
        Task("t", "v", "a", "b", release_time=-1)


def test_command_expiry_and_sequence():
    command = ControlCommand("v", 10, 1.0, 0.2, 0.0, ttl=0.1)
    assert command.is_valid_at(1.05, 9)
    assert not command.is_valid_at(0.99)
    assert not command.is_valid_at(1.1)
    assert not command.is_valid_at(1.05, 10)
    with pytest.raises(ValueError):
        ControlCommand("v", 0, 0, 0, 0, ttl=0)


def test_predicted_capacity():
    state = RoadState("ab", 3, current_vehicle_count=1, committed_vehicle_count=2)
    assert state.occupancy_ratio == pytest.approx(1/3)
    assert state.predicted_occupancy_ratio == 1
    assert state.downstream_available_capacity == 0 and state.is_saturated
    assert RoadState("closed", 0).is_saturated
    with pytest.raises(ValueError):
        RoadState("ab", -1)


def test_movement_zone_and_reservation():
    zone = ConflictZone("z", ((0,0),(1,0),(1,1),(0,1)))
    movement = Movement("m", "ab", "bc", Maneuver.LEFT, ((0,0),(1,1)), (zone.zone_id,), 0.2)
    reservation = Reservation("v", "i", movement.movement_id, 1.2, 1.0, 1.4, {"z":(1.0,1.6)})
    assert reservation.status == ReservationStatus.REQUESTED
    assert json.loads(json.dumps(asdict(reservation)))["status"] == "REQUESTED"
    with pytest.raises(ValueError):
        Reservation("v", "i", "m", 2.0, 1.0, 1.4)
    with pytest.raises(ValueError):
        Reservation("v", "i", "m", 1.2, 1.0, 1.4, {"z":(2.0,1.0)})
    with pytest.raises(ValueError):
        ConflictZone("z", ((0,0),(1,0),(2,0)))
    with pytest.raises(ValueError):
        Movement("m", "a", "b", Maneuver.LEFT, ((0,0),), (), 0.2)


def test_multirate_timer_does_not_drift():
    timer = PeriodicTimer(0.05)
    times = [tick*0.02 for tick in range(500) if timer.ready(tick*0.02)]
    assert len(times) == 200
    assert all(-1e-9 <= time-index*0.05 < 0.02+1e-9 for index,time in enumerate(times))
