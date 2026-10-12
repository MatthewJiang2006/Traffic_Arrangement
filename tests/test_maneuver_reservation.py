import pytest
from math import pi

from logistics_park.safety.collision import OBB
from logistics_park.scheduling.maneuver_reservation import ManeuverReservations


def box(x, y=0):
    return OBB(x, y, 0, 9, 3.3)


def test_blocked_region_cannot_be_reserved_until_full_vehicle_clears():
    r = ManeuverReservations({"bay": OBB(0, 0, 0, 10, 10)})
    fleet = {"owner": box(-20), "blocker": box(9)}
    assert not r.request("bay", "owner", fleet, 0)
    fleet["blocker"] = box(10)
    assert r.request("bay", "owner", fleet, 1)


def test_tail_holds_reservation_after_center_leaves():
    r = ManeuverReservations({"bay": OBB(0, 0, 0, 10, 10)})
    fleet = {"owner": box(-20)}
    assert r.request("bay", "owner", fleet, 0)
    fleet["owner"] = box(6)
    assert not r.release("bay", "owner", fleet, 1)
    fleet["owner"] = box(10)
    assert r.release("bay", "owner", fleet, 2)


def test_overlapping_regions_cannot_have_different_owners():
    r = ManeuverReservations({"a": OBB(0, 0, 0, 10, 10),
                              "b": OBB(8, 0, 0, 10, 10)})
    fleet = {"first": box(-30), "second": box(30)}
    assert r.request("a", "first", fleet, 0)
    assert not r.request("b", "second", fleet, 1)
    with pytest.raises(ValueError):
        r.release("a", "second", fleet, 2)


def test_intrusion_includes_rotated_tail_and_touching():
    r = ManeuverReservations({"bay": OBB(0, 0, 0, 10, 10)})
    fleet = {"owner": box(-30), "other": box(30)}
    assert r.request("bay", "owner", fleet, 0)
    fleet["other"] = box(9.5)
    assert r.intruders("bay", fleet) == ["other"]
    fleet["other"] = OBB(9, 0, pi/4, 9, 3.3)
    assert r.intruders("bay", fleet) == ["other"]
