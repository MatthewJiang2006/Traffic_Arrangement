"""A follower must stay outside a leader's pending U-turn or reverse sweep.

series09_normal_08: the follower stopped 3.1 m behind a leader that was 8.2 m
before its U-turn entry. The U-turn sweep reached back into the follower's
body, so the leader could never take the permission (passage_occupied_body)
and the follower could never advance (stationary_swept_body). Backing the
follower up 5 m cleared the sweep, which is exactly what prevention has to
achieve without moving a body: keep it out of the sweep in the first place.
"""
import math
from types import SimpleNamespace

import pytest

from logistics_park.common.messages import VehicleState
from logistics_park.runtime import transport_control as control
from logistics_park.runtime.transport_control import BODY_LENGTH, GAP


class Line:
    """Straight plan along +x, enough for sweep-intersection arithmetic."""

    def __init__(self, length=300., y=0.):
        self.length = length
        self.y = y
        self.edges = ('maneuver',)
        self.model = SimpleNamespace(edges={'maneuver': {'kind': 'uturn'}})

    def point(self, station):
        return station, self.y

    def heading(self, station):
        return 0.


def group(entry, exit_s):
    return (0, 0, entry, exit_s)


def block(ego_progress, leader_progress, entry=80., exit_s=120., group_edges=None):
    plan = Line()
    monkeypatch_group = group_edges or group(entry, exit_s)
    original = control.maneuver_groups
    control.maneuver_groups = lambda leader_plan: [monkeypatch_group]
    try:
        return control.maneuver_block_station(plan, ego_progress, plan, leader_progress)
    finally:
        control.maneuver_groups = original


def test_follower_is_held_before_the_sweep_boundary():
    station = block(ego_progress=40., leader_progress=60.)
    assert station is not None
    # The body front leads the rear axle by BODY_OFFSET + half length, so the
    # rear-axle station stops clear of the first swept body.
    assert 70.5 <= station <= 71.5
    assert station < 76.


def test_no_pending_maneuver_means_no_extra_hold():
    assert block(40., 20.) is None          # entry 60 m ahead, beyond horizon
    assert block(40., 130.) is None         # maneuver already entered
    assert block(200., 60.) is None         # follower is already past the sweep


def test_entered_maneuver_is_left_to_live_guards():
    """An entered sweep is protected by bodies and maneuver holds, not here."""
    assert block(40., 81.) is None


@pytest.mark.parametrize('margin', [.35])
def test_reverse_sweep_is_covered_too(margin):
    plan = Line()
    plan.model = SimpleNamespace(edges={'maneuver': {'kind': 'service_access', 'reverse': True}})
    original = control.maneuver_groups
    control.maneuver_groups = lambda leader_plan: [group(80., 100.)]
    try:
        assert control.maneuver_block_station(plan, 40., plan, 60., margin=margin) is not None
    finally:
        control.maneuver_groups = original


def test_road_maneuver_without_uturn_or_reverse_is_ignored():
    plan = Line()
    plan.model = SimpleNamespace(edges={'maneuver': {'kind': 'road'}})
    original = control.maneuver_groups
    control.maneuver_groups = lambda leader_plan: [group(80., 120.)]
    try:
        assert control.maneuver_block_station(plan, 40., plan, 60.) is None
    finally:
        control.maneuver_groups = original


def test_hold_follows_the_sweep_boundary_not_a_fixed_gap():
    """Moving the sweep entry back moves the hold back by the same amount."""
    near = block(ego_progress=40., leader_progress=60., entry=80.)
    far = block(ego_progress=40., leader_progress=60., entry=95.)
    assert near is not None and far is not None
    assert far - near > 10.
