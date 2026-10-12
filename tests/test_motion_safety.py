from math import pi
import pytest

from logistics_park.common.messages import ControlCommand, VehicleState
from logistics_park.safety.collision import OBB, overlaps
from logistics_park.safety.following_guard import following_speed_cap
from logistics_park.simulation.lightweight.kinematics import VehicleLimits, step, wheel_speeds, body_speeds
from logistics_park.navigation.pure_pursuit import PurePursuit


def test_sat_rotation_separation_touch_and_margin():
    box = OBB(0,0,0,0.12,0.08)
    assert overlaps(box,OBB(0.05,0,pi/4,0.12,0.08))
    assert not overlaps(box,OBB(0.3,0,pi/4,0.12,0.08))
    assert overlaps(box,OBB(0.12,0,0,0.12,0.08))
    nearby = OBB(0.14,0,0,0.12,0.08)
    assert not overlaps(box,nearby)
    assert overlaps(box,nearby,0.015)
    # Long parallel diagonal boxes have overlapping AABBs but disjoint OBBs.
    assert not overlaps(OBB(0,0,pi/4,1,0.05),OBB(-0.1,0.1,pi/4,1,0.05))


def test_differential_drive_and_limits():
    limits = VehicleLimits()
    left,right = wheel_speeds(0.3,1.0,limits.wheel_track)
    assert body_speeds(left,right,limits.wheel_track) == pytest.approx((0.3,1.0))
    state = VehicleState("v",0,0,0)
    updated = step(state,ControlCommand("v",0,0,5,20),0.02,limits)
    assert updated.speed == pytest.approx(limits.a_max*0.02)
    assert updated.omega == limits.omega_max
    assert updated.timestamp == 0.02 and state.timestamp == 0
    with pytest.raises(ValueError):
        step(state,ControlCommand("other",0,0,0,0),0.02,limits)


def test_straight_and_rotating_kinematics():
    limits = VehicleLimits(a_max=10)
    state = VehicleState("v",0,0,0,speed=0.2)
    straight = step(state,ControlCommand("v",0,0,0.2,0,ttl=2),1,limits)
    assert straight.x == pytest.approx(0.2) and straight.y == 0
    rotated = step(state,ControlCommand("v",0,0,0.2,pi/2,ttl=2),1,limits)
    assert rotated.x == pytest.approx(0.2/(pi/2))
    assert rotated.y == pytest.approx(0.2/(pi/2))
    assert rotated.heading == pytest.approx(pi/2)


def test_expired_command_brakes():
    state = VehicleState("v",0,0,0,speed=0.2,timestamp=1)
    updated = step(state,ControlCommand("v",0,0,0.5,2),0.02,VehicleLimits())
    assert updated.speed < state.speed and updated.omega == 0


def test_following_uses_arc_length_and_stopped_leader():
    rear = VehicleState("rear",0,0,0,edge_id="e",s=0.2)
    leader = VehicleState("front",10,10,0,edge_id="e",s=0.38)
    assert 0 <= following_speed_cap(rear,leader,0.12) < 0.03
    leader.s = 0.3
    assert following_speed_cap(rear,leader,0.12) == 0
    leader.edge_id = "other"
    assert following_speed_cap(rear,leader,0.12) == float("inf")


def test_tracker_corrects_lateral_and_heading_errors(graph,config):
    tracker = PurePursuit(config.tracking)
    state = VehicleState("v",0.8,0.23,0.1,speed=0.2,edge_id="e0",s=0.35,
                         lateral_offset=0.03,heading_error=0.1,route=config.route)
    command = tracker.command(graph,state,0,cyclic=True)
    assert command.omega_target < 0
    assert 0 < command.v_target <= graph.edges["e0"].speed_limit

def test_fast_sat_matches_independent_corner_projections_for_rotated_trucks():
    import random,math
    rng=random.Random(913)
    def reference(a,b,margin):
        one,two=a.corners(margin),b.corners(margin)
        for h in [a.heading,b.heading]:
            for ax,ay in [(math.cos(h),math.sin(h)),(-math.sin(h),math.cos(h))]:
                p=[x*ax+y*ay for x,y in one];q=[x*ax+y*ay for x,y in two]
                if max(p)<min(q)-1e-12 or max(q)<min(p)-1e-12:return False
        return True
    for _ in range(1000):
        a=OBB(rng.uniform(-100,100),rng.uniform(-100,100),rng.uniform(-math.pi,math.pi),8.344,2.6)
        b=OBB(a.x+rng.uniform(-12,12),a.y+rng.uniform(-12,12),rng.uniform(-math.pi,math.pi),rng.uniform(2,9),rng.uniform(1,4))
        for margin in [0.,.2,.35]:assert overlaps(a,b,margin)==reference(a,b,margin)
