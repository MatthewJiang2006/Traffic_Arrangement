import math

import pytest

from logistics_park.common.messages import VehicleState
from logistics_park.runtime.drivable_surface import DrivableSurface, area
from logistics_park.runtime.road_guard import RoadGuard, body_polygon
from logistics_park.runtime.safety_timing import braking_distance, CONTROL_REACTION_S, EMERGENCY_REACTION_S
from logistics_park.runtime.traffic_policy import stop_line_speed, following_speed


def rectangle(x0,y0,x1,y1,z=1.1):
    return [[x0,y0,z,x1,y0,z,x1,y1,z], [x0,y0,z,x1,y1,z,x0,y1,z]]


def surface(*regions):
    return DrivableSurface(dict(schema=1,surfaces=[dict(actor=str(i),triangles=triangles) for i,triangles in enumerate(regions)]))


def test_a_hole_under_the_middle_is_detected_even_when_every_body_corner_is_supported():
    s=surface(rectangle(-10,-10,10,-.2),rectangle(-10,.2,10,10),
              rectangle(-10,-.2,-.2,.2),rectangle(.2,-.2,10,.2))
    polygon=body_polygon(0,0,0)
    assert all(s.covers([(x-.001,y-.001),(x+.001,y-.001),(x+.001,y+.001),(x-.001,y+.001)],1.29) for x,y in polygon)
    assert s.uncovered_area(polygon,1.29)==pytest.approx(.16)
    assert not s.covers(polygon,1.29)


def test_overlapping_triangle_patches_cover_a_rotated_body_without_double_counting():
    s=surface(rectangle(-20,-20,1,20),rectangle(-1,-20,20,20))
    for heading in (0,.2,math.pi/2,-math.pi):
        assert s.covers(body_polygon(0,0,heading),1.29)


def test_an_overhead_surface_cannot_be_used_as_ground_support():
    s=surface(rectangle(-20,-20,20,20,z=5.))
    body=body_polygon(0,0,0)
    assert s.uncovered_area(body,1.29)==pytest.approx(area(body))


def test_triangle_winding_and_patch_boundaries_do_not_create_false_holes():
    triangles=rectangle(-20,-20,20,20)
    triangles[0]=triangles[0][6:9]+triangles[0][3:6]+triangles[0][0:3]
    s=surface(triangles)
    assert s.covers(body_polygon(0,0,0),1.29)


@pytest.mark.parametrize('speed,target,start,end',[(4.,8.,-50.,30.),(-4.,-8.,-30.,50.)])
def test_speed_is_limited_before_the_emergency_braking_sweep_leaves_pavement(speed,target,start,end):
    guard=RoadGuard(surface(rectangle(start,-4,end,4)))
    state=VehicleState('truck',0,0,0,speed=speed)
    selected,steering=guard.command(state,target,0.,1.29,0.)
    assert 0<=abs(selected)<abs(target) and selected*target>=0 and steering==0
    assert guard.limited==1
    assert guard.braking_path_clear(state,selected,steering,1.29)


def test_a_stationary_parked_truck_needs_no_fictional_future_motion():
    guard=RoadGuard(surface(rectangle(-4,-1.4,5,1.4)))
    assert guard.command(VehicleState('parked',0,0,0),0,0,1.29,0)==(0,0)


def test_an_observed_body_outside_pavement_is_never_silently_accepted():
    guard=RoadGuard(surface(rectangle(-10,-2,10,2)))
    with pytest.raises(RuntimeError,match='complete body outside'):
        guard.command(VehicleState('truck',0,1.,0),0,0,1.29,0)


def test_live_sweep_cannot_cross_a_gap_between_two_supported_observations():
    guard=RoadGuard(surface(rectangle(-30,-4,-.2,4),rectangle(.2,-4,30,4)))
    assert guard.command(VehicleState('truck',-6,0,0),0,0,1.29,0)==(0,0)
    with pytest.raises(RuntimeError,match='body sweep outside'):
        guard.command(VehicleState('truck',6,0,0),0,0,1.29,1)


def test_longitudinal_limits_reserve_the_same_reaction_budget_as_native_fault_braking():
    assert CONTROL_REACTION_S>=EMERGENCY_REACTION_S+.09
    for distance in (.3,1,5,20,100):
        speed=stop_line_speed(distance)
        assert braking_distance(speed,.25)<=distance+1e-10
    for leader in (0.,1.,4.,8.):
        for gap in (0.,2.,10.):
            selected=following_speed(8.344+1.2+gap,leader)
            assert selected*CONTROL_REACTION_S+selected**2/1.2<=gap+leader**2/1.2+1e-10


def test_acceleration_matches_independent_polygon_subtraction_at_holes_and_edges():
    import random
    from logistics_park.runtime.road_guard import swept_body
    row=dict(schema=1,surfaces=[dict(actor='pavement',triangles=rectangle(-20,-10,20,-.3)+
        rectangle(-20,.3,20,10)+rectangle(-20,-.3,-.2,.3)+rectangle(.2,-.3,20,.3))])
    fast=DrivableSurface(row);reference=DrivableSurface(row,accelerated=False)
    if not fast.accelerator:pytest.skip('System GEOS unavailable')
    randomizer=random.Random(16016)
    for _ in range(500):
        x=randomizer.uniform(-23,23);y=randomizer.uniform(-12,12);h=randomizer.uniform(-math.pi,math.pi)
        polygon=swept_body((x,y,h),(x+randomizer.uniform(-2,2),y+randomizer.uniform(-2,2),h+randomizer.uniform(-.2,.2)))
        z=randomizer.choice((.89,1.29,5.))
        assert fast.uncovered_area(polygon,z)==pytest.approx(reference.uncovered_area(polygon,z),abs=1e-8)


def test_repeated_same_frame_guard_checks_reuse_the_same_verified_result():
    guard=RoadGuard(surface(rectangle(-50,-4,30,4)));state=VehicleState('truck',0,0,0,speed=4.)
    first=guard.command(state,8.,0.,1.29,1.)
    checks=guard.checked
    assert guard.command(state,8.,0.,1.29,1.)==first and guard.checked==checks
    assert guard.previous['truck'][4]==first[0]
    guard.command(state,0.,0.,1.29,1.)
    assert guard.checked==checks+1


def test_repeated_observation_does_not_reset_the_retained_wheel_state(monkeypatch):
    guard=RoadGuard(surface(rectangle(-100,-100,100,100)));seen=[]
    original=guard.braking_path_clear
    def record(*args,**kwargs):
        seen.append(args[4] if len(args)>4 else kwargs.get('initial_delta',0.))
        return original(*args,**kwargs)
    monkeypatch.setattr(guard,'braking_path_clear',record)
    state=VehicleState('truck',0,0,0,speed=.1,timestamp=1.)
    guard.command(state,.5,.2,1.29,1.)
    state.timestamp=2.;seen.clear();guard.command(state,.5,.2,1.29,2.)
    assert seen and all(x==.2 for x in seen)
    seen.clear();guard.command(state,.5,.2,1.29,2.1)
    assert seen and all(x==.2 for x in seen)


def test_chunk_hull_optimization_preserves_every_swept_corner_exactly():
    from logistics_park.runtime.road_guard import sweep_points,swept_body
    from logistics_park.runtime.drivable_surface import convex_hull
    poses=[(i*.3,.1*i*i,.05*i) for i in range(5)]
    raw=[];hulls=[]
    for a,b in zip(poses,poses[1:]):raw.extend(sweep_points(a,b));hulls.extend(swept_body(a,b))
    assert convex_hull(raw)==convex_hull(hulls)


def test_emergency_stop_uses_the_actual_ramped_reference_after_acknowledgement():
    guard=RoadGuard(surface(rectangle(-50,-4,30,4)))
    state=VehicleState('truck',0,0,0,speed=.4,timestamp=1.)
    proposed=guard.command(state,8.,0.,1.29,1.)
    assert proposed[0]>1.
    guard.acknowledge_commands({'truck':.5})
    state.x=15.;state.timestamp=2.
    assert guard.command(state,0.,0.,1.29,2.)==(0.,0.)
    assert guard.previous['truck'][4]==0.


def test_model_infeasibility_brakes_until_observed_stop_without_disabling_pavement_checks():
    guard=RoadGuard(surface(rectangle(-20,-2,8,2)))
    s=VehicleState('truck',0,0,0,speed=4.,timestamp=1.)
    assert guard.command(s,0.,0.,1.29,1.)==(0.,0.)
    assert guard.braking_recoveries==1
    s.speed=.5;s.timestamp=2.
    assert guard.command(s,1.,0.,1.29,2.)==(0.,0.)
    s.speed=0.;s.timestamp=3.
    assert guard.command(s,.2,0.,1.29,3.)==(.2,0.)
    assert not guard.braking_recovery


def test_braking_recovery_has_a_finite_deadline_and_checks_the_actual_body():
    guard=RoadGuard(surface(rectangle(-20,-2,8,2)));s=VehicleState('truck',0,0,0,speed=4.)
    guard.command(s,0.,0.,1.29,0.)
    with pytest.raises(RuntimeError,match='bounded braking recovery'):guard.command(s,1.,0.,1.29,21.)
    s.y=2.
    with pytest.raises(RuntimeError,match='complete body outside'):guard.command(s,1.,0.,1.29,22.)
