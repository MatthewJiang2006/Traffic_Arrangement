import math

import pytest

from logistics_park.common.messages import VehicleState
from logistics_park.runtime.transport_control import envelope
from logistics_park.safety.verification import assess_contract, lateral_body_extent, physical_command_ttl, straight_road_spans


@pytest.mark.parametrize('offset,heading', [(0., 0.), (.65, 0.), (0., .2), (-.2, -.1), (.15, math.pi)])
def test_extent_matches_independently_transformed_body_corners(offset, heading):
    body = envelope(VehicleState('truck', 0., offset, heading))
    assert lateral_body_extent(offset, heading) == pytest.approx(max(abs(y) for x, y in body.corners()))


def test_rear_axle_threshold_does_not_prove_complete_body_stays_inside_road():
    model = {'edges': [dict(edge_id='road', kind='road', width=3.8, speed_limit=8., centerline=[[0., 0.], [40., 0.]])]}
    report = assess_contract(model, 2.)
    witness = report['rear_error_counterexample']
    assert witness['permitted_by_rear_error_check']
    assert witness['rear_offset_m'] == pytest.approx(.675)
    assert witness['body_excess_m'] == pytest.approx(.075)
    assert report['stopping_timing']['additional_constant_speed_distance_m'] == 0
    assert report['stopping_timing']['native_command_deadline_s'] == .5
    assert not report['stopping_timing']['hold_exceeds_planner_reaction']
    assert report['status'] == 'NOT_CERTIFIED'


def test_consistent_thresholds_still_do_not_certify_native_physics_or_liveness():
    model = {'edges': [dict(edge_id='road', kind='road', width=4.4, speed_limit=8., centerline=[[0., 0.], [40., 0.]])]}
    report = assess_contract(model, .2)
    assert not report['zero_yaw_rear_error_counterexample_found']
    assert not report['stopping_timing']['hold_exceeds_planner_reaction']
    assert report['status'] == 'NOT_CERTIFIED' and report['unproved']


def test_effective_ttl_override_is_read_without_running_the_native_validator():
    assert physical_command_ttl('cfg.update(command_ttl_s=2, state_timeout_s=5.)') == 2.
    with pytest.raises(ValueError):physical_command_ttl('cfg.update(state_timeout_s=5.)')
    with pytest.raises(ValueError):physical_command_ttl('cfg.update(command_ttl_s=2); cfg.update(command_ttl_s=3)')


def test_body_containment_check_merges_subdivided_straight_roads_and_rejects_gaps():
    edges = [dict(edge_id=str(i), capacity_alias='road', kind='road', width=3.8, speed_limit=8.,
                  centerline=[[i * 4., 0.], [(i + 1) * 4., 0.]]) for i in range(10)]
    assert len(list(straight_road_spans(edges))) == 1
    report = assess_contract({'edges': edges}, 2.)
    assert report['zero_yaw_rear_error_counterexample_found']
    assert report['rear_error_counterexample']['rear_xy_m'] == pytest.approx([20., .675])
    assert len(list(straight_road_spans([edges[0], edges[2]]))) == 0


def test_curved_alias_is_not_misrepresented_as_a_straight_lane_strip():
    edges = [dict(edge_id='a', capacity_alias='road', centerline=[[0., 0.], [20., 0.]]),
             dict(edge_id='b', capacity_alias='road', centerline=[[20., 0.], [20., 20.]])]
    assert not list(straight_road_spans(edges))
