"""Keep W2's reverse dock independent of the overlapping forward approach."""
import math
from pathlib import Path

import pytest

from logistics_park.runtime.transport_routes import RouteGeometry, TransportMap
from logistics_park.runtime.transport_control import TransportController
from logistics_park.common.messages import VehicleState


ROOT = Path(__file__).resolve().parents[1]


def dock_plan():
    model = TransportMap(ROOT / 'config/topology/industrialarea_ue_internal_waiting_transport.json')
    edges = ['road_j_3_2_j_2_2__service_0__test0',
             'road_j_3_2_j_2_2__service_0__test1', 'access_008__test0']
    edges += [f'access_009__test{i}' for i in range(4)]
    return RouteGeometry(model, edges,
                         [dict(event='GEAR_CHANGE', after_edge_count=3, direction=-1)],
                         'road', ['W2-loading'])


def test_forward_stop_corrects_heading_instead_of_chasing_nearby_gear_point():
    plan = dock_plan()
    gear = plan.checkpoints[0]['s']
    # The rear axle is only six centimetres off the road, but its nose is
    # already pointing too far west. More left lock makes docking worse.
    steering = plan.steering((44.960, 248.474), math.radians(-99.701), gear - .310, .24)
    assert steering > 0


def test_reverse_projection_does_not_return_to_the_completed_forward_leg():
    plan = dock_plan()
    gear = plan.checkpoints[0]['s']
    error, station = plan.project((44.992190664159935, 248.81496154191103), gear,
                                  start_s=gear)
    assert station > gear + .3
    assert .1 < error < .4


@pytest.mark.parametrize('floor_offset', [0, .07])
def test_projection_floor_clamps_segments_at_the_requested_station(floor_offset):
    plan = dock_plan()
    floor = plan.checkpoints[0]['s'] + floor_offset
    point = plan.point(floor - .1)
    _, station = plan.project(point, floor, start_s=floor)
    assert station >= floor - 1e-10


def test_controller_excludes_a_completed_gear_from_progress_measurement():
    plan = dock_plan()
    controller = TransportController(plan.model, {'truck': plan}, service_s=0)
    controller.activate('truck', 0)
    execution = controller.executions['truck']
    gear = plan.checkpoints[0]['s']
    execution.progress = gear
    execution.checkpoint = 1
    state = VehicleState('truck', 44.992190664159935, 248.81496154191103,
                         -1.6221451352468779, speed=-.7894606839270867)
    controller.tick({'truck': state}, 1)
    assert execution.progress > gear + .3


@pytest.mark.parametrize('incoming_reverse', [False, True])
def test_pending_gear_uses_incoming_steering_until_the_vehicle_has_stopped(incoming_reverse):
    plan = dock_plan()
    if incoming_reverse:
        for edge in plan.edges:
            plan.model.edges[edge]['reverse'] = not plan.model.edges[edge].get('reverse', False)
    controller = TransportController(plan.model, {'truck': plan}, service_s=0)
    controller.activate('truck', 0)
    execution = controller.executions['truck']
    gear = plan.checkpoints[0]['s']
    execution.progress = gear - .1
    x, y = plan.point(gear)
    heading = plan.heading(gear - .02)
    direction = -1 if incoming_reverse else 1
    state = VehicleState('truck', x, y, heading, speed=direction * .3)
    command = controller.tick({'truck': state}, 1)['truck']
    assert execution.checkpoint == 0
    assert plan.reverse(execution.progress) is incoming_reverse
    assert abs(command[1]) < math.radians(1)
    state.speed = direction * .01
    controller.tick({'truck': state}, 2)
    assert execution.checkpoint == 1
    assert plan.reverse(execution.progress) is not incoming_reverse


@pytest.mark.parametrize('dt', [.1, .2, .3])
@pytest.mark.parametrize('lag', [.2, .4, .6])
@pytest.mark.parametrize('offset,yaw_error', [(.15, 3), (-.15, -3)])
def test_stop_and_reverse_with_native_steering_response(dt, lag, offset, yaw_error):
    plan = dock_plan()
    gear = plan.checkpoints[0]['s']
    station = gear - 5
    x, y = plan.point(station)
    x += offset
    heading = plan.heading(station) + math.radians(yaw_error)
    speed, actual_delta, peak = 1.5, 0., 0.
    reverse = False
    for _ in range(1000):
        options = {'start_s': gear} if reverse else {'stop_s': gear}
        error, projected = plan.project((x, y), station, **options)
        station = max(station, projected)
        peak = max(peak, error)
        if reverse:
            target = -min(1., max(0, plan.length - station) * .8)
        else:
            remaining = max(0, gear - station)
            target = min(1.5, math.sqrt(1.2 * remaining), remaining * .8)
            if remaining < .22 and abs(speed) < .08:
                reverse, station, target = True, gear, 0.
        desired_delta = plan.steering((x, y), heading, station, speed)
        actual_delta += (.82 * desired_delta - actual_delta) * (1 - math.exp(-dt / lag))
        speed += max(-.6 * dt, min(.6 * dt, target - speed))
        if abs(target) < .08:
            speed *= math.exp(-dt / .15)
        change = speed * math.tan(actual_delta) / 2.3285 * dt
        middle = heading + change / 2
        x += speed * math.cos(middle) * dt
        y += speed * math.sin(middle) * dt
        heading += change
        if station > plan.length - .22 and abs(speed) < .08:
            break
    assert reverse and station > plan.length - .22
    assert peak < .7
