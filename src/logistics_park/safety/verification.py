"""Bounded checks and explicit gaps; this module does not certify native UE."""
import ast
import inspect
import math

from logistics_park.runtime.traffic_policy import stop_line_speed
from logistics_park.runtime.transport_control import BODY_LENGTH, BODY_OFFSET, BODY_WIDTH
from logistics_park.runtime.safety_timing import EMERGENCY_REACTION_S,NATIVE_CONTROL_TIMEOUT_S


def lateral_body_extent(rear_offset, heading_error):
    """Maximum absolute lateral coordinate of the four body corners."""
    center = rear_offset + BODY_OFFSET * math.sin(heading_error)
    half = BODY_LENGTH / 2 * abs(math.sin(heading_error)) + BODY_WIDTH / 2 * abs(math.cos(heading_error))
    return abs(center) + half


def physical_command_ttl(source):
    """Read the effective hard-coded UE override without importing its runner."""
    values = []
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):continue
        if not isinstance(node.func.value, ast.Name) or node.func.value.id != 'cfg' or node.func.attr != 'update':continue
        for keyword in node.keywords:
            if keyword.arg == 'command_ttl_s':values.append(float(ast.literal_eval(keyword.value)))
    if len(values) != 1 or not math.isfinite(values[0]) or values[0] <= 0:
        raise ValueError('Cannot establish one fixed native command TTL')
    return values[0]


def straight_road_spans(roads):
    """Merge collinear slices before testing an entire body's lane strip."""
    groups = {}
    for edge in roads:groups.setdefault(edge.get('capacity_alias', edge['edge_id']), []).append(edge)
    for alias, edges in groups.items():
        origin, end = edges[0]['centerline'][0], edges[0]['centerline'][-1]
        length = math.dist(origin, end)
        if length < 1e-8:continue
        co, si = (end[0] - origin[0]) / length, (end[1] - origin[1]) / length
        intervals = []
        for edge in edges:
            points = edge['centerline']
            if any(abs(-(p[0] - origin[0]) * si + (p[1] - origin[1]) * co) > .01 for p in points):break
            stations = [(p[0] - origin[0]) * co + (p[1] - origin[1]) * si for p in points]
            intervals.append((min(stations), max(stations), edge))
        else:
            intervals.sort(key=lambda row: row[0])
            spans = []
            for lo, hi, edge in intervals:
                if not spans or lo > spans[-1][1] + .01:spans.append([lo, hi, [edge]])
                else:
                    spans[-1][1] = max(spans[-1][1], hi)
                    spans[-1][2].append(edge)
            for lo, hi, members in spans:
                if hi - lo > BODY_LENGTH + 2:
                    station = (lo + hi) / 2
                    yield alias, members, (origin[0] + co * station, origin[1] + si * station), (co, si)


def assess_contract(model, command_ttl_s):
    """Refute two insufficient assumptions, without claiming reachability.

    An offset witness disproves ``rear axle error <= tolerance implies the
    complete body fits this lane``. It is not evidence that a native run has
    reached that pose. Timing compares declared limits, not measured brakes.
    """
    roads = [edge for edge in model['edges'] if edge['kind'] == 'road']
    witnesses = []
    for alias, edges, center, direction in straight_road_spans(roads):
        width = min(edge['width'] for edge in edges)
        tolerance = min(edge.get('maximum_path_error_m', .7) for edge in edges)
        safe_offset = width / 2 - BODY_WIDTH / 2
        if safe_offset < 0 or tolerance <= safe_offset:continue
        rear_offset = max(0., safe_offset) + .75 * (tolerance - max(0., safe_offset))
        co, si = direction
        witnesses.append(dict(capacity_alias=alias, lane_width_m=width,
            rear_xy_m=[center[0] - si * rear_offset, center[1] + co * rear_offset],
            world_heading_rad=math.atan2(si, co),
            rear_offset_m=rear_offset, rear_error_limit_m=tolerance, heading_error_deg=0.,
            permitted_by_rear_error_check=rear_offset <= tolerance,
            body_excess_m=lateral_body_extent(rear_offset, 0.) - width / 2))
    reaction = inspect.signature(stop_line_speed).parameters['reaction_s'].default
    speed = max(edge['speed_limit'] for edge in roads)
    timing = dict(planner_reaction_s=reaction, host_command_ttl_s=command_ttl_s,
                  native_command_deadline_s=NATIVE_CONTROL_TIMEOUT_S,
                  configured_emergency_reaction_s=EMERGENCY_REACTION_S,
                  hold_exceeds_planner_reaction=EMERGENCY_REACTION_S > reaction,
                  additional_constant_speed_distance_m=max(0., EMERGENCY_REACTION_S - reaction) * speed,
                  note='Modern timestamped native controls bound command hold independently of the host TTL. These are configured model bounds; native physics remains subject to measured validation.')
    return dict(status='NOT_CERTIFIED',
        zero_yaw_rear_error_counterexample_found=bool(witnesses),
        rear_error_counterexample_count=len(witnesses),
        rear_error_counterexample=witnesses[0] if witnesses else None,
        counterexample_scope='Topology lane-strip implication only; native reachability and actual UE pavement departure are not established.',
        stopping_timing=timing,
        protections=['Loaded-pavement complete body, observed sweep and modeled braking-path command filter',
                     'Native timestamp/deadline/state-age validation, replay rejection and command-loss braking',
                     'Independent native-sample pavement audit covering all route kinds'],
        unproved=['Continuous-state containment outside the bounded steering/braking response model',
                  'Reachability of every physical wait graph and eventual task completion',
                  'Fault-free operation for unbounded time'])
