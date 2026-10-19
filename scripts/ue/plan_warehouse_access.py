"""Read-only hybrid-A* candidate paths from legal roads to berths."""
import heapq
import json
import math
from pathlib import Path
import time

import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/ue_scene/station_siting"
request_path = OUT / "route_request.json"
request = json.loads(request_path.read_text()) if request_path.exists() else {}
target_file = OUT / request.get("output", "route_candidates.json")
plan = json.loads((ROOT / "config/planning/ue_station_proposal.json").read_text(encoding="utf-8"))
roads = json.loads((ROOT / "config/topology/industrialarea_ue.json").read_text())
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
origin = unreal.GameplayStatics.get_world_origin_location(world)
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
ignore = [a for a in actors if a.get_class().get_name() in
          ("TrafficTruckPawn", "BP_Truck_C", "CameraActor")]
STEP = request.get("step_m", 1.5)
TURN_DEG = request.get("turn_deg", 10)
HEADING_COUNT = 360 // TURN_DEG
if TURN_DEG not in (10, 15, 20):
    raise ValueError("turn_deg must divide 360 and be 10, 15 or 20")
MAX_EXPANSIONS = request.get("max_expansions", 5000)
GROUND_TOLERANCE = request.get("ground_tolerance_m", .08)
GROUND_DIAGNOSTIC = request.get("ground_contact_is_diagnostic", False)
WHEEL_HALF_TRACK = 1.1


def ground(x, y):
    start = unreal.Vector(x*100-origin.x, y*100-origin.y, 1200-origin.z)
    end = unreal.Vector(start.x, start.y, -100-origin.z)
    hit = unreal.SystemLibrary.line_trace_single(
        world, start, end, unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,
        False, ignore, unreal.DrawDebugTrace.NONE, True)
    row = hit.to_tuple() if hit else None
    return ((row[5].z+origin.z)/100, row[9].get_name() if row[9] else "") if row and row[0] else (None, "")


cache = {}
active_target = None

def parked_overlap(x,y,yaw,point):
    h=math.radians(yaw);other=math.radians(point['heading_deg'])
    bx,by=point.get('rear_xy_m',point['xy_m'])
    # Actual body centre is forward of the axle midpoint. Test padded full
    # bodies so every other selected bay may remain occupied during access.
    offset=.75
    ax,ay=x+offset*math.cos(h),y+offset*math.sin(h)
    bx,by=bx+.75*math.cos(other),by+.75*math.sin(other)
    for angle in (h,h+math.pi/2,other,other+math.pi/2):
        ux,uy=math.cos(angle),math.sin(angle)
        ra=4.172*abs(math.cos(h-angle))+1.3*abs(math.sin(h-angle))
        rb=4.172*abs(math.cos(other-angle))+1.3*abs(math.sin(other-angle))
        if abs((ax-bx)*ux+(ay-by)*uy)>ra+rb+.35:return False
    return True


def clear(x, y, yaw):
    key = (round(x, 2), round(y, 2), round(yaw) % 360)
    if key in cache:
        return cache[key]
    if any(p['id'] != active_target and parked_overlap(x,y,yaw,p)
           for p in request.get('parked_points',[])):
        cache[key]=(False,'occupied neighbouring waiting bay')
        return cache[key]
    if request.get('pose_is_rear'):
        x+=.75*math.cos(math.radians(yaw))
        y+=.75*math.sin(math.radians(yaw))
    perimeter = request.get('perimeter_m')
    if perimeter:
        co, si = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        if any(not (perimeter[0] <= x+dx*co-dy*si <= perimeter[2]
                    and perimeter[1] <= y+dx*si+dy*co <= perimeter[3])
               for dx, dy in ((-4.5,-1.65),(-4.5,1.65),(4.5,-1.65),(4.5,1.65))):
            cache[key] = (False, 'outside park perimeter')
            return cache[key]
    z, surface = ground(x, y)
    if z is None or not .7 < z < 1.35:
        cache[key] = (False, "no ground: "+surface)
        return cache[key]
    co, si = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    # Check the wheel-side ground as a continuous profile. A vehicle can sit
    # across a gradual ramp even when its front and rear differ in elevation;
    # the old centre-to-corner 8 cm test wrongly rejected those poses.
    for dy in (() if GROUND_DIAGNOSTIC else (-WHEEL_HALF_TRACK, WHEEL_HALF_TRACK)):
        previous_h = None
        for i in range(13):
            dx = -4.5 + .75*i
            h, _ = ground(x+dx*co-dy*si, y+dx*si+dy*co)
            if (h is None or abs(h-z) > .22 or
                    (previous_h is not None and abs(h-previous_h) > GROUND_TOLERANCE)):
                cache[key] = (False, "ground discontinuity")
                return cache[key]
            previous_h = h
    for dx in (() if GROUND_DIAGNOSTIC else (-3.0, 0.0, 3.0)):
        previous_h = None
        for i in range(5):
            dy = -WHEEL_HALF_TRACK + .55*i
            h, _ = ground(x+dx*co-dy*si, y+dx*si+dy*co)
            if (h is None or abs(h-z) > .22 or
                    (previous_h is not None and abs(h-previous_h) > GROUND_TOLERANCE)):
                cache[key] = (False, "ground discontinuity")
                return cache[key]
            previous_h = h
    center = unreal.Vector(x*100-origin.x, y*100-origin.y, (z+2.3)*100-origin.z)
    hit = unreal.SystemLibrary.box_trace_single(
        world, center, center, unreal.Vector(450, 165, 195),
        unreal.Rotator(pitch=0, yaw=yaw, roll=0),
        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, False, ignore,
        unreal.DrawDebugTrace.NONE, True)
    row = hit.to_tuple() if hit else None
    cache[key] = (False, "overlap: "+row[9].get_name()) if row and row[0] else (True, surface)
    return cache[key]


def angle_error(a, b):
    return abs((a-b+180) % 360-180)


def previous(pose, steer):
    x, y, yaw = pose
    prior_yaw = (yaw-steer*TURN_DEG) % 360
    if steer:
        delta = math.radians(steer*TURN_DEG)
        k = delta/STEP
        dx = (math.sin(math.radians(yaw))-math.sin(math.radians(prior_yaw)))/k
        dy = (math.cos(math.radians(prior_yaw))-math.cos(math.radians(yaw)))/k
    else:
        dx = STEP*math.cos(math.radians(yaw))
        dy = STEP*math.sin(math.radians(yaw))
    return (x-dx, y-dy, prior_yaw)


def successor(pose, steer):
    x, y, yaw = pose
    next_yaw = (yaw+steer*TURN_DEG) % 360
    if steer:
        k = math.radians(steer*TURN_DEG)/STEP
        dx = (math.sin(math.radians(next_yaw))-math.sin(math.radians(yaw)))/k
        dy = (math.cos(math.radians(yaw))-math.cos(math.radians(next_yaw)))/k
    else:
        dx = STEP*math.cos(math.radians(yaw))
        dy = STEP*math.sin(math.radians(yaw))
    return (x+dx, y+dy, next_yaw)


def anchors(target):
    x, y = target
    rows = []
    for edge in roads["edges"]:
        if not edge["edge_id"].startswith("road_"):
            continue
        path = edge["centerline"]
        for a, b in zip(path, path[1:]):
            length = math.hypot(a[0]-b[0], a[1]-b[1])
            if length < .01:
                continue
            heading = math.degrees(math.atan2(b[1]-a[1], b[0]-a[0])) % 360
            count = max(1, math.ceil(length/4))
            for i in range(count+1):
                u = i/count
                px, py = a[0]+(b[0]-a[0])*u, a[1]+(b[1]-a[1])*u
                if math.hypot(px-x, py-y) <= 48:
                    rows.append((px, py, heading, edge["edge_id"]))
    return rows


def route(label, point):
    global active_target
    active_target=label
    if request.get('parked_points'):cache.clear()
    target = (point["xy_m"][0], point["xy_m"][1], point["heading_deg"] % 360)
    candidates = anchors(target[:2])
    if not candidates:
        return {"id": label, "status": "NO_ROAD_ANCHOR"}
    if not clear(*target)[0]:
        return {"id": label, "status": "TARGET_BLOCKED"}
    if request.get("mode", "entry") == "exit":
        return route_exit(label, target, candidates)
    if request.get("mode", "entry") == "reverse_entry":
        return route_reverse_entry(label, target, candidates)
    # Reverse graph search: each predecessor can drive forward to the current
    # pose using a straight or radius 8.6 m arc. No reverse gear is allowed.
    def heuristic(p):
        return min(math.hypot(p[0]-a[0], p[1]-a[1]) for a in candidates)

    start_key = (round(target[0]), round(target[1]), round(target[2]/TURN_DEG) % HEADING_COUNT)
    frontier = [(heuristic(target), 0, start_key, target)]
    best = {start_key: 0}
    next_pose = {}
    expanded = 0
    counter = 0
    bounds = (target[0]-55, target[0]+55, target[1]-55, target[1]+55)
    while frontier and expanded < MAX_EXPANSIONS:
        _, cost, key, pose = heapq.heappop(frontier)
        if cost > best.get(key, 1e9)+.01:
            continue
        expanded += 1
        close = [(math.hypot(pose[0]-a[0], pose[1]-a[1]), a)
                 for a in candidates if angle_error(pose[2], a[2]) <= 16]
        if close:
            distance, anchor = min(close, key=lambda item: item[0])
            if distance <= 2:
                path = [[pose[0], pose[1], pose[2]]]
                while key in next_pose:
                    key, successor = next_pose[key]
                    path.append([successor[0], successor[1], successor[2]])
                return {"id": label, "status": "FORWARD_PATH_CANDIDATE",
                        "start_road_edge": anchor[3],
                        "start_lane_xy_m": [anchor[0], anchor[1]],
                        "path_xy_heading": path, "length_m": len(path)*STEP,
                        "expanded_states": expanded}
        for steer in (-1, 0, 1):
            before = previous(pose, steer)
            if not (bounds[0] <= before[0] <= bounds[1]
                    and bounds[2] <= before[1] <= bounds[3]):
                continue
            midpoint = ((before[0]+pose[0])/2, (before[1]+pose[1])/2,
                        (before[2]+steer*TURN_DEG/2) % 360)
            if not clear(*before)[0] or not clear(*midpoint)[0]:
                continue
            prior_key = (round(before[0]), round(before[1]),
                         round(before[2]/TURN_DEG) % HEADING_COUNT)
            new_cost = cost+STEP+(.1 if steer else 0)
            if new_cost >= best.get(prior_key, 1e9)-.01:
                continue
            best[prior_key] = new_cost
            next_pose[prior_key] = (key, pose)
            counter += 1
            heapq.heappush(frontier, (new_cost+heuristic(before), new_cost,
                                      prior_key, before))
    return {"id": label, "status": "NO_FORWARD_PATH_IN_SEARCH",
            "expanded_states": expanded, "nearby_road_anchors": len(candidates)}


def route_exit(label, target, candidates):
    """Forward-only search from a berth back onto a legal road centreline."""
    def heuristic(p):
        return min(math.hypot(p[0]-a[0], p[1]-a[1]) for a in candidates)

    start_key = (round(target[0]), round(target[1]), round(target[2]/TURN_DEG) % HEADING_COUNT)
    frontier = [(heuristic(target), 0, start_key, target)]
    best = {start_key: 0}
    parent = {}
    expanded = 0
    bounds = (target[0]-55, target[0]+55, target[1]-55, target[1]+55)
    while frontier and expanded < MAX_EXPANSIONS:
        _, cost, key, pose = heapq.heappop(frontier)
        if cost > best.get(key, 1e9)+.01:
            continue
        expanded += 1
        close = [(math.hypot(pose[0]-a[0], pose[1]-a[1]), a)
                 for a in candidates if angle_error(pose[2], a[2]) <= 16]
        if close:
            distance, anchor = min(close, key=lambda item: item[0])
            if distance <= 2:
                path = [[pose[0], pose[1], pose[2]]]
                while key in parent:
                    key, predecessor = parent[key]
                    path.append([predecessor[0], predecessor[1], predecessor[2]])
                path.reverse()
                return {"id": label, "status": "FORWARD_PATH_CANDIDATE",
                        "end_road_edge": anchor[3],
                        "end_lane_xy_m": [anchor[0], anchor[1]],
                        "path_xy_heading": path, "length_m": len(path)*STEP,
                        "expanded_states": expanded}
        for steer in (-1, 0, 1):
            after = successor(pose, steer)
            if not (bounds[0] <= after[0] <= bounds[1]
                    and bounds[2] <= after[1] <= bounds[3]):
                continue
            midpoint = ((pose[0]+after[0])/2, (pose[1]+after[1])/2,
                        (pose[2]+steer*TURN_DEG/2) % 360)
            if not clear(*after)[0] or not clear(*midpoint)[0]:
                continue
            next_key = (round(after[0]), round(after[1]),
                        round(after[2]/TURN_DEG) % HEADING_COUNT)
            new_cost = cost+STEP+(.1 if steer else 0)
            if new_cost >= best.get(next_key, 1e9)-.01:
                continue
            best[next_key] = new_cost
            parent[next_key] = (key, pose)
            heapq.heappush(frontier, (new_cost+heuristic(after), new_cost,
                                      next_key, after))
    return {"id": label, "status": "NO_FORWARD_EXIT_IN_SEARCH",
            "expanded_states": expanded, "nearby_road_anchors": len(candidates)}


def route_reverse_entry(label, target, candidates):
    """Search backwards from a berth for a road pose that can back into it."""
    def heuristic(p):
        return min(math.hypot(p[0]-a[0], p[1]-a[1]) for a in candidates)

    start_key = (round(target[0]), round(target[1]), round(target[2]/TURN_DEG) % HEADING_COUNT)
    frontier = [(heuristic(target), 0, start_key, target)]
    best = {start_key: 0}
    next_pose = {}
    expanded = 0
    bounds = (target[0]-40, target[0]+40, target[1]-40, target[1]+40)
    while frontier and expanded < MAX_EXPANSIONS:
        _, cost, key, pose = heapq.heappop(frontier)
        if cost > best.get(key, 1e9)+.01:
            continue
        expanded += 1
        close = [(math.hypot(pose[0]-a[0], pose[1]-a[1]), a)
                 for a in candidates if angle_error(pose[2], a[2]) <= 16]
        if close:
            distance, anchor = min(close, key=lambda item: item[0])
            if distance <= 2:
                path = [[pose[0], pose[1], pose[2]]]
                while key in next_pose:
                    key, later = next_pose[key]
                    path.append([later[0], later[1], later[2]])
                return {"id": label, "status": "REVERSE_PATH_CANDIDATE",
                        "start_road_edge": anchor[3],
                        "start_lane_xy_m": [anchor[0], anchor[1]],
                        "path_xy_heading": path, "length_m": len(path)*STEP,
                        "expanded_states": expanded,
                        "maneuver": "reverse from lane pose to berth; forward approach to lane pose required"}
        if cost >= 30:
            continue
        for steer in (-1, 0, 1):
            # The predecessor lies forward of the berth. Backing from it to
            # the current pose traverses the same arc in the opposite order.
            before = successor(pose, steer)
            if not (bounds[0] <= before[0] <= bounds[1]
                    and bounds[2] <= before[1] <= bounds[3]):
                continue
            midpoint = ((before[0]+pose[0])/2, (before[1]+pose[1])/2,
                        (pose[2]+steer*TURN_DEG/2) % 360)
            if not clear(*before)[0] or not clear(*midpoint)[0]:
                continue
            prior_key = (round(before[0]), round(before[1]),
                         round(before[2]/TURN_DEG) % HEADING_COUNT)
            new_cost = cost+STEP+(.1 if steer else 0)
            if new_cost >= best.get(prior_key, 1e9)-.01:
                continue
            best[prior_key] = new_cost
            next_pose[prior_key] = (key, pose)
            heapq.heappush(frontier, (new_cost+heuristic(before), new_cost,
                                      prior_key, before))
    return {"id": label, "status": "NO_REVERSE_ENTRY_IN_SEARCH",
            "expanded_states": expanded, "nearby_road_anchors": len(candidates)}


points = [(s["id"]+"-"+kind, s[kind]) for s in plan["stations"]
          for kind in ("loading", "unloading", "waiting") if s[kind]]
points += [(p["id"], p) for p in plan["shared_waiting"]]
if 'points' in request:
    points = [(p['id'], p) for p in request['points']]
ids = set(request.get("ids", [label for label, _ in points]))
start = time.monotonic()
if request.get("mode") == "check_paths":
    paths = json.loads((ROOT/request["path_file"]).read_text())
    result = []
    for label, path in paths.items():
        active_target=label
        cache.clear()
        failures = []
        for a, b in zip(path, path[1:]):
            count = max(1, int(math.ceil(math.hypot(a[0]-b[0], a[1]-b[1])/.25)))
            dyaw = (b[2]-a[2]+180) % 360-180
            for i in range(count+1):
                u = i/count
                pose = (a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]), (a[2]+u*dyaw)%360)
                passed, reason = clear(*pose)
                if not passed:
                    failures.append({"pose": pose, "reason": reason})
                    break
            if failures:
                break
        result.append({"id": label, "status": "PASS" if not failures else "FAIL",
                       "failures": failures})
else:
    result = [route(label, point) for label, point in points if label in ids]
target_file.write_text(json.dumps({
    "read_only": True, "planner": "hybrid A* and static 9 m truck sweep",
    "mode": request.get("mode", "entry"),
    "step_m": STEP, "turn_deg": TURN_DEG,
    "ground_tolerance_m": GROUND_TOLERANCE,
    "ground_contact_is_diagnostic": GROUND_DIAGNOSTIC,
    "vehicle_envelope_m": [9, 3.3, 3.9], "elapsed_s": time.monotonic()-start,
    "results": result}, indent=2)+"\n")
