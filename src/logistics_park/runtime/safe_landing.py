"""Find a real safe holding pose before committing the next route section."""
import math
from logistics_park.common.messages import VehicleState
from logistics_park.safety.collision import overlaps
from logistics_park.scheduling.conflict_timing import maneuver_groups
from .transport_control import envelope
from .safety_timing import braking_distance

# An inbound truck further than this from its berth has not started the
# approach; its future outlet may be deferred behind a docked departure.
REMOTE_INBOUND_M=15.


def service_exit_sweeps(executor,node,*,full_bundle=False):
    """Validated immediate dock departure shapes, even before a lease exists.

    These exclude holding poses, not moving traffic or whole task routes.
    Cache distinct first action bundles to include alternate outbound ports.
    """
    from types import SimpleNamespace
    if not hasattr(executor,'landing_exit_sweeps'):executor.landing_exit_sweeps={}
    cache_key=(node,full_bundle)
    if cache_key in executor.landing_exit_sweeps:return executor.landing_exit_sweeps[cache_key]
    network=executor.dispatch.network;shapes={}
    for target,site in network.services.items():
        if target==node or site['kind'] not in ('loading','unloading','waiting'):continue
        route=network.route(node,target)
        if not route:continue
        departure=executor.geometry.plan(SimpleNamespace(origin=node,destination=target,route=route))
        # Reserve the dock's immediate access/recovery and its tail outlet.
        # A merged permission may also contain a later road intersection;
        # protecting that whole bundle here would displace queues into
        # unrelated upstream traffic before any truck needs to depart.
        end_index=0
        while end_index<len(departure.edges) and departure.model.edges[departure.edges[end_index]]['kind']=='service_access':
            end_index+=1
        if not end_index:continue
        exit_end=end_index
        arrival_start=None
        if full_bundle:
            groups=maneuver_groups(departure)
            if groups and groups[0][0]==0:end_index=groups[0][1]+1
            # A short trip may merge departure and another dock's arrival into
            # one permission. Hypothetical outlets must not reserve that other
            # private berth: its actual inbound leader can otherwise wait for
            # this dock's following truck. Keep the road outlet, while the
            # selected real departure below retains its entire permission.
            arrival_start=next((i for i in range(exit_end,end_index)
                if departure.model.edges[departure.edges[i]]['kind']=='service_access'),None)
            if arrival_start is not None:end_index=arrival_start
        key=tuple(departure.edges[:end_index])
        if key in shapes:continue
        special=any(departure.model.edges[k].get('reverse') or departure.model.edges[k]['kind']=='uturn'
                    for k in departure.edges[:end_index])
        end=min(departure.length,departure.edge_ends[end_index-1]+(11.344 if special else 6.))
        if arrival_start is not None:end=min(end,departure.edge_starts[arrival_start])
        count=max(1,math.ceil(end/.25))
        shapes[key]=[envelope(VehicleState('dock-exit',*departure.point(end*i/count),departure.heading(end*i/count)))
                     for i in range(count+1)]
    result=[body for sweep in shapes.values() for body in sweep]
    executor.landing_exit_sweeps[cache_key]=result
    return result


def terminal_parking_blockers_clear_soon(executor,vehicle,target):
    """Evidence for waiting at the owned source, never permission to depart."""
    dispatch=executor.dispatch
    evidence=getattr(executor,'departure_waits',{}).get((vehicle,target))
    if not evidence or evidence['time']!=dispatch.now or not evidence['blockers']:return False
    for row in evidence['blockers']:
        if row['reason'] not in ('occupied_landing_pose','reserved_landing_pose'):return False
        owner=row['blocked_by'];move=dispatch.moves.get(owner)
        ex=executor.controller.executions.get(owner);state=executor.latest_states.get(owner)
        if (move is None or getattr(move,'purpose',None)!='REPOSITION'
                or dispatch.network.services[move.destination]['kind']!='waiting'
                or dispatch.holders.get(move.destination)!=owner
                or ex is None or ex.finished or state is None):return False
        remaining=ex.plan.length-ex.progress
        speed=abs(state.speed)
        if not (math.isfinite(remaining) and math.isfinite(speed)
                and 0<=remaining<=20. and speed>.15 and remaining/speed<=30.):return False
    return True


def immediate_departure_sweep(executor,vehicle,plan):
    groups=maneuver_groups(plan)
    if not groups or groups[0][0]!=0:return []
    group=groups[0]
    special=any(plan.model.edges[plan.edges[i]].get('reverse') or plan.model.edges[plan.edges[i]]['kind']=='uturn'
                for i in range(group[0],group[1]+1))
    end=min(plan.length,group[3]+(11.344 if special else 6.))
    if not hasattr(executor,'parking_departure_sweeps'):executor.parking_departure_sweeps={}
    key=(plan.edges,end)
    if key not in executor.parking_departure_sweeps:
        count=max(1,math.ceil(end/.25))
        executor.parking_departure_sweeps[key]=[envelope(VehicleState(vehicle,*plan.point(end*i/count),plan.heading(end*i/count)))
                                               for i in range(count+1)]
    return executor.parking_departure_sweeps[key]


def parking_departure_clear(executor,vehicle,plan,states):
    """A terminal parking choice must have a clear immediate source action."""
    from logistics_park.scheduling.local_traffic import sweeps_intersect
    sweep=immediate_departure_sweep(executor,vehicle,plan)
    boxes=getattr(executor.controller,'boxes',{})
    return not any(other!=vehicle and abs(state.speed)<.15
                   and sweeps_intersect(sweep,boxes.get(other,[envelope(state)]),.35)
                   for other,state in states.items())


def source_exit_blockers(executor,vehicle,plan,states):
    from logistics_park.scheduling.local_traffic import sweeps_intersect
    sweep=immediate_departure_sweep(executor,vehicle,plan)
    boxes=getattr(executor.controller,'boxes',{})
    return [name for name,state in states.items() if name!=vehicle and abs(state.speed)<.15
            and sweeps_intersect(sweep,boxes.get(name,[envelope(state)]),.35)]


def source_admission_leaders(executor,vehicle,target,plan,progress,states):
    """Parked same-target bodies precede an unstarted source's queue intent."""
    dispatch=executor.dispatch
    if (progress>=.5 or vehicle not in dispatch.positions or vehicle not in states
            or abs(states[vehicle].speed)>=.15):return frozenset()
    candidates={other for other,assignment in dispatch.assignments.items()
                if other!=vehicle and assignment.state in ('READY','WAITING')
                and other in dispatch.positions and other not in executor.moves
                and dispatch._target(assignment)==target}
    if not candidates:return frozenset()
    return frozenset(candidates.intersection(source_exit_blockers(executor,vehicle,plan,states)))


def source_must_leave_before_inbound(executor,vehicle,owner,states):
    """A stationary inbound promise cannot freeze its actual source blocker.

    This changes only future exit prediction. Actual bodies, passage leases,
    target pose checks and the final controller's full sweeps stay mandatory.
    """
    dispatch=executor.dispatch
    # This dependency ordering is required by the default dispatcher too.
    # Disabling optional rolling optimization must not restore the circular
    # promise in which an inbound truck waits for the source it protects.
    move=dispatch.moves.get(owner);ex=executor.controller.executions.get(owner)
    a=dispatch.assignments.get(vehicle);source=getattr(dispatch,'positions',{}).get(vehicle)
    if (not move or ex is None or source is None or not a or ex.finished
            or a.state not in ('READY','WAITING') or dispatch.network.services[source]['kind'] not in ('loading','unloading')
            or dispatch.positions.get(owner)!=move.origin or ex.progress>=.5
            or abs(states[owner].speed)>=.15 or abs(states[vehicle].speed)>=.15):return False
    from logistics_park.scheduling.local_traffic import sweeps_intersect
    sweep=immediate_departure_sweep(executor,owner,ex.plan)
    boxes=getattr(executor.controller,'boxes',{})
    return bool(sweep and sweeps_intersect(sweep,boxes.get(vehicle,[envelope(states[vehicle])]),.35))


def docked_departure_has_priority(executor,vehicle,owner,states):
    """A truck standing in a berth outranks a remote inbound exit reserve.

    Keeping a queue off an inbound truck's future outlet is a promise about a
    dock that truck has not reached yet. Protecting that promise across the
    apron of an occupied berth closes a loop with no body in it: the docked
    truck may not go, the queued trucks may not enter, and the inbound truck
    may not approach. That holds whether the promise targets a neighbouring
    dock or this very berth. Only future exit prediction changes here; bodies,
    passage leases, target pose checks and the controller's full sweeps stay
    mandatory, exactly as for source_must_leave_before_inbound.
    """
    dispatch=executor.dispatch
    outbound=dispatch.moves.get(vehicle);inbound=dispatch.moves.get(owner)
    if not outbound or not inbound:return False
    dock=getattr(outbound,'origin',None)
    site=dispatch.network.services.get(dock) or {}
    if site.get('kind') not in ('loading','unloading'):return False
    rear=site.get('rear_xy_m');standing=states.get(vehicle);other=states.get(owner)
    if not rear or standing is None or other is None:return False
    # "Docked" is a measured pose at the berth, not a bookkeeping state: a
    # truck that already started leaving is no longer the occupant and must
    # not displace a live approach.
    if math.dist((standing.x,standing.y),tuple(rear))>4.:return False
    if abs(standing.speed)>=.15 or abs(other.speed)>=.15:return False
    target=dispatch.network.services.get(inbound.destination) or {}
    entry=target.get('rear_xy_m')
    if not entry:return False
    # A truck already inside the approach corridor really does need the outlet
    # next; only a remote reservation is deferred.
    return math.dist((other.x,other.y),tuple(entry))>=REMOTE_INBOUND_M


def parking_exit_conflicts(executor,target):
    """Dock exits which a stationary truck at target would obstruct."""
    if not hasattr(executor,'parking_exit_conflicts'):executor.parking_exit_conflicts={}
    if target not in executor.parking_exit_conflicts:
        from logistics_park.scheduling.local_traffic import sweeps_intersect
        network=executor.dispatch.network;site=network.services[target]
        body=envelope(VehicleState('parked-goal',*site['rear_xy_m'],math.radians(site['heading_deg'])))
        executor.parking_exit_conflicts[target]=frozenset(node for node,row in network.services.items()
            if node!=target and row['kind'] in ('loading','unloading')
            and sweeps_intersect([body],service_exit_sweeps(executor,node),.35))
    return executor.parking_exit_conflicts[target]


def selected_exit_plan(executor,node,owner):
    dispatch=executor.dispatch
    move=getattr(executor.dispatch,'moves',{}).get(owner)
    execution=getattr(executor.controller,'executions',{}).get(owner)
    if move and move.origin==node and execution is not None:return execution.plan
    assignment=getattr(dispatch,'assignments',{}).get(owner)
    if not assignment or assignment.work.loading_node!=node:return None
    # The next leg is already known while this truck is loading. Waiting for
    # its MOVE_REQUESTED event would let queues settle across that exit first.
    if not hasattr(executor,'pending_exit_plans'):executor.pending_exit_plans={}
    key=(owner,node,assignment.work.task_id)
    if key not in executor.pending_exit_plans:
        from types import SimpleNamespace
        target=assignment.work.unloading_node
        route=dispatch._route(node,target,owner,future=True)
        executor.pending_exit_plans[key]=(executor.geometry.plan(SimpleNamespace(origin=node,destination=target,route=route))
                                         if route else None)
    return executor.pending_exit_plans[key]


def occupied_exit_sweeps(executor,node,owner):
    """The selected departure's indivisible action, for stopped queue poses.

    A live source exit may merge access, a lane change and a junction. Its
    selected bundle must stay free of parked queues, while hypothetical
    departures retain the smaller static access protection.
    """
    plan=selected_exit_plan(executor,node,owner)
    if plan is None:
        # Unloading completion can remove the assignment before a parking
        # move is chosen. Do not briefly release the occupied dock's outlets
        # during that transition, or a queue can enter before the next tick.
        occupied=getattr(executor.dispatch,'holders',{}).get(node)==owner
        return service_exit_sweeps(executor,node,full_bundle=occupied)
    groups=maneuver_groups(plan)
    if not groups or groups[0][0]!=0:return service_exit_sweeps(executor,node)
    group=groups[0]
    special=any(plan.model.edges[plan.edges[i]].get('reverse') or plan.model.edges[plan.edges[i]]['kind']=='uturn'
                for i in range(group[0],group[1]+1))
    end=min(plan.length,group[3]+(11.344 if special else 6.))
    if not hasattr(executor,'occupied_exit_shapes'):executor.occupied_exit_shapes={}
    key=(plan.edges,end)
    if key not in executor.occupied_exit_shapes:
        count=max(1,math.ceil(end/.25))
        executor.occupied_exit_shapes[key]=[envelope(VehicleState(owner,*plan.point(end*i/count),plan.heading(end*i/count)))
                                          for i in range(count+1)]
    selected=executor.occupied_exit_shapes[key]
    move=getattr(executor.dispatch,'moves',{}).get(owner)
    if move is None or getattr(move,'origin',None)!=node:
        # Before loading finishes the outgoing route can still change with
        # congestion. Keep every immediate outlet clear, rather than letting
        # a queue rely on one provisional future route.
        if not hasattr(executor,'pending_exit_shapes'):executor.pending_exit_shapes={}
        pending=(node,key)
        if pending not in executor.pending_exit_shapes:
            executor.pending_exit_shapes[pending]=list(dict.fromkeys(
                selected+service_exit_sweeps(executor,node,full_bundle=True)))
        return executor.pending_exit_shapes[pending]
    return selected


def berth_exit_blockers(executor,target,vehicle):
    """Admission may not park a new truck across an occupied dock's exit."""
    dispatch=executor.dispatch;states=getattr(executor,'latest_states',{})
    if not states:return set()
    blocked=set()
    from logistics_park.scheduling.local_traffic import sweeps_intersect
    site=dispatch.network.services[target]
    goal=envelope(VehicleState(vehicle or 'parking-goal',*site['rear_xy_m'],math.radians(site['heading_deg'])))
    conflicts=set(parking_exit_conflicts(executor,target))
    if not hasattr(executor,'occupied_parking_conflicts'):executor.occupied_parking_conflicts={}
    for node,owner in dispatch.holders.items():
        if node!=target and dispatch.network.services[node]['kind'] in ('loading','unloading'):
            selected=selected_exit_plan(executor,node,owner)
            if selected is None:continue
            key=(target,node,selected.edges)
            if key not in executor.occupied_parking_conflicts:
                executor.occupied_parking_conflicts[key]=sweeps_intersect([goal],occupied_exit_sweeps(executor,node,owner),.35)
            if executor.occupied_parking_conflicts[key]:conflicts.add(node)
    for node in conflicts:
        owner=dispatch.holders.get(node)
        if owner in (None,vehicle) or owner not in states:continue
        site=dispatch.network.services[node]
        source=envelope(VehicleState(owner,*site['rear_xy_m'],math.radians(site['heading_deg'])))
        move=dispatch.moves.get(owner)
        assignment=getattr(dispatch,'assignments',{}).get(owner)
        pending_same_destination=bool(assignment
            and assignment.work.loading_node==node
            and assignment.work.unloading_node==target
            and not (move and move.origin==node))
        # A truck still loading cannot depart until it owns its unloading
        # berth. Protecting that future route against another arrival at the
        # same berth creates a circular wait. Its occupied source remains
        # protected by body separation and the normal departure admission.
        if pending_same_destination and overlaps(goal,envelope(states[owner]),.35):
            blocked.add(owner)
            continue
        if (pending_same_destination
                and not overlaps(goal,source,.35)
                and not overlaps(goal,envelope(states[owner]),.35)):
            continue
        entering=bool(move and move.destination==node and
                      dispatch.admission_priorities.get(owner,(0.,False))[1])
        if entering or overlaps(source,envelope(states[owner]),.35):blocked.add(owner)
    if blocked:
        if not hasattr(executor,'exit_blocker_log'):executor.exit_blocker_log={}
        key=(target,vehicle);previous=executor.exit_blocker_log.get(key)
        if previous is None or previous[0]!=blocked or dispatch.now-previous[1]>=5.:
            dispatch._event('BERTH_EXIT_ADMISSION_BLOCKED',vehicle_id=vehicle,node=target,
                           blocked_by=sorted(blocked),reason='parking_pose_blocks_occupied_exit')
            executor.exit_blocker_log[key]=(set(blocked),dispatch.now)
    return blocked


def safe_landing(executor,vehicle,plan,desired,states,*,progress=0.,speed=0.,target=None,traffic=True,protect_following=False,blocker_evidence=None):
    local=executor.controller.local_traffic
    groups=maneuver_groups(plan)
    protected=set()
    dispatch=getattr(executor,'dispatch',None)
    if dispatch and hasattr(executor,'geometry'):
        # A queue in front of an empty dock must also leave room for the
        # admitted truck's future departure. Do not require a current lease.
        if target and desired<plan.length-.5 and dispatch.network.services[target]['kind'] in ('loading','unloading'):
            protected.add(target)
        for node,owner in dispatch.holders.items():
            if not traffic:continue
            if owner==vehicle or owner not in states:continue
            site=dispatch.network.services[node]
            if site['kind'] not in ('loading','unloading'):continue
            source=envelope(VehicleState(owner,*site['rear_xy_m'],math.radians(site['heading_deg'])))
            move=getattr(dispatch,'moves',{}).get(owner)
            assignment=getattr(dispatch,'assignments',{}).get(owner)
            inbound_dock=bool(assignment and move and getattr(move,'destination',None)==node)
            if inbound_dock and source_must_leave_before_inbound(executor,vehicle,owner,states):
                inbound_dock=False
            if inbound_dock and docked_departure_has_priority(executor,vehicle,owner,states):
                inbound_dock=False
                if hasattr(dispatch,'_event'):
                    log=getattr(executor,'deferred_exit_reserve_log',{})
                    key=(vehicle,node);stamp=getattr(dispatch,'now',0.)
                    if stamp-log.get(key,-math.inf)>=5.:
                        dispatch._event('DOCKED_DEPARTURE_PRIORITY',vehicle_id=vehicle,
                            reserved_by=owner,node=node)
                        log[key]=stamp
                    executor.deferred_exit_reserve_log=log
            # Leave the outlet free before an admitted loader or unloader arrives;
            # a following truck may pass its last safe stop before docking.
            if inbound_dock or overlaps(source,envelope(states[owner]),.35):protected.add(node)
    exit_shapes={node:(
        occupied_exit_sweeps(executor,node,dispatch.holders[node]) if traffic and node in dispatch.holders
        else service_exit_sweeps(executor,node)) for node in sorted(protected)}
    exits=[body for shapes in exit_shapes.values() for body in shapes]
    def record_blocker(owner,reason,resource):
        if blocker_evidence is not None and owner is not None and owner!=vehicle:
            row=dict(blocked_by=owner,reason=reason,resource=resource)
            if row not in blocker_evidence:blocker_evidence.append(row)
    if traffic and exits and (protect_following or desired<plan.length-.5):
        # A safe queue head does not make its following positions safe. Check
        # where this truck would actually stop behind a same-lane leader,
        # including a leader still approaching its confirmed holding line.
        executions=getattr(executor.controller,'executions',{})
        saved=getattr(executor,'safe_landing_states',{})
        for other,state in states.items():
            if other==vehicle:continue
            error,station=plan.project((state.x,state.y),progress,behind=0.,ahead=80.)
            angle=abs((state.heading-plan.heading(station)+math.pi)%(2*math.pi)-math.pi)
            if error>1.4 or angle>math.pi/3 or station<=progress+.1:continue
            leader=executions.get(other)
            hold=saved.get((other,leader.plan.edges)) if leader else None
            if hold is not None and not (protect_following and abs(state.speed)<.15):
                error,station=plan.project(leader.plan.point(hold),desired,behind=100.,ahead=10.)
                if error>1.4:continue
            elif abs(state.speed)>=.15:continue
            following=station-9.544
            if progress<following<desired:desired=following
    candidates={desired}
    candidates.update(max(0.,g[2]-7.8) for g in groups if g[2]-7.8<desired)
    minimum=progress+braking_distance(speed)
    for station in sorted(candidates,reverse=True):
        if station<minimum:continue
        # A vehicle cannot promise to finish a maneuver and then stop inside
        # its required outlet. Those actions must be approved as one section.
        if any(g[2]<station and min(plan.length,g[3]+(11.344 if any(
            plan.model.edges[plan.edges[i]].get('reverse') or plan.model.edges[plan.edges[i]]['kind']=='uturn'
            for i in range(g[0],g[1]+1)) else 6.))>station+1e-6 for g in groups):continue
        body=envelope(VehicleState(vehicle,*plan.point(station),plan.heading(station)))
        from logistics_park.scheduling.local_traffic import sweeps_intersect
        exit_blockers=[node for node,shapes in exit_shapes.items() if sweeps_intersect([body],shapes,.35)]
        if exit_blockers:
            for node in exit_blockers:record_blocker(dispatch.holders.get(node),'occupied_exit_landing',node)
            continue
        physical_blockers=[other for other,state in states.items() if traffic and other!=vehicle
            and abs(state.speed)<.15 and overlaps(body,envelope(state),.35)]
        if physical_blockers:
            for owner in physical_blockers:record_blocker(owner,'occupied_landing_pose',target)
            continue
        passage_blockers=[(owner,start) for (owner,start),lease in local.passages.items()
            if owner!=vehicle and any(overlaps(body,b,.35) for b in lease['sweep'])] if traffic and local else []
        if passage_blockers:
            for owner,start in passage_blockers:record_blocker(owner,'reserved_landing_pose','passage:'+str(start))
            continue
        if blocker_evidence is not None and station>=desired-.35:blocker_evidence.clear()
        return station
    return None
