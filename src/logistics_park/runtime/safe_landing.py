"""Find a real safe holding pose before committing the next route section."""
import math
from logistics_park.common.messages import VehicleState
from logistics_park.safety.collision import overlaps
from logistics_park.scheduling.conflict_timing import maneuver_groups
from .transport_control import envelope


def service_exit_sweeps(executor,node):
    """Validated immediate dock departure shapes, even before a lease exists.

    These exclude holding poses, not moving traffic or whole task routes.
    Cache distinct first action bundles to include alternate outbound ports.
    """
    from types import SimpleNamespace
    if not hasattr(executor,'landing_exit_sweeps'):executor.landing_exit_sweeps={}
    if node in executor.landing_exit_sweeps:return executor.landing_exit_sweeps[node]
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
        key=tuple(departure.edges[:end_index])
        if key in shapes:continue
        special=any(departure.model.edges[k].get('reverse') or departure.model.edges[k]['kind']=='uturn'
                    for k in departure.edges[:end_index])
        end=min(departure.length,departure.edge_ends[end_index-1]+(11.344 if special else 6.))
        count=max(1,math.ceil(end/.25))
        shapes[key]=[envelope(VehicleState('dock-exit',*departure.point(end*i/count),departure.heading(end*i/count)))
                     for i in range(count+1)]
    result=[body for sweep in shapes.values() for body in sweep]
    executor.landing_exit_sweeps[node]=result
    return result


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
    move=getattr(dispatch,'moves',{}).get(owner)
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
    if plan is None:return service_exit_sweeps(executor,node)
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
    return executor.occupied_exit_shapes[key]


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


def safe_landing(executor,vehicle,plan,desired,states,*,progress=0.,speed=0.,target=None,traffic=True):
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
            if overlaps(source,envelope(states[owner]),.35):protected.add(node)
    exits=[body for node in sorted(protected) for body in (
        occupied_exit_sweeps(executor,node,dispatch.holders[node]) if traffic and node in dispatch.holders
        else service_exit_sweeps(executor,node))]
    if traffic and exits and desired<plan.length-.5:
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
            if hold is not None:
                error,station=plan.project(leader.plan.point(hold),desired,behind=100.,ahead=10.)
                if error>1.4:continue
            elif abs(state.speed)>=.15:continue
            following=station-9.544
            if progress<following<desired:desired=following
    candidates={desired}
    candidates.update(max(0.,g[2]-7.8) for g in groups if g[2]-7.8<desired)
    minimum=progress+abs(speed)**2/1.2+abs(speed)*.2+.5
    for station in sorted(candidates,reverse=True):
        if station<minimum:continue
        # A vehicle cannot promise to finish a maneuver and then stop inside
        # its required outlet. Those actions must be approved as one section.
        if any(g[2]<station and min(plan.length,g[3]+(11.344 if any(
            plan.model.edges[plan.edges[i]].get('reverse') or plan.model.edges[plan.edges[i]]['kind']=='uturn'
            for i in range(g[0],g[1]+1)) else 6.))>station+1e-6 for g in groups):continue
        body=envelope(VehicleState(vehicle,*plan.point(station),plan.heading(station)))
        from logistics_park.scheduling.local_traffic import sweeps_intersect
        if sweeps_intersect([body],exits,.35):continue
        if traffic and any(other!=vehicle and abs(state.speed)<.15 and overlaps(body,envelope(state),.35)
               for other,state in states.items()):continue
        if traffic and local and any(owner!=vehicle and any(overlaps(body,b,.35) for b in lease['sweep'])
                         for (owner,_),lease in local.passages.items()):continue
        return station
    return None
