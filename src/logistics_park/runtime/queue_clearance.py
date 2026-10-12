"""Temporary off-road clearance, preserving the executed standard trajectory."""
import math
from dataclasses import replace

def refresh_requirements(executor):
    d=executor.dispatch;c=executor.controller;d.live_requirements={}
    for name,move in executor.moves.items():
        ex=c.executions[name];rows=[c.model.edges[key] for i,key in enumerate(ex.plan.edges)
            if ex.plan.edge_ends[i]>=ex.progress-11.344 and ex.plan.edge_starts[i]<=ex.progress+80.]
        d.live_requirements[name]=dict(required_resources={r for row in rows for r in row.get('required_resources',[])},
            must_be_empty_nodes={n for row in rows for n in row.get('must_be_empty_nodes',[])})

def blocked_followers(controller,states,name):
    """Actual same-direction followers, plus observed crossing dependants."""
    front=states[name];followers=set()
    for other,state in states.items():
        if other==name or controller.executions[other].finished:continue
        ex=controller.executions[other]
        if math.dist((state.x,state.y),(front.x,front.y))>35:continue
        error,station=ex.plan.project((front.x,front.y),ex.progress,behind=0,ahead=35.)
        angle=abs((front.heading-ex.plan.heading(station)+math.pi)%(2*math.pi)-math.pi)
        if error<1.4 and angle<math.pi/3 and station>ex.progress+.1:followers.add(other)
    local=controller.local_traffic
    followers.update(other for other,denial in local.last_denials.items() if denial.get('blocked_by')==name)
    followers.update(other for other,denials in local.passage_denials.items()
        if any(denial.get('blocked_by')==name for denial in denials))
    return followers

def clear_stopped_queues(executor,states,now):
    d=executor.dispatch;c=executor.controller
    if not c.local_traffic:return
    d.waiting_lenders={}
    for name,move in executor.moves.items():
        if d.efficiency_policy:continue  # A committed parking target is not a lendable remote berth promise.
        ex=c.executions[name];index=len(ex.plan.edges)-1
        while index>0 and c.model.edges[ex.plan.edges[index-1]]['kind']=='service_access':index-=1
        if (move.waiting and d.holders.get(move.destination)==name and d.positions.get(name)!=move.destination
                and abs(states[name].speed)<.08 and not ex.finished
                and ex.progress<ex.plan.edge_starts[index]-12. and ex.plan.length-ex.progress>40.
                and c.model.edges[ex.plan.edges[ex.plan.edge_index(ex.progress)]]['kind']=='road'
                and not any(key[0]==name for key in c.local_traffic.passages)):
            d.waiting_lenders[name]=ex.plan.length-ex.progress
    if not hasattr(executor,'clearance_stopped_since'):executor.clearance_stopped_since={};executor.clearance_attempts={}
    _clear_parked_waiting(executor,states,now)
    for name,state in states.items():
        policy=d.efficiency_policy
        move=executor.moves.get(name)
        proactive=bool(policy and policy.proactive_admission and move and not move.waiting
            and d._blockers(move.destination,name))
        if abs(state.speed)>.08 and not proactive:executor.clearance_stopped_since.pop(name,None);continue
        since=executor.clearance_stopped_since.setdefault(name,now)
        followers=blocked_followers(c,states,name) if d.offroad_busy_wait else set()
        if (not proactive and now-since<(3 if followers else 10)) or now-executor.clearance_attempts.get(name,-100)<5:continue
        move=executor.moves.get(name);ex=c.executions[name]
        denial=c.local_traffic.last_denials.get(name,{})
        body_blocker=denial.get('blocked_by') if denial.get('reason')=='predicted_body' else None
        if body_blocker is None:
            body_blocker=next((row['blocked_by'] for row in c.local_traffic.passage_denials.get(name,[])
                if row['reason']=='passage_occupied_body'),None)
        body_blocked=body_blocker in states and abs(states[body_blocker].speed)<.08
        policy=d.efficiency_policy
        persistent=body_blocked and now-since>= (policy.waiting_commit_s if policy else 60.)
        if policy and move and move.waiting and not persistent:continue
        if (not move or not (d._blockers(move.destination,name) or body_blocked or followers)
                or ex.finished or any(key[0]==name for key in c.local_traffic.passages)
                or c.model.edges[ex.plan.edges[ex.plan.edge_index(ex.progress)]]['kind']!='road'):continue
        target=d._target(d.assignments[name]) if name in d.assignments else move.destination
        executor.clearance_attempts[name]=now
        options=[];station=0.;prefix=[];network=d.network;rejections={}
        def reject(reason):rejections[reason]=rejections.get(reason,0)+1
        for key in move.route['edge_ids']:
            prefix.append(key);station+=network.edges[key]['length_m']
            if station<ex.progress+6:continue
            if station>ex.progress+80:break
            edge=network.edges[key]
            if edge['kind']!='road':continue
            for q,site in network.services.items():
                if site['kind']!='waiting':continue
                if policy and not d.can_divert_waiting(move,q,persistent_block=persistent):
                    reject('quarantine_or_target_commit');continue
                lender=d.holders.get(q)
                if (site['kind']!='waiting' or q==move.destination or (lender and lender not in d.waiting_lenders)
                        or d._blocks_committed_route(q,name,frozenset({lender}) if lender else frozenset())):
                    reject('occupied_or_committed');continue
                if any(n in d.holders and d.holders[n]!=name for n in site.get('departure_requires_empty',[])):
                    reject('blocked_exit');continue
                occupied=d._occupied(name)
                suffix=network.route(edge['to_node'],q,incoming_edge=key,occupied_nodes=occupied,
                    transit_warehouses=frozenset())
                if not suffix:reject('no_legal_entry');continue
                # Check a legal future departure, not an immediately empty
                # trip through all busy service bays. Clearing this front is
                # precisely what allows their current owners to depart.
                future_occupied={n for n in occupied if network.services[n]['kind'] not in ('loading','unloading')}
                onward=network.route(q,target,occupied_nodes=future_occupied,
                    transit_warehouses=d._traffic_warehouses(q,target))
                if not onward:reject('no_legal_exit');continue
                keys=prefix+suffix['edge_ids']
                route=dict(suffix,edge_ids=keys,distance_m=sum(network.edges[k]['length_m'] for k in keys),
                    estimated_travel_time_s=sum(network.edges[k]['estimated_travel_time_s'] for k in keys),
                    required_resources=sorted({r for k in keys for r in network.edges[k].get('required_resources',[])}),
                    must_be_empty_nodes=sorted({n for k in keys for n in network.edges[k].get('must_be_empty_nodes',[])}))
                if not d._parking_approach_clear(suffix,name):reject('dock_body_blocks_entry');continue
                remaining=route['distance_m']-ex.progress
                if policy:
                    prefix_time=0.;cursor=0.
                    for edge_id in keys:
                        row=network.edges[edge_id];end=cursor+row['length_m']
                        fraction=max(0.,end-max(cursor,ex.progress))/max(1e-9,row['length_m'])
                        prefix_time+=fraction*row['estimated_travel_time_s'];cursor=end
                    extra=max(0.,remaining+onward['distance_m']-(move.route['distance_m']-ex.progress))
                    recovery=d.recovery_estimates.get(name,math.inf)
                    if not policy.affordable(remaining,extra,prefix_time):reject('detour_or_eta_budget');continue
                    if math.isfinite(recovery) and prefix_time+policy.diversion_gain_s>=max(0.,recovery-now):
                        reject('original_route_clears_sooner');continue
                    route=dict(route,parking_eta_s=prefix_time,waiting_detour_m=extra,
                               original_recovery_eta_s=recovery-now if math.isfinite(recovery) else None)
                if lender and remaining+10>=d.waiting_lenders[lender]:continue
                score=remaining+onward['distance_m'] if d.offroad_busy_wait else remaining
                options.append((score,q,route,lender))
        if policy and not options:
            d._event('QUEUE_BUFFER_UNAVAILABLE',vehicle_id=name,target=target,
                followers=sorted(followers),rejections=rejections,
                parking_eta_limit_s=policy.waiting_eta_s,detour_limit_m=policy.waiting_extra_m)
        for _,q,route,lender in sorted(options,key=lambda row:(row[0],row[1])):
            try:
                plan=executor.geometry.plan(replace(move,destination=q,route=route,waiting=True))
                if lender and plan.length-ex.progress+10>=d.waiting_lenders[lender]:continue
                if proactive:c.reroute_forward(name,plan,now,state=state)
                else:c.reroute_stopped(name,plan,now,state=state)
            except ValueError:continue
            executor.moves[name]=d.divert_to_waiting(name,q,route,reassign_from=lender,persistent_block=persistent)
            if policy:d._event('WAITING_DIVERSION_COST',vehicle_id=name,waiting_node=q,
                parking_eta_s=route['parking_eta_s'],extra_m=route['waiting_detour_m'],
                recovery_eta_s=route['original_recovery_eta_s'])
            if followers:
                if name in d.assignments and d.traffic_routing is not None:d.assignments[name].traffic_wait_since=now
                d._event('ROAD_BLOCKING_WAIT_DIVERTED',vehicle_id=name,followers=sorted(followers),waiting_node=q,task_id=move.task_id)
            d.waiting_lenders.pop(name,None)
            if lender:d.waiting_lenders.pop(lender,None)
            refresh_requirements(executor)
            break


def _clear_parked_waiting(executor,states,now):
    """Evacuate an assigned waiting truck that blocks an issued maneuver."""
    from .transport_control import envelope
    from logistics_park.scheduling.task_dispatch import DispatchMove
    d=executor.dispatch;c=executor.controller
    blocked={row['blocked_by'] for rows in c.local_traffic.passage_denials.values()
        for row in rows if row['reason']=='passage_occupied_body'}
    for name in sorted(blocked):
        a=d.assignments.get(name);origin=d.positions.get(name)
        if (not a or a.state not in ('WAITING','READY') or origin is None
                or d.network.services[origin]['kind']!='waiting' or name in executor.moves
                or not c.executions[name].finished or abs(states[name].speed)>.08
                or now-executor.clearance_attempts.get(name,-100)<10.):continue
        target=d._target(a);options=[]
        for q,site in d.network.services.items():
            if d.efficiency_policy and not d.efficiency_policy.parking_enabled(q):continue
            lender=d.holders.get(q)
            if (site['kind']!='waiting' or q==origin or (lender and lender not in d.waiting_lenders)
                    or d._blockers(q,name)-({lender} if lender else set())
                    or d._blocks_committed_route(q,name,frozenset({lender}) if lender else frozenset())):continue
            if any(n in d.holders and d.holders[n]!=name for n in site.get('departure_requires_empty',[])):continue
            route=d._route(origin,q,name)
            onward=d._route(q,target,name,future=True)
            if not route or not onward:continue
            if d.efficiency_policy:
                direct=d._route(origin,target,name,future=True)
                if not direct or not d.efficiency_policy.affordable(route['distance_m'],
                        max(0.,route['distance_m']+onward['distance_m']-direct['distance_m']),
                        route['estimated_travel_time_s']):continue
            if not d._parking_approach_clear(route,name):continue
            if lender and route['distance_m']+10>=d.waiting_lenders[lender]:continue
            options.append((route['estimated_travel_time_s'],q,route,lender))
        executor.clearance_attempts[name]=now
        if not options:continue
        selected=None
        for _,q,route,lender in sorted(options,key=lambda row:(row[0],row[1])):
            try:
                candidate=DispatchMove(d.sequence+1,a.work.task_id,name,a.visit_id,origin,q,route,True)
                plan=executor.geometry.plan(candidate)
            except ValueError:continue
            if math.dist((states[name].x,states[name].y),plan.point(0))>.5:continue
            if abs((states[name].heading-plan.heading(0)+math.pi)%(2*math.pi)-math.pi)>.35:continue
            selected=q,route,plan,lender;break
        if selected is None:continue
        q,route,plan,lender=selected
        if lender:
            del d.holders[q]
            d._event('WAITING_RESERVATION_REASSIGNED',vehicle_id=name,previous_unentered_holder=lender,node=q)
            d.waiting_lenders.pop(lender,None)
        move=d._issue(a,q,route,True)
        c.continue_route(name,plan,now,state=states[name])
        executor.moves[name]=move;executor.sources[name]=envelope(states[name])
        d._event('ASSIGNED_WAITING_VEHICLE_CLEARED',vehicle_id=name,task_id=a.work.task_id,origin=origin,destination=q)
        refresh_requirements(executor)
