"""Bind task commands to the existing, checked service-route geometry."""
import math
from logistics_park.scheduling.maneuver_contract import holding_line, HOLD_BEFORE_ENTRY_M, ActionContract
from logistics_park.map.geometry import Polyline
from .transport_routes import RouteGeometry


class ValidatedServiceRoutes:
    """A conservative bridge: an unbound route must be validated separately.

    This first bridge covers the current 132 loading/unloading itineraries.
    Waiting-node movements need their own geometry manifest before UE execution.
    It never substitutes another route when dispatch chooses a blocked-node detour.
    """
    def __init__(self, network, model, jobs):
        self.model=model
        self.bindings={}
        for job in jobs:
            route=network.route(job['origin'],job['destination'])
            if route is None:raise ValueError('Manifest route is no longer reachable')
            self._check_geometry(network, route, job)
            self.bindings[(job['origin'],job['destination'])]=(tuple(route['edge_ids']),job)

    def plan(self, move):
        binding=self.bindings.get((move.origin,move.destination))
        if binding is None or tuple(move.route['edge_ids'])!=binding[0]:
            raise ValueError('Dispatch route has no validated UE geometry binding')
        job=binding[1]
        return RouteGeometry(self.model,job['edges'],job['checkpoints'],move.origin,[move.destination])

    def _check_geometry(self, network, route, job):
        # Audit the existing <=5 m slices against today's source centerlines.
        # Only a removed <15 cm attachment may change the preceding endpoint.
        expected=[]
        for key in route['edge_ids']:
            e=network.edges[key]
            if e['kind']=='service_access' and e['phase'].endswith('attachment') and e['length_m']<.15:continue
            line=Polyline(tuple(map(tuple,e['centerline'])))
            count=max(1,math.ceil(e['length_m']/5))
            for i in range(count):
                expected.append((e,line.at(e['length_m']*i/count)[0],line.at(e['length_m']*(i+1)/count)[0]))
        if len(expected)!=len(job['edges']):raise ValueError('Stale service-route geometry manifest')
        for (source,start,end),key in zip(expected,job['edges']):
            row=self.model.edges[key]
            if (row['capacity_alias']!=source['capacity_alias'] or row['kind']!=source['kind']
                    or bool(row.get('reverse'))!=bool(source.get('reverse'))
                    or row.get('maneuver')!=source.get('maneuver')
                    or row['speed_limit']!=(8 if source['kind']=='road' else source['speed_limit'])
                    or math.dist(row['centerline'][0],start)>1e-6
                    or math.dist(row['centerline'][-1],end)>(.150001 if key.startswith('pair_snap_') else 1e-6)):
                raise ValueError('Stale service-route geometry manifest')
            path=Polyline(tuple(map(tuple,source['centerline'])))
            if any(path.project(*p).distance>1e-6 for p in row['centerline'][1:-1]):
                raise ValueError('Stale service-route interior geometry')


class ServiceRouteGeometry:
    """Compose exact current service edges without enumerating node pairs.

    All long edges use the same <=5 m slices as the road scan. Only tiny
    attachment stubs may be removed, with an itinerary-local endpoint snap.
    """
    def __init__(self,network,model):
        self.network=network;self.model=model;self.cache={};self.checked_parts=set()
        self.parts={}
        for key in network.edges:
            pieces=[k for k in model.edges if k.startswith(key+'__test') and k[len(key+'__test'):].isdigit()]
            if pieces:self.parts[key]=sorted(pieces,key=lambda k:int(k.rsplit('__test',1)[1]))

    def plan(self,move):
        import copy,hashlib
        keys=tuple(move.route['edge_ids'])
        if not keys:raise ValueError('Empty movement route')
        if (self.network.edges[keys[0]]['from_node']!=move.origin or self.network.edges[keys[-1]]['to_node']!=move.destination
                or any(b not in self.network.successors[a] for a,b in zip(keys,keys[1:]))):
            raise ValueError('Illegal service movement sequence')
        if any(k in getattr(self.network,'blocked_edges',()) or self.network.edges[k].get('phase','').endswith('_candidate') for k in keys):raise ValueError('Disabled execution alternative')
        if any(self.network.edges[k].get('attachment_validation_required') or self.network.edges[k].get('access_validation_required') for k in keys):
            raise ValueError('Pending access cannot be executed')
        if keys in self.cache:return self.cache[keys]
        tag=hashlib.sha256(repr(keys).encode()).hexdigest()[:16]
        expanded=[]
        for key in keys:
            source=self.network.edges[key]
            if key not in self.parts:
                if source['kind']=='service_access' and source['phase'].endswith('attachment') and source['length_m']<.15:continue
                raise ValueError('Missing service edge geometry '+key)
            if key not in self.checked_parts:
                line=Polyline(tuple(map(tuple,source['centerline'])))
                count=max(1,math.ceil(source['length_m']/5))
                if count!=len(self.parts[key]):raise ValueError('Stale source slices '+key)
                for i,part in enumerate(self.parts[key]):
                    row=self.model.edges[part]
                    if (math.dist(row['centerline'][0],line.at(source['length_m']*i/count)[0])>1e-6
                            or math.dist(row['centerline'][-1],line.at(source['length_m']*(i+1)/count)[0])>1e-6
                            or bool(row.get('reverse'))!=bool(source.get('reverse'))
                            or row['capacity_alias']!=source['capacity_alias']
                            or any(line.project(*p).distance>1e-6 for p in row['centerline'][1:-1])):
                        raise ValueError('Stale source geometry '+key)
                self.checked_parts.add(key)
            for part in self.parts[key]:
                nxt=self.model.edges[part]
                if expanded:
                    previous=self.model.edges[expanded[-1]]
                    gap=math.dist(previous['centerline'][-1],nxt['centerline'][0])
                    if gap>.15:raise ValueError('Unsafe access join')
                    if previous['to_node']!=nxt['from_node'] or gap>1e-6:
                        clone=copy.deepcopy(previous);clone_id='dispatch_snap_'+tag+'_'+str(len(expanded))
                        clone.update(edge_id=clone_id,to_node=nxt['from_node'])
                        clone['centerline'][-1]=list(nxt['centerline'][0]);clone['endpoint_snap_m']=gap
                        self.model.edges[clone_id]=clone;self.model.aliases[clone_id]=clone['capacity_alias']
                        for spec in self.model.shared_resources.values():
                            if expanded[-1] in spec['edge_ids']:spec['edge_ids'].append(clone_id)
                        expanded[-1]=clone_id
                expanded.append(part)
        checkpoints=[]
        for i,(a,b) in enumerate(zip(expanded,expanded[1:]),1):
            if bool(self.model.edges[a].get('reverse'))!=bool(self.model.edges[b].get('reverse')):
                checkpoints.append(dict(event='GEAR_CHANGE',after_edge_count=i,direction=-1 if self.model.edges[b].get('reverse') else 1))
        checkpoints.append(dict(event='ARRIVAL',after_edge_count=len(expanded),leg=0,site=move.destination,node=move.destination))
        plan=RouteGeometry(self.model,expanded,checkpoints,move.origin,[move.destination])
        plan.arrival_heading=math.radians(self.network.services[move.destination]['heading_deg'])
        # Exact source and destination headings are part of the service contract.
        for node,s in ((move.origin,0),(move.destination,plan.length)):
            site=self.network.services[node];wanted=math.radians(site['heading_deg'])
            if math.dist(plan.point(s),site['rear_xy_m'])>.15 or abs((plan.heading(s)-wanted+math.pi)%(2*math.pi)-math.pi)>.35:
                raise ValueError('Incompatible service-node pose '+node)
        self.cache[keys]=plan
        return plan

    def stationary(self,node):
        """A parked vehicle stays in occupancy and all footprint safety checks."""
        from logistics_park.map.geometry import Polyline
        from .transport_control import envelope,zone_box
        from logistics_park.common.messages import VehicleState
        from logistics_park.safety.collision import overlaps
        site=self.network.services[node];x,y=site['rear_xy_m'];h=math.radians(site['heading_deg'])
        key='parked_'+node;alias='berth:'+node
        if key not in self.model.edges:
            points=[[x,y],[x+.02*math.cos(h),y+.02*math.sin(h)]]
            self.model.edges[key]=dict(edge_id=key,from_node=node,to_node=node+'-parked',centerline=points,
                capacity_alias=alias,kind='service_access',maneuver='STRAIGHT',reverse=False,speed_limit=0,maximum_path_error_m=.7)
            self.model.aliases[key]=alias;self.model.physical_paths[alias]=Polyline(tuple(map(tuple,points)))
            self.model.raw['physical_capacities'][alias]=1
            body=envelope(VehicleState('parked',x,y,h))
            for spec in self.model.shared_resources.values():
                if 'bounds' in spec and overlaps(body,zone_box(spec),.8):spec['edge_ids'].append(key)
        return RouteGeometry(self.model,[key],[],node,[])


class ContinuousTaskExecutor:
    """Observed leg transitions, task callbacks and persistent road occupancy."""
    def __init__(self,dispatch,geometry,controller):
        if not controller.retain_finished_vehicles:raise ValueError('Persistent vehicle occupancy required')
        self.dispatch=dispatch;self.geometry=geometry;self.controller=controller
        self.moves={};self.sources={};self.completed_moves=[];self.safe_landing_states={};self.landing_rejections={}
        from logistics_park.scheduling.travel_time import TravelTimeEstimator
        self.travel_time=TravelTimeEstimator();self.eta_log_at=-math.inf
        from logistics_park.scheduling.eta_service import ETAService
        self.eta_service=ETAService();self.eta_query_logged_at={}
        self.dispatch.route_eta_predictor=self.travel_time.estimate_route
        if self.dispatch.global_scheduler is not None:self.dispatch.global_scheduler.prepare(self.dispatch)
        self.move_measurements={}
        if controller.local_traffic:
            controller.local_traffic.protect_connected_merges=dispatch.scheduling.connected_clearance
            controller.local_traffic.check_diversion_room=dispatch.scheduling.connected_clearance
        if dispatch.efficiency_policy and controller.local_traffic:
            controller.local_traffic.compact_permissions=True
            controller.local_traffic.proactive_admission=dispatch.efficiency_policy.proactive_admission
            if dispatch.efficiency_policy.proactive_admission:
                from .safe_landing import parking_exit_conflicts
                dispatch.departure_guard=self._departure_guard
                dispatch.parking_departure_guard=self._parking_departure_guard
                dispatch.parking_wait_guard=self._parking_wait_guard
                dispatch.berth_exit_blockers=self._berth_exit_blockers
                # Compile static parking/exit conflicts before the first
                # measured control tick, not while moving vehicles await it.
                for node,site in dispatch.network.services.items():
                    if site['kind'] in ('loading','unloading','waiting'):parking_exit_conflicts(self,node)

    def prepare_schedule(self,states):
        """Warm advisory profiles and route caches while the fleet is stopped."""
        planner=getattr(self.dispatch,'global_scheduler',None)
        if planner is None:return
        import time
        started=time.monotonic();self.latest_states=states
        if self.dispatch.traffic_routing:
            self.dispatch.traffic_routing.observe(states,self.controller,self.dispatch.positions,0.,moves={})
        budget=planner.budget_s
        try:
            planner.budget_s=max(1.,budget)
            planner.plan(self.dispatch,list(self.dispatch.tasks.values()))
        finally:planner.budget_s=budget
        self.dispatch._event('ROLLING_CACHE_PREPARED',elapsed_s=time.monotonic()-started,
                             physical_leases_changed=False,movement_commands_issued=False)

    def attach(self,vehicle,node,state,now,*,body_boxes=None):
        from .transport_control import Execution,envelope
        from logistics_park.safety.collision import overlaps
        if vehicle in self.controller.executions or self.dispatch.positions.get(vehicle)!=node:
            raise ValueError('Attach requires a confirmed unique parked vehicle')
        plan=self.geometry.stationary(node)
        if abs(state.speed)>.1 or math.dist((state.x,state.y),plan.point(0))>.5:
            raise ValueError('Attach requires actual stopped node observation')
        boxes=body_boxes or [envelope(state)]
        touched={k for k,area in self.controller.shared_zones.items() if any(overlaps(b,area) for b in boxes)}
        if any(self.controller.shared_scheduler.owners.get(k) not in (None,vehicle) for k in touched):
            raise ValueError('Parked vehicle overlaps an already-owned road area')
        for key in sorted(touched):self.controller.shared_scheduler.request(key,vehicle,now)
        self.controller.plans[vehicle]=plan
        self.controller.executions[vehicle]=Execution(plan,finished=True,last_progress_time=now)
        self.controller.capacity.capacities.update(self.controller.model.raw['physical_capacities'])
        self.controller.events.append(dict(time=now,event='PARKED_VEHICLE_ATTACHED',vehicle_id=vehicle,node=node))

    def tick(self,states,now,*,body_boxes=None):
        from .transport_control import envelope,ARRIVAL_POSITION_TOLERANCE_M
        from logistics_park.safety.collision import overlaps
        if set(states)!=set(self.controller.executions):raise ValueError('All retained vehicles require fresh observations')
        self.dispatch._time(now)
        self.latest_states=states
        if self.dispatch.efficiency_policy:
            local=self.controller.local_traffic
            if local:local.eta_speed_scales=self.travel_time.scale
            self.travel_time.observe(states,self.controller.executions,local.last_targets if local else {},now)
            self.dispatch.recovery_estimates={}
            if local:
                for name,window in getattr(getattr(local,'timing',None),'windows',{}).items():
                    delay=window.get('delay_s',0.);blockers=window.get('blocked_by',[])
                    if blockers and all(abs(states[n].speed)>.2 for n in blockers) and math.isfinite(delay):
                        self.dispatch.recovery_estimates[name]=now+delay
            if now-self.eta_log_at>=5.:
                self.dispatch._event('OBSERVED_TRAVEL_TIME',**self.travel_time.snapshot())
                self.eta_log_at=now
        boxes=body_boxes or {n:[envelope(s)] for n,s in states.items()}
        self.latest_body_boxes=boxes
        for vehicle,move in list(self.moves.items()):
            state=states[vehicle]
            measure=self.move_measurements.get(move.command_id)
            if measure:
                if abs(state.speed)<.08:measure['stopped_s']+=max(0.,now-measure['last_at'])
                measure['last_at']=now
            if move.command_id not in self.dispatch.departures and not any(overlaps(b,self.sources[vehicle],.1) for b in boxes[vehicle]):
                self.dispatch.departed(move.command_id,vehicle,move.visit_id,now,tail_clear=True)
            ex=self.controller.executions[vehicle]
            if ex.finished:
                if math.dist((state.x,state.y),ex.plan.point(ex.plan.length))>ARRIVAL_POSITION_TOLERANCE_M:
                    raise ValueError('Arrival requires actual stopped destination observation')
                # A suspension/ground settling impulse can follow the first
                # stopped sample. Keep braking and both leases until a fresh
                # stationary observation confirms arrival.
                if abs(state.speed)>.1:continue
                if measure:
                    elapsed=now-measure['start']
                    unchanged=(move.origin==measure['origin'] and move.destination==measure['target']
                               and tuple(move.route['edge_ids'])==measure['edges'])
                    self.travel_time.record_leg(move.destination,measure['nominal_s'],elapsed,measure['stopped_s'],unchanged_route=unchanged)
                    self.dispatch._event('LEG_ETA_MEASURED',vehicle_id=vehicle,origin=move.origin,target=move.destination,
                        elapsed_s=elapsed,predicted_s=measure['predicted_s'],stopped_s=measure['stopped_s'],unchanged_route=unchanged)
                    del self.move_measurements[move.command_id]
                self.dispatch.arrived(move.command_id,vehicle,move.visit_id,now,stopped=True)
                self.completed_moves.append(move);del self.moves[vehicle]
        if self.dispatch.traffic_routing is not None:
            self.dispatch.traffic_routing.observe(states,self.controller,self.dispatch.positions,now,moves=self.dispatch.moves)
        if self.controller.local_traffic:
            from .queue_clearance import refresh_requirements,clear_stopped_queues
            from .conflict_detour import try_conflict_detours
            try_conflict_detours(self,states,now)
            refresh_requirements(self)
            clear_stopped_queues(self,states,now)
            self.dispatch.promote_approaches(self._berth_priorities(states))
        if self.dispatch.offroad_busy_wait:
            self.dispatch.clearance_estimates={}
            from logistics_park.scheduling.conflict_timing import arrival_profile
            for owner,move in self.moves.items():
                if self.dispatch.positions.get(owner)!=move.origin or abs(states[owner].speed)<.2:continue
                local=self.controller.local_traffic
                if local and (local.last_denials.get(owner) or local.paused_by.get(owner)):continue
                ex=self.controller.executions[owner];source=self.sources[owner]
                # Estimate only an observed moving departure. Stationary service
                # deadlines are insufficient proof that the owner can leave.
                target=local.last_targets.get(owner,0.) if local else abs(states[owner].speed)
                if target<.2:continue
                profile=arrival_profile(ex,states[owner],now,hold=self.controller.destination_holds.get(owner),target_speed=target)
                for station in profile.stations:
                    future=envelope(type(states[owner])(owner,*ex.plan.point(station),ex.plan.heading(station)))
                    if not overlaps(future,source,.35):
                        eta=profile.eta(station)
                        if math.isfinite(eta):self.dispatch.clearance_estimates[owner]=now+eta
                        break
        if getattr(self.dispatch,'traffic_forecast',None) is not None:
            local=self.controller.local_traffic
            blocked=frozenset(name for name in states if local and (local.last_denials.get(name)
                              or local.passage_denials.get(name) or local.paused_by.get(name)))
            self.dispatch.traffic_forecast.update(states,self.controller.executions,now,
                holds=self.controller.destination_holds,speed_scales=self.travel_time.scale,blocked_vehicles=blocked,
                eta_service=self.eta_service)
        for move in self.dispatch.tick(now):
            vehicle=move.vehicle_id;plan=self.geometry.plan(move)
            self.controller.continue_route(vehicle,plan,now,state=states[vehicle],observations=states,body_boxes=boxes)
            self.moves[vehicle]=move
            self.sources[vehicle]=envelope(states[vehicle])
            if self.dispatch.scheduling.calibrated_eta:
                estimate=self.travel_time.estimate_route(self.dispatch.network,move.route,move.origin,move.destination)
                self.move_measurements[move.command_id]=dict(start=now,last_at=now,stopped_s=0.,
                    origin=move.origin,target=move.destination,edges=tuple(move.route['edge_ids']),
                    nominal_s=estimate['nominal_s'],predicted_s=estimate['total_s'])
        self.dispatch.promote_approaches(self._berth_priorities(states))
        self.controller.destination_holds={}
        for vehicle,move in self.moves.items():
            if self.dispatch.holders.get(move.destination)==vehicle:
                if self.dispatch.scheduling.protect_granted_queues:
                    hold=self._granted_queue_hold(vehicle,states)
                    if hold is not None:self.controller.destination_holds[vehicle]=hold
                continue
            plan=self.controller.executions[vehicle].plan
            index=len(plan.edges)-1
            while index>0 and self.geometry.model.edges[plan.edges[index-1]]['kind']=='service_access':index-=1
            self.controller.destination_holds[vehicle]=(self._queue_gate(vehicle,plan,states[vehicle]) if self.dispatch.efficiency_policy
                else max(0.,plan.edge_starts[index]-8.344-3.))
            if self.dispatch.efficiency_policy and self.dispatch.efficiency_policy.proactive_admission:
                from .safe_landing import safe_landing
                desired=self.controller.destination_holds[vehicle]
                ex=self.controller.executions[vehicle]
                landing=safe_landing(self,vehicle,plan,desired,states,progress=ex.progress,speed=states[vehicle].speed,target=move.destination,protect_following=True)
                saved=getattr(self,'safe_landing_states',{}).get((vehicle,plan.edges))
                if landing is not None:
                    self.controller.destination_holds[vehicle]=landing
                    self.safe_landing_states[vehicle,plan.edges]=landing
                elif saved is not None and saved<=desired+1e-6:
                    # A prior berth grant can cache the route endpoint. Once
                    # that unentered promise is released, it cannot override
                    # the current upstream admission line and send a queue
                    # into another dock's departure sweep.
                    self.controller.destination_holds[vehicle]=saved
        if self.dispatch.efficiency_policy and self.dispatch.efficiency_policy.proactive_admission:
            from logistics_park.scheduling.wait_dependencies import wait_snapshot
            self.controller.local_traffic.admission_dependencies=wait_snapshot(self,states)['dependencies']
        self.controller.parking_entry_priority=getattr(getattr(self.dispatch,'scheduling',None),'parking_entry_priority',False)
        self.controller.transport_entry_priority=getattr(getattr(self.dispatch,'scheduling',None),'transport_entry_priority',False)
        self.controller.parking_moves={v for v,m in self.moves.items() if getattr(m,'purpose',None)=='REPOSITION'}
        commands=self.controller.tick(states,now,body_boxes=boxes)
        if self.dispatch.efficiency_policy:
            from .wait_cycle_recovery import recover_wait_cycles
            recover_wait_cycles(self,states,commands,now)
        return commands

    def _granted_queue_hold(self,vehicle,states):
        """A destination lease cannot authorize a queue across a dock exit."""
        from .safe_landing import safe_landing
        ex=self.controller.executions[vehicle];plan=ex.plan
        if ex.finished:return None
        # Avoid geometry work for free-flowing traffic. Only an actual stopped
        # same-lane leader can force an admitted truck into a queue here.
        for other,state in states.items():
            if other==vehicle or abs(state.speed)>=.15:continue
            error,station=plan.project((state.x,state.y),ex.progress,behind=0.,ahead=80.)
            angle=abs((state.heading-plan.heading(station)+math.pi)%(2*math.pi)-math.pi)
            if error>1.4 or angle>math.pi/3 or station<=ex.progress+.1:continue
            landing=safe_landing(self,vehicle,plan,plan.length,states,
                progress=ex.progress,speed=states[vehicle].speed,
                target=self.moves[vehicle].destination,protect_following=True)
            if landing is not None and landing<plan.length-.5:
                return landing
            break
        return None

    def _berth_exit_blockers(self,target,vehicle):
        from .safe_landing import berth_exit_blockers
        return berth_exit_blockers(self,target,vehicle)

    def _parking_departure_guard(self,vehicle,origin,target,route,waiting):
        from types import SimpleNamespace
        from .safe_landing import parking_departure_clear
        plan=self.geometry.plan(SimpleNamespace(origin=origin,destination=target,route=route))
        if not parking_departure_clear(self,vehicle,plan,self.latest_states):
            key=(vehicle,target)
            if self.dispatch.now-self.landing_rejections.get(key,-math.inf)>=5.:
                self.dispatch._event('PARKING_EXIT_OCCUPIED',vehicle_id=vehicle,origin=origin,target=target)
                self.landing_rejections[key]=self.dispatch.now
            return False
        return self._departure_guard(vehicle,origin,target,route,waiting)

    def _parking_wait_guard(self,vehicle,origin,target,route):
        from types import SimpleNamespace
        from .safe_landing import parking_departure_clear,terminal_parking_blockers_clear_soon
        if not terminal_parking_blockers_clear_soon(self,vehicle,target):return False
        plan=self.geometry.plan(SimpleNamespace(origin=origin,destination=target,route=route))
        return parking_departure_clear(self,vehicle,plan,self.latest_states)

    def _departure_guard(self,vehicle,origin,target,route,waiting):
        from types import SimpleNamespace
        from .safe_landing import safe_landing,source_exit_blockers
        plan=self.geometry.plan(SimpleNamespace(origin=origin,destination=target,route=route))
        desired=plan.length if waiting or self.dispatch.holders.get(target)==vehicle else self._berth_gate(plan)
        blockers=[]
        if getattr(getattr(self.dispatch,'scheduling',None),'rolling_scheduler',False):
            blocked=source_exit_blockers(self,vehicle,plan,self.latest_states)
            if blocked:
                key=(vehicle,target)
                self.departure_waits=getattr(self,'departure_waits',{})
                self.departure_waits[key]=dict(time=self.dispatch.now,blockers=[
                    dict(blocked_by=name,reason='occupied_source_exit',resource=origin) for name in blocked])
                self.dispatch.actual_departure_blockers[key]=dict(time=self.dispatch.now,source=origin,blocked_by=blocked)
                if self.dispatch.now-self.landing_rejections.get(key,-math.inf)>=5.:
                    self.dispatch._event('DEPARTURE_ADMISSION_REJECTED',vehicle_id=vehicle,origin=origin,target=target,
                        reason='occupied_source_exit',desired_stop_s=desired,blocked_by=blocked)
                    self.landing_rejections[key]=self.dispatch.now
                return False
            self.dispatch.actual_departure_blockers.pop((vehicle,target),None)
        landing=safe_landing(self,vehicle,plan,desired,self.latest_states,target=target,blocker_evidence=blockers)
        if not hasattr(self,'departure_waits'):self.departure_waits={}
        key=(vehicle,target)
        goal=waiting or self.dispatch.holders.get(target)==vehicle
        if landing is None or goal and landing<plan.length-.35:
            self.departure_waits[key]=dict(time=self.dispatch.now,blockers=blockers)
            if self.dispatch.now-self.landing_rejections.get(key,-math.inf)>=5.:
                self.dispatch._event('DEPARTURE_ADMISSION_REJECTED',vehicle_id=vehicle,
                    origin=origin,target=target,reason='unsafe_goal_pose' if landing is not None else 'no_safe_hold',desired_stop_s=desired,
                    blocked_by=sorted({row['blocked_by'] for row in blockers}))
                self.landing_rejections[key]=self.dispatch.now
            return False
        self.departure_waits.pop(key,None)
        self.dispatch._event('SAFE_LANDING_SELECTED',vehicle_id=vehicle,origin=origin,
            target=target,stop_s=landing,desired_stop_s=desired)
        if not hasattr(self,'safe_landing_states'):self.safe_landing_states={}
        self.safe_landing_states[vehicle,plan.edges]=landing
        return True

    def _berth_gate(self,plan):
        """The first holding line whose permission needs the final dock.

        Adjacent intersection/access actions can be an indivisible bundle.
        Admission after that bundle's stop line creates a self-blocking loop:
        the truck cannot reach the admission window without the very permit
        that the window is supposed to grant.
        """
        from logistics_park.scheduling.conflict_timing import maneuver_groups
        index=len(plan.edges)-1
        while index>0 and plan.model.edges[plan.edges[index-1]]['kind']=='service_access':index-=1
        entry=plan.edge_starts[index]
        group=next((g for g in maneuver_groups(plan) if g[2]<=entry+1e-6 and g[3]>=entry-1e-6),None)
        gate=holding_line(group[2] if group else entry)
        if not getattr(getattr(self,'dispatch',None),'efficiency_policy',None):return gate
        # A preceding maneuver can require clearance beyond this holding line
        # even when its gap is too long for maneuver_groups to merge it.
        # Request the dock at the earliest indivisible permission boundary.
        for previous in reversed(maneuver_groups(plan)):
            if previous[2]>=gate+HOLD_BEFORE_ENTRY_M-1e-6:continue
            ordinary=not any(
                plan.model.edges[plan.edges[i]].get('reverse') or
                plan.model.edges[plan.edges[i]]['kind']=='uturn'
                for i in range(previous[0],previous[1]+1))
            outlet=ActionContract.for_group(plan,previous,ordinary).outlet
            if outlet>gate+1e-6:
                gate=holding_line(previous[2])
        if self.dispatch.efficiency_policy.proactive_admission and plan.stops:
            # Admission ETA and the static exit-safe stop line must agree.
            # Otherwise a queue head stops upstream of the old near-ETA window
            # and can never obtain the very berth needed to continue.
            from .safe_landing import safe_landing
            if not hasattr(self,'exit_admission_gates'):self.exit_admission_gates={}
            key=plan.edges
            if key not in self.exit_admission_gates:
                chosen=safe_landing(self,'admission-gate',plan,gate,{},target=plan.stops[-1],traffic=False)
                # No reachable stop exists before this dock: acquire its
                # admission while still at the owned source. Keeping the old
                # unreachable line would reject every departure yet postpone
                # the dock grant until the truck somehow reached that line.
                self.exit_admission_gates[key]=0. if chosen is None else chosen
            gate=self.exit_admission_gates[key]
        return gate

    def _queue_gate(self,vehicle,plan,state):
        """Keep a non-head dock queue out of the last intersection's outlet.

        The selected queue head still approaches the dock admission line; its
        ETA uses that original line, so holding an earlier queue cannot create
        another admission/permission self-lock. Never impose a new line behind
        an entered truck or inside its braking distance.
        """
        from logistics_park.scheduling.conflict_timing import maneuver_groups
        gate=self._berth_gate(plan)
        target=self.moves[vehicle].destination
        if not hasattr(self,'queue_gate_states'):self.queue_gate_states={}
        selected=self.dispatch.approach_intents.get(target)
        queued=selected not in (None,vehicle)
        if not queued:
            blockers=self.dispatch._blockers(target,vehicle)
            occupied={owner for node,owner in self.dispatch.holders.items()
                if owner in blockers and self.dispatch.positions.get(owner)==node}
            if occupied:
                release=max(self.dispatch.clearance_estimates.get(n,math.inf) for n in occupied)
                eta=(self._admission_eta(vehicle,plan,state,self.controller.executions[vehicle],station=gate)
                     if math.isfinite(release) else math.inf)
                queued=not math.isfinite(release) or self.dispatch.now+eta<release+3.
        if not queued:
            self.queue_gate_states.pop(vehicle,None)
            return gate
        groups=maneuver_groups(plan)
        final=next((g for g in groups if abs(holding_line(g[2])-gate)<1e-6 or gate==0 and g[2]<HOLD_BEFORE_ENTRY_M),None)
        if final and any(plan.model.edges[plan.edges[i]]['kind']=='junction' for i in range(final[0],final[1]+1)):
            return gate
        candidates=[g for g in groups if g[3]<gate+HOLD_BEFORE_ENTRY_M-1e-6 and
            any(plan.model.edges[plan.edges[i]]['kind']=='junction' for i in range(g[0],g[1]+1))]
        if not candidates:return gate
        hold=holding_line(candidates[-1][2])
        previous=self.queue_gate_states.get(vehicle)
        ex=self.controller.executions[vehicle]
        if previous and previous[0] is plan and previous[1]==hold:return hold
        available=hold-ex.progress
        from .safety_timing import braking_distance
        stopping=braking_distance(state.speed)
        if available<stopping and not (available>=-.35 and abs(state.speed)<.15):return gate
        self.queue_gate_states[vehicle]=(plan,hold)
        self.dispatch._event('DOCK_QUEUE_BEFORE_JUNCTION',vehicle_id=vehicle,node=target,
            queue_head=self.dispatch.approach_intents.get(target),stop_s=hold,berth_gate_s=gate)
        return hold

    def _admission_eta(self,vehicle,plan,state,execution=None,station=None):
        from types import SimpleNamespace
        from logistics_park.scheduling.conflict_timing import arrival_profile
        if execution is None:execution=SimpleNamespace(plan=plan,progress=0.,checkpoint=0,finished=False)
        if getattr(getattr(self.dispatch,'scheduling',None),'rolling_scheduler',False):
            if not hasattr(self,'eta_service'):
                from logistics_park.scheduling.eta_service import ETAService
                self.eta_service=ETAService();self.eta_query_logged_at={}
            now=self.dispatch.now;estimator=getattr(self,'travel_time',None)
            result=self.eta_service.query(vehicle,execution,state,now,plan.length if station is None else station,
                hold=self.controller.destination_holds.get(vehicle),speed_scales=estimator.scale if estimator else None)
            key=(vehicle,station)
            if now-self.eta_query_logged_at.get(key,-math.inf)>=1.:
                self.eta_query_logged_at[key]=now
                self.dispatch._event('ETA_PROFILE_QUERY',vehicle_id=vehicle,station_m=plan.length if station is None else station,
                    eta_s=result.eta_s if math.isfinite(result.eta_s) else None,freeflow_s=result.freeflow_s if math.isfinite(result.freeflow_s) else None,
                    reachable=result.reachable,reason=result.reason,version=result.version)
            # Ordering uses a free-flow lower bound; blocked reachability is
            # explicit in the trace and never grants actual movement authority.
            delay=estimator.blockage_delay(vehicle,self.controller.local_traffic,now,self.dispatch.efficiency_policy.blocked_delay_s) if estimator and self.dispatch.efficiency_policy else 0.
            return result.freeflow_s+delay
        if not hasattr(self,'admission_eta_cache'):self.admission_eta_cache={}
        now=self.dispatch.now;cache_key=(vehicle,station);cached=self.admission_eta_cache.get(cache_key)
        if (cached is None or cached[0] is not plan or now-cached[1]>=.5
                or abs(state.speed-cached[3])>.3 or abs(execution.progress-cached[4])>2.):
            policy=self.dispatch.efficiency_policy;estimator=getattr(self,'travel_time',None)
            scales=estimator.scale if policy and estimator else None
            eta=arrival_profile(execution,state,now,limit_s=600.,dt=.5,speed_scales=scales).eta(plan.length if station is None else station)
            if policy and estimator:
                eta+=estimator.blockage_delay(vehicle,self.controller.local_traffic,now,policy.blocked_delay_s)
            cached=(plan,now,eta,state.speed,execution.progress);self.admission_eta_cache[cache_key]=cached
        elapsed=now-cached[1] if abs(state.speed)>.1 else 0.
        return max(0.,cached[2]-elapsed)

    def _berth_priorities(self,states):
        from types import SimpleNamespace
        priorities=self._approach_priorities(states)
        for vehicle,a in self.dispatch.assignments.items():
            origin=self.dispatch.positions.get(vehicle);target=self.dispatch._target(a)
            if a.state not in ('READY','WAITING') or origin is None or origin==target:continue
            route=self.dispatch._route(origin,target,vehicle,future=True)
            if not route:continue
            estimate=route['distance_m']
            if self.dispatch.offroad_busy_wait:
                plan=self.geometry.plan(SimpleNamespace(origin=origin,destination=target,route=route))
                estimate=self._admission_eta(vehicle,plan,states[vehicle])+route.get('congestion_delay_s',0.)
            if self.dispatch.efficiency_policy:
                index=len(plan.edges)-1
                while index>0 and self.geometry.model.edges[plan.edges[index-1]]['kind']=='service_access':index-=1
                entry_eta=self._admission_eta(vehicle,plan,states[vehicle],station=self._berth_gate(plan))
                leaders=(self._admission_leaders(vehicle,target,plan,0.,states)
                         if self.dispatch.scheduling.connected_clearance else frozenset())
                priorities[vehicle]=(estimate,False,leaders,entry_eta)
            else:priorities[vehicle]=(estimate,False)
        # A parked same-destination candidate can occupy an unstarted
        # applicant's exit too. It has no live move, so lane-order detection
        # alone cannot see it. Let that actual blocker leave first rather
        # than promising the berth to the truck it physically prevents.
        from .safe_landing import source_admission_leaders
        for vehicle,priority in list(priorities.items()):
            origin=self.dispatch.positions.get(vehicle)
            if origin is None:continue
            move=self.moves.get(vehicle)
            if move:
                execution=self.controller.executions[vehicle]
                plan=execution.plan;progress=execution.progress;target=move.destination
            else:
                assignment=self.dispatch.assignments[vehicle];target=self.dispatch._target(assignment)
                if origin==target:continue
                route=self.dispatch._route(origin,target,vehicle,future=True)
                if not route:continue
                plan=self.geometry.plan(SimpleNamespace(origin=origin,destination=target,route=route));progress=0.
            leaders=source_admission_leaders(self,vehicle,target,plan,progress,states)
            if leaders:
                existing=priority[2] if len(priority)>2 else frozenset()
                priorities[vehicle]=(*priority[:2],existing|leaders,*priority[3:])
        return priorities

    def _admission_leaders(self,vehicle,target,plan,progress,states):
        """Observe shared-bay lane order before a parked truck gets a permit."""
        site=self.dispatch.network.services[target]
        resources=set(site.get('occupation_resources',[]))
        leaders=set()
        for other,move in self.moves.items():
            if other==vehicle or other not in states:continue
            other_site=self.dispatch.network.services[move.destination]
            if not (resources.intersection(other_site.get('occupation_resources',[]))
                    or site.get('warehouse') and site.get('warehouse')==other_site.get('warehouse')):continue
            state=states[other]
            other_ex=getattr(getattr(self,'controller',None),'executions',{}).get(other)
            if other_ex is not None:
                # A shared dock can require the same unique U-turn/connector
                # from different approach lanes. Select its nearer approach
                # before granting the rear truck the adjacent dock's promise.
                # Projection alone misses the leader just before a lane merge.
                def approaches(route,current):
                    spans=getattr(route,'dock_exclusive_spans',None)
                    if spans is None:
                        spans={};capacities=route.model.raw.get('physical_capacities',{})
                        for i,key in enumerate(route.edges):
                            edge=route.model.edges[key];alias=route.model.aliases[key]
                            if edge['kind'] not in ('uturn','junction') or capacities.get(alias)!=1:continue
                            sections=spans.setdefault(alias,[])
                            if sections and abs(sections[-1][1]-route.edge_starts[i])<1e-6:
                                sections[-1]=(sections[-1][0],route.edge_ends[i])
                            else:sections.append((route.edge_starts[i],route.edge_ends[i]))
                        route.dock_exclusive_spans=spans
                    return {alias:min(max(0.,a-current) for a,b in sections if b>current+.1)
                            for alias,sections in spans.items() if any(b>current+.1 for a,b in sections)}
                own=approaches(plan,progress);front=approaches(other_ex.plan,other_ex.progress)
                if any(front[alias]<=20. and own[alias]>front[alias]+11.344 for alias in own.keys()&front.keys()):
                    leaders.add(other);continue
            error,station=plan.project((state.x,state.y),progress,behind=0.,ahead=80.)
            angle=abs((state.heading-plan.heading(station)+math.pi)%(2*math.pi)-math.pi)
            if error<1.4 and angle<math.pi/3 and station>progress+.1:leaders.add(other)
        return frozenset(leaders)

    def _approach_priorities(self,states=None):
        priorities={}
        for vehicle,move in self.moves.items():
            ex=self.controller.executions[vehicle];plan=ex.plan;index=len(plan.edges)-1
            while index>0 and self.geometry.model.edges[plan.edges[index-1]]['kind']=='service_access':index-=1
            started=ex.progress>.05 or bool(states and abs(states[vehicle].speed)>.1)
            committed=started and ex.progress>=plan.edge_starts[index]
            local=self.controller.local_traffic
            if local:
                committed=committed or any(key[0]==vehicle and lease['plan'] is plan
                    and lease['end']>=plan.length-1e-6 and started and ex.progress>=plan.edge_starts[key[1]]
                    for key,lease in local.passages.items())
                if (self.dispatch.efficiency_policy and self.dispatch.efficiency_policy.proactive_admission
                        and started and ex.progress>=self._berth_gate(plan)):
                    # The berth gate includes earlier indivisible actions.
                    # Once one of those actions has actually started under a
                    # permit, withdrawing its destination promise can leave
                    # no safe place to stop before the final dock access.
                    committed=committed or any(key[0]==vehicle and lease['plan'] is plan
                        and ex.progress>=plan.edge_starts[key[1]]
                        for key,lease in local.passages.items())
            remaining=plan.length-ex.progress
            if self.dispatch.offroad_busy_wait and states:
                remaining=self._admission_eta(vehicle,plan,states[vehicle],ex)
                traffic=self.dispatch.traffic_routing
                if traffic:
                    costs=traffic.costs_for(vehicle,self.dispatch.moves);station=0.
                    for key in move.route['edge_ids']:
                        length=self.dispatch.network.edges[key]['length_m']
                        fraction=min(1.,max(0.,(station+length-ex.progress)/max(.001,length)))
                        remaining+=costs.get(key,0.)*fraction;station+=length
                else:remaining+=move.route.get('congestion_delay_s',0.)
            if self.dispatch.efficiency_policy and states:
                boundary=self._berth_gate(plan)
                hold=self.controller.destination_holds.get(vehicle)
                if hold is None:
                    # A grant removes the active hold, not the exit-safe line
                    # already reached by this route's queue head. Retain that
                    # observed admission boundary across subsequent grant checks.
                    hold=getattr(self,'safe_landing_states',{}).get((vehicle,plan.edges))
                if (self.dispatch.efficiency_policy.proactive_admission
                        and self.dispatch.approach_intents.get(move.destination)==vehicle
                        and hold is not None and hold<boundary-.5 and ex.progress>=hold-.5):
                    # Exit protection can move a queue head's safe line
                    # upstream. It cannot reach the old near-ETA window while
                    # held there. Admission and stopping must use one boundary;
                    # only the actual head already at that line gets this.
                    boundary=hold
                    if not hasattr(self,'dynamic_admission_logs'):self.dynamic_admission_logs={}
                    key=(vehicle,plan.edges,round(hold,3))
                    if self.dispatch.now-self.dynamic_admission_logs.get(key,-math.inf)>=5.:
                        self.dispatch._event('DYNAMIC_BERTH_ADMISSION_LINE',vehicle_id=vehicle,
                            target=move.destination,stop_s=hold,reason='queue_head_at_exit_safe_line')
                        self.dynamic_admission_logs[key]=self.dispatch.now
                entry_eta=self._admission_eta(vehicle,plan,states[vehicle],ex,station=boundary)
                priorities[vehicle]=(remaining,committed,frozenset(),entry_eta)
            else:priorities[vehicle]=(remaining,committed)
        if states:
            for vehicle,move in self.moves.items():
                ex=self.controller.executions[vehicle]
                if self.dispatch.scheduling.connected_clearance:
                    leaders=self._admission_leaders(vehicle,move.destination,ex.plan,ex.progress,states)
                    priority=priorities[vehicle]
                    priorities[vehicle]=(priority[0],priority[1],leaders,*priority[3:])
                    continue
                resources=set(self.dispatch.network.services[move.destination].get('occupation_resources',[]))
                leaders=set()
                for other,other_move in self.moves.items():
                    if other==vehicle or not resources.intersection(self.dispatch.network.services[other_move.destination].get('occupation_resources',[])):continue
                    state=states[other]
                    error,station=ex.plan.project((state.x,state.y),ex.progress,behind=0,ahead=40.)
                    heading_error=abs((state.heading-ex.plan.heading(station)+math.pi)%(2*math.pi)-math.pi)
                    if error<1.4 and heading_error<math.pi/3 and station>ex.progress+.1:
                        leaders.add(other)
                priority=priorities[vehicle]
                priorities[vehicle]=(priority[0],priority[1],frozenset(leaders),*priority[3:])
        return priorities

    def resource_snapshot(self,vehicle):
        """Aggregate task, physical road and shared leases for the gate backend.

        The gate must take this snapshot from a fresh execution tick and still
        confirm its physical outside boundary; empty task leases alone do not
        authorize departure.
        """
        c=self.controller
        resources=set(self.dispatch.resource_snapshot(vehicle))
        for alias,occupants in c.capacity.occupants.items():
            if vehicle in occupants:resources.add('road:'+alias)
        for alias,committed in c.capacity.committed.items():
            if vehicle in committed:resources.add('road:'+alias)
        if c.shared_scheduler:
            resources.update('shared:'+key for key,owner in c.shared_scheduler.owners.items() if owner==vehicle)
        if c.local_traffic:
            resources.update('local-passage:'+str(key[1]) for key in c.local_traffic.passages if key[0]==vehicle)
        resources.update('junction:'+key for key,grant in c.scheduler.owners.items() if grant.request.vehicle_id==vehicle)
        return frozenset(resources)
