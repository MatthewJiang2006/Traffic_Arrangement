"""Greedy ETA task dispatch above lane/road reservation and vehicle control."""
from dataclasses import dataclass,replace
from enum import Enum
import math
from functools import lru_cache

from logistics_park.common.messages import finite
from .fleet_lifecycle import FleetPhase, FleetRegistry


@lru_cache(maxsize=512)
def _access_body_samples(points,reverse):
    from logistics_park.map.geometry import Polyline
    from logistics_park.safety.collision import OBB
    line=Polyline(points);result=[]
    for index in range(math.ceil(line.length/.5)+1):
        (x,y),h=line.at(min(index*.5,line.length));h+=math.pi if reverse else 0
        result.append(OBB(x+.75*math.cos(h),y+.75*math.sin(h),h,8.344,2.6))
    return tuple(result)


class WorkStatus(str, Enum):
    PENDING = 'PENDING'
    ACTIVE = 'ACTIVE'
    COMPLETE = 'COMPLETE'


@dataclass(frozen=True)
class TransportWork:
    task_id: str
    loading_node: str
    unloading_node: str
    release_time: float = 0.
    priority: int = 0

    def __post_init__(self):
        finite(self.release_time)
        if not all((self.task_id,self.loading_node,self.unloading_node)) or self.release_time<0:
            raise ValueError('Invalid transport work')


@dataclass(frozen=True)
class DispatchMove:
    command_id: int
    task_id: str
    vehicle_id: str
    visit_id: str
    origin: str
    destination: str
    route: dict
    waiting: bool
    purpose: str = 'TRANSPORT'


@dataclass
class Assignment:
    work: TransportWork
    vehicle_id: str
    visit_id: str
    operation: str = 'LOADING'
    state: str = 'READY'
    queued_at: float | None = None
    service_until: float | None = None
    cargo_units: int = 0
    traffic_wait_since: float | None = None


class TaskDispatcher:
    """Node leases include approaching vehicles and parked physical occupants.

    The backend must acknowledge actual stopped arrivals and tail-clear
    departures. Movement commands are plans, never teleportation instructions.
    """
    def __init__(self, network, fleet: FleetRegistry, service_s=5.,protect_committed_routes=False,traffic_routing=None,follow_to_busy_destination=False,park_idle_vehicles=False,offroad_busy_wait=False,efficiency_policy=None,scheduling_options=None):
        finite(service_s)
        if service_s<0:raise ValueError('Negative service duration')
        self.network=network;self.fleet=fleet;self.service_s=service_s
        self.tasks={};self.status={};self.assignments={};self.positions={}
        self.holders={};self.moves={};self.departures=set();self.events=[]
        self.follow_to_busy_destination=follow_to_busy_destination
        self.park_idle_vehicles=park_idle_vehicles
        self.offroad_busy_wait=offroad_busy_wait;self.clearance_estimates={};self.recovery_estimates={}
        self.departure_rejected_at={}
        self.actual_departure_blockers={}
        self.admission_priorities={}
        from .efficiency_policy import EfficiencyPolicy
        self.efficiency_policy=EfficiencyPolicy(**efficiency_policy) if efficiency_policy is not None else None
        self.approach_intents={};self.waiting_history={}
        self.hold_logged_at={}
        self.live_requirements={}
        self.parking_deferred_candidates={}
        self.parking_terminal_wait_since={}
        self.parking_terminal_wait_logged_at={}
        self.waiting_lenders={}
        self.now=0.;self.sequence=0;self.queue_cache={};self.protect_committed_routes=protect_committed_routes;self.traffic_routing=traffic_routing
        from .optimization import SchedulingOptions
        self.scheduling=SchedulingOptions(**(scheduling_options or {}))
        self.last_decision=-math.inf;self.last_decision_signature=None
        self.rolling_plan_at=-math.inf
        self.departure_plan_at=-math.inf;self.departure_not_before={};self.departure_plan_wait_since={}
        if self.scheduling.rolling_scheduler:
            from .global_scheduler import RollingScheduler
            self.global_scheduler=RollingScheduler(self.scheduling.planning_budget_s)
        else:self.global_scheduler=None
        if self.scheduling.traffic_forecast:
            from .traffic_forecast import TrafficForecast
            self.traffic_forecast=TrafficForecast()
        else:self.traffic_forecast=None

    def _score_route(self,origin,target,route):
        predictor=getattr(self,'route_eta_predictor',None)
        if self.scheduling.calibrated_eta and predictor:
            return predictor(self.network,route,origin,target)['total_s']
        return route['estimated_travel_time_s']

    def _pickup_eta(self,work,vehicle,route):
        origin=self.positions[vehicle];travel=self._score_route(origin,work.loading_node,route)
        if self.scheduling.calibrated_eta and origin==work.loading_node:return 0.
        blockers=self._blockers(work.loading_node,vehicle)
        if not self.scheduling.calibrated_eta:
            return travel+self.service_s*(int(bool(blockers))+len(self._destination_queue(work.loading_node)))
        # Clearance forecasts affect ranking only. A finite prediction never
        # releases a berth, grants a passage, or overrides the departure guard.
        release=0.
        for owner in blockers:
            forecast=self.clearance_estimates.get(owner)
            a=self.assignments.get(owner)
            if forecast is not None and math.isfinite(forecast):delay=max(0.,forecast-self.now)
            elif a and a.state=='SERVICING':delay=max(0.,a.service_until-self.now)+20.
            else:delay=30.
            release=max(release,delay)
        ahead=[a for a in self.assignments.values() if a.vehicle_id!=vehicle and a.vehicle_id not in blockers
               and a.operation=='LOADING' and a.work.loading_node==work.loading_node]
        arrivals=[]
        for a in ahead:
            eta=self._admission_priority(a,self.admission_priorities)[0]
            arrivals.append(eta if math.isfinite(eta) else 60.)
        for eta in sorted(arrivals):release=max(release,eta)+self.service_s+20.
        return max(travel,release)

    def _batch_pairs(self,pending):
        from .optimization import bottleneck_assignment
        vehicles=[v for v in sorted(self.fleet.available_vehicle_ids) if v in self.positions and v not in self.moves]
        signature=(tuple((v,self.positions[v]) for v in vehicles),tuple(w.task_id for w in pending))
        if not vehicles or not pending:return []
        if signature==self.last_decision_signature and self.now-self.last_decision<self.scheduling.decision_interval_s:return []
        self.last_decision=self.now;self.last_decision_signature=signature
        submitted={key:i for i,key in enumerate(self.tasks)}
        ordered=sorted(pending,key=lambda w:(-(w.priority+min(10.,(self.now-w.release_time)/120.)),w.release_time,submitted[w.task_id]))
        counts={}
        for a in self.assignments.values():
            if a.operation=='LOADING':counts[a.work.loading_node]=counts.get(a.work.loading_node,0)+1
        frontier=[];routes={}
        for w in ordered:
            count=counts.get(w.loading_node,0)
            if self.scheduling.max_loading_inflight and count>=self.scheduling.max_loading_inflight:continue
            for vehicle in vehicles:
                busy=bool(self._blockers(w.loading_node,vehicle))
                route=self._route(self.positions[vehicle],w.loading_node,vehicle,future=busy)
                if route and (not busy or self.follow_to_busy_destination or self._waiting_options(self.positions[vehicle],w.loading_node,vehicle)):
                    routes[vehicle,w.task_id]=route
            if not any((vehicle,w.task_id) in routes for vehicle in vehicles):continue
            frontier.append(w);counts[w.loading_node]=count+1
            if len(frontier)>=len(vehicles):break
        estimates={};costs=[]
        for vehicle in vehicles:
            row=[];origin=self.positions[vehicle]
            for j,work in enumerate(frontier):
                route=routes.get((vehicle,work.task_id))
                if not route:
                    row.append(math.inf);continue
                pickup=self._pickup_eta(work,vehicle,route)
                onward=self.network.route(work.loading_node,work.unloading_node,
                    occupied_nodes=frozenset(),**({'transit_warehouses':self._traffic_warehouses(work.loading_node,work.unloading_node)} if self.traffic_routing else {}))
                if not onward:row.append(math.inf);continue
                finish=pickup+2*self.service_s+self._score_route(work.loading_node,work.unloading_node,onward)
                estimates[vehicle,j]=(pickup,finish);row.append(finish)
            costs.append(row)
        pairs=[]
        for i,j in bottleneck_assignment(costs):
            vehicle=vehicles[i];pickup,finish=estimates[vehicle,j]
            pairs.append((frontier[j],vehicle,pickup))
            self._event('BATCH_MATCH_SELECTED',vehicle_id=vehicle,task_id=frontier[j].task_id,
                        estimated_pickup_delay_s=pickup,estimated_completion_delay_s=finish)
        return pairs

    def _continuation_pending(self,pending):
        # Startup keeps the previously accepted dispatch sequence. After a
        # completion, rank available work by its best feasible empty pickup;
        # optionally subtract a bounded loaded-leg weight to favor long tails.
        # Ranking never changes any physical berth or passage reservation.
        if not self.scheduling.continuation_dispatch or not any(
                status==WorkStatus.COMPLETE for status in self.status.values()):
            return pending
        vehicles=[v for v in sorted(self.fleet.available_vehicle_ids)
                  if v in self.positions and v not in self.moves]
        if not vehicles:return pending
        counts={}
        for a in self.assignments.values():
            if a.operation=='LOADING':counts[a.work.loading_node]=counts.get(a.work.loading_node,0)+1
        signature=(tuple((v,self.positions[v]) for v in vehicles),
                   tuple(w.task_id for w in pending),tuple(sorted(counts.items())))
        cache=getattr(self,'continuation_cache',None)
        if cache and cache[0]==signature and self.now-cache[1]<self.scheduling.decision_interval_s:
            scores=cache[2]
        else:
            scores={}
            for work in pending:
                scores[work.task_id]=math.inf
                if self.scheduling.max_loading_inflight and counts.get(work.loading_node,0)>=self.scheduling.max_loading_inflight:
                    continue
                best=math.inf
                for vehicle in vehicles:
                    origin=self.positions[vehicle];busy=bool(self._blockers(work.loading_node,vehicle))
                    route=self._route(origin,work.loading_node,vehicle,future=busy)
                    if not route:continue
                    if busy and not self.follow_to_busy_destination and not self._waiting_options(origin,work.loading_node,vehicle):continue
                    best=min(best,self._pickup_eta(work,vehicle,route))
                if not math.isfinite(best):continue
                onward=self.network.route(work.loading_node,work.unloading_node,occupied_nodes=frozenset(),
                    **({'transit_warehouses':self._traffic_warehouses(work.loading_node,work.unloading_node)} if self.traffic_routing else {}))
                if not onward:continue
                scores[work.task_id]=best-self.scheduling.pickup_tail_weight*self._score_route(work.loading_node,work.unloading_node,onward)
            self.continuation_cache=(signature,self.now,scores)
        return sorted(pending,key=lambda w:(-w.priority,scores[w.task_id],w.release_time,w.task_id))

    def _time(self, now):
        finite(now)
        if now<self.now:raise ValueError('Out-of-order dispatch time')
        self.now=now

    def _event(self,event,**data):
        self.events.append(dict(time=self.now,event=event,**data))

    def submit(self, work: TransportWork):
        if work.task_id in self.tasks:raise ValueError('Duplicate transport task')
        for node,kind in ((work.loading_node,'loading'),(work.unloading_node,'unloading')):
            if node not in self.network.services or self.network.services[node]['kind']!=kind:
                raise ValueError('Task requires a loading node and an unloading node')
        self.tasks[work.task_id]=work;self.status[work.task_id]=WorkStatus.PENDING

    def place_vehicle(self, vehicle_id, visit_id, node, now, *, stopped):
        self._time(now)
        v=self.fleet.vehicles[vehicle_id]
        if (v.visit_id!=visit_id or v.phase!=FleetPhase.IDLE or not stopped
                or now<v.updated_at or vehicle_id in self.assignments or vehicle_id in self.positions):
            raise ValueError('Only a confirmed stopped idle vehicle can be placed')
        if node not in self.network.services or self.network.services[node]['capacity']!=1:
            raise ValueError('Unknown or unsupported service capacity')
        if node in self.holders:raise ValueError('Initial bay already occupied')
        self.positions[vehicle_id]=node;self.holders[node]=vehicle_id

    def _occupied(self, vehicle):
        # Far-future berth bookings are admission promises, not parked bodies
        # that close a transit/exit route. Local trajectory admission controls
        # approaching vehicles; actual source occupancy stays until tail-clear.
        return frozenset(node for node,holder in self.holders.items() if holder!=vehicle
            and (not self.follow_to_busy_destination or self.positions.get(holder)==node))

    def _route(self, origin, target, vehicle, *, future=False):
        if origin==target:return dict(edge_ids=[],distance_m=0.,estimated_travel_time_s=0.)
        occupied=self._occupied(vehicle)
        if future:
            resources=set(self.network.services[target].get('occupation_resources',[]))
            occupied={n for n in occupied if n!=target and not resources.intersection(self.network.services[n].get('occupation_resources',[]))}
        kwargs={}
        if self.traffic_routing is not None:
            kwargs=dict(edge_costs=self.traffic_routing.costs_for(vehicle,self.moves),transit_warehouses=self._traffic_warehouses(origin,target))
        route=self.network.route(origin,target,occupied_nodes=occupied,**kwargs)
        if route and self.traffic_forecast is not None:
            predicted=self.traffic_forecast.route_delay(self.network,route,vehicle,self.now)
            previous=route.get('congestion_delay_s',0.)
            # Replace the advisory congestion component, never double-count it.
            if predicted>previous:
                route={**route,'congestion_delay_s':predicted,
                       'estimated_travel_time_s':route['estimated_travel_time_s']+predicted-previous}
        return route

    def _traffic_warehouses(self,origin,target):
        warehouses=set()
        for node in (origin,target):
            site=self.network.services[node]
            if site.get('warehouse'):warehouses.add(site['warehouse'])
            warehouses.update(site.get('assigned_warehouses',[]))
        return frozenset(warehouses)

    def _blockers(self,target,vehicle):
        resources=set(self.network.services[target].get('occupation_resources',[]))
        blocked={v for n,v in self.holders.items() if v!=vehicle and
                (n==target or resources.intersection(self.network.services[n].get('occupation_resources',[])))}
        guard=getattr(self,'berth_exit_blockers',None)
        if guard:blocked.update(guard(target,vehicle))
        return blocked

    def _source_waiters(self,origin,vehicle):
        """Demand for a dock includes its shared clearance and transit access.

        A follower may target the adjacent unloading bay or pass through the
        loading access without targeting this source node itself.
        """
        resources=set(self.network.services[origin].get('occupation_resources',[]))
        waiting=set()
        for other,move in self.moves.items():
            if other==vehicle:continue
            edges=getattr(self.network,'edges',{})
            traverses_source=any(edges.get(key,{}).get('to_node')==origin
                                 for key in move.route.get('edge_ids',[]))
            if (move.destination==origin or origin in move.route.get('must_be_empty_nodes',[])
                    or traverses_source
                    or resources.intersection(move.route.get('required_resources',[]))
                    or vehicle in self._blockers(move.destination,other)):
                waiting.add(other)
        waiting.update(other for other,pending in self.assignments.items()
            if other!=vehicle and pending.state in ('READY','WAITING')
            and (self._target(pending)==origin or vehicle in self._blockers(self._target(pending),other)))
        return waiting

    def _target(self, assignment):
        return assignment.work.loading_node if assignment.operation=='LOADING' else assignment.work.unloading_node

    def _waiting_options(self, origin, target, vehicle):
        key=(origin,target)
        if key not in self.queue_cache:
            self.queue_cache[key]=self.network.queue_options(origin,target)
        options=[]
        direct=self._route(origin,target,vehicle,future=True) if self.offroad_busy_wait else None
        for option in self.queue_cache[key]:
            q=option['waiting_node']
            if self.efficiency_policy and not self.efficiency_policy.parking_enabled(q):continue
            if self.protect_committed_routes and self._blocks_committed_route(q,vehicle):continue
            target_busy=bool(self._blockers(target,vehicle))
            if ((target_busy and self.traffic_routing is None and not option['usable_while_destination_occupied']) or q in self.holders or self._blockers(q,vehicle)
                    or option['attachment_validation_required'] or option['access_validation_required']):continue
            # Avoid parking where departure is already blocked by another
            # reserved/occupied waiting place (notably Q1 behind Q2).
            if any(n in self.holders and self.holders[n]!=vehicle for n in option['departure_requires_empty']):continue
            approach=self._route(origin,q,vehicle)
            onward=self._route(q,target,vehicle,future=True)
            if approach and not self._parking_approach_clear(approach,vehicle):continue
            if approach and onward:
                detour=max(0.,approach['distance_m']+onward['distance_m']-(direct['distance_m'] if direct else 0.))
                if self.efficiency_policy and not self.efficiency_policy.affordable(
                        approach['distance_m'],detour,approach['estimated_travel_time_s']):continue
                approach={**approach,'waiting_onward_freeflow_s':onward.get('freeflow_travel_time_s',onward['estimated_travel_time_s']), 'waiting_detour_m':detour}
                cost=approach['estimated_travel_time_s']+onward['estimated_travel_time_s']
                if self.scheduling.waiting_completion_cost:
                    travel=self._score_route(origin,q,approach)
                    release=max((self.clearance_estimates.get(b,self.now+30.) for b in self._blockers(target,vehicle)),default=self.now)
                    if not math.isfinite(release):release=self.now+30.
                    cost=max(travel,release-self.now)+self._score_route(q,target,onward)
                    approach={**approach,'waiting_completion_eta_s':cost}
                options.append((cost,q,approach))
        return sorted(options,key=lambda item:((item[2]['waiting_detour_m'],item[0],item[1]) if self.offroad_busy_wait and not self.scheduling.waiting_completion_cost else (item[0],item[1])))

    def _parking_plan(self):
        """Provisional future allocation; only observed idle trucks may move."""
        from .optimization import bottleneck_assignment
        idle=[v for v in sorted(self.fleet.available_vehicle_ids) if v in self.positions
              and v not in self.moves and self.network.services[self.positions[v]]['kind']!='waiting']
        if not idle:return {}
        signature=(tuple(idle),tuple(sorted((name,a.operation,a.work.unloading_node)
                                          for name,a in self.assignments.items())))
        if (self.now-getattr(self,'parking_plan_at',-math.inf)<5.
                and (not self.scheduling.parking_tail_reserve
                     or signature==getattr(self,'parking_plan_signature',None))):
            return getattr(self,'parking_plan_cache',{})
        sites=[q for q,site in sorted(self.network.services.items()) if site['kind']=='waiting'
               and q not in self.holders and (not self.efficiency_policy or self.efficiency_policy.parking_enabled(q))]
        futures=[]
        for name,a in self.assignments.items():
            eta=self._admission_priority(a,self.admission_priorities)[0]
            if not math.isfinite(eta):eta=600.
            if a.operation=='LOADING':
                onward=self.network.route(a.work.loading_node,a.work.unloading_node,occupied_nodes=frozenset(),
                    **({'transit_warehouses':self._traffic_warehouses(a.work.loading_node,a.work.unloading_node)} if self.traffic_routing else {}))
                eta+=self._score_route(a.work.loading_node,a.work.unloading_node,onward) if onward else 600.
            futures.append((min(1200.,eta+2*self.service_s),name,a.work.unloading_node))
        futures=sorted(futures,reverse=True)[:max(0,len(sites)-len(idle))]
        costs=[];potential_idle_costs=[]
        self.parking_deferred_candidates={}
        for name in idle:
            row=[];potential=[];origin=self.positions[name]
            terminal=self._last_terminal_parking_vehicle(name)
            for q in sites:
                site=self.network.services[q]
                if any(n in self.holders for n in site.get('departure_requires_empty',[])):
                    row.append(math.inf);potential.append(math.inf);continue
                route=self._route(origin,q,name)
                value=self._score_route(origin,q,route) if route else math.inf
                potential.append(value)
                blocked=self.protect_committed_routes and self._blocks_committed_route(q,name)
                clear=bool(route and (not blocked or terminal) and self._parking_approach_clear(route,name))
                row.append(value if not blocked and clear else math.inf)
                if blocked and clear and math.isfinite(value):
                    self.parking_deferred_candidates.setdefault(name,[]).append((value,q,route))
            costs.append(row);potential_idle_costs.append(potential)
        occupied=frozenset(q for q in self.holders if self.network.services[q]['kind']=='waiting')
        for eta,name,origin in futures:
            row=[]
            for q in sites:
                route=self.network.route(origin,q,occupied_nodes=occupied,
                    **({'transit_warehouses':self._traffic_warehouses(origin,q)} if self.traffic_routing else {}))
                row.append(eta+self._score_route(origin,q,route) if route else math.inf)
            costs.append(row)
        self.parking_future_targets={}
        if self.scheduling.parking_tail_reserve and futures:
            # Finish-time ETA can fluctuate or miss a long berth queue. Solve
            # the remaining trucks' terminal parking first, then leave those
            # short destinations for them. These are advisory choices only.
            # Rolling planning minimizes final completion, including remaining
            # transport. Legacy tail protection keeps its ETA-independent rule.
            future_costs=(costs[len(idle):] if self.scheduling.rolling_scheduler else
                          [[value-eta if math.isfinite(value) else math.inf for value in costs[len(idle)+index]]
                           for index,(eta,name,origin) in enumerate(futures)])
            future_pairs=bottleneck_assignment(future_costs)
            reserved={sites[j] for i,j in future_pairs}
            restricted=[[math.inf if (q in reserved or set(self.network.services[q].get('departure_requires_empty',[])) & reserved)
                         else value for q,value in zip(sites,row)] for row in costs[:len(idle)]]
            idle_pairs=bottleneck_assignment(restricted)
            potential_restricted=[[math.inf if (q in reserved or set(self.network.services[q].get('departure_requires_empty',[])) & reserved)
                                   else value for q,value in zip(sites,row)] for row in potential_idle_costs]
            potential_pairs=bottleneck_assignment(potential_restricted)
            # Never retain a real source dock merely to honor a speculative
            # parking preference when the available graph cannot fit everyone.
            if len(future_pairs)==len(futures) and len(potential_pairs)==len(idle):
                # Keep the late truck's preference while an alternative is
                # temporarily blocked by an existing movement. Actual issue
                # still requires all current route and body checks to pass.
                self.parking_future_targets={futures[i][1]:sites[j] for i,j in future_pairs}
                self._event('TAIL_PARKING_PREFERENCES',targets=dict(self.parking_future_targets),physical_leases_changed=False)
                result={idle[i]:sites[j] for i,j in idle_pairs}
            else:
                result={idle[i]:sites[j] for i,j in bottleneck_assignment(costs) if i<len(idle)}
        else:
            result={idle[i]:sites[j] for i,j in bottleneck_assignment(costs) if i<len(idle)}
        self.parking_plan_at=self.now;self.parking_plan_cache=result;self.parking_plan_signature=signature
        if result:self._event('PARKING_ALLOCATION_PLANNED',targets=dict(result),forecast_vehicles=len(futures),physical_leases_changed=False)
        return result

    def _last_terminal_parking_vehicle(self,vehicle):
        if (not self.scheduling.parking_tail_reserve or not self.status
                or any(s!=WorkStatus.COMPLETE for s in self.status.values())):return False
        idle=[v for v in self.fleet.available_vehicle_ids if v in self.positions
              and v not in self.moves and self.network.services[self.positions[v]]['kind']!='waiting']
        return idle==[vehicle]

    def _wait_for_terminal_parking(self,vehicle,origin,options):
        """Keep the real source briefly instead of taking a long terminal detour.

        A blocked candidate remains advisory. Its lease and movement are issued
        only after all normal route and departure checks succeed on a later tick.
        """
        if not options or self._source_waiters(origin,vehicle):return False
        available=min(self._score_route(origin,q,route) for _,q,route in options)
        candidates=[(cost,q,route) for cost,q,route in self.parking_deferred_candidates.get(vehicle,[])
                    if q not in self.holders and self._blocks_committed_route(q,vehicle)
                    and cost+30.+3.<available]
        if not candidates:return False
        cost,target,route=min(candidates,key=lambda item:(item[0],item[1]))
        guard=getattr(self,'parking_departure_guard',getattr(self,'departure_guard',None))
        if guard is not None and not guard(vehicle,origin,target,route,True):
            wait_guard=getattr(self,'parking_wait_guard',None)
            if wait_guard is None or not wait_guard(vehicle,origin,target,route):return False
        previous=self.parking_terminal_wait_since.get(vehicle)
        if previous is None or previous[0]!=origin:
            previous=(origin,self.now);self.parking_terminal_wait_since[vehicle]=previous
        waited=self.now-previous[1]
        if waited>=30.:
            if self.parking_terminal_wait_logged_at.get(vehicle,-math.inf)<previous[1]+30.:
                self._event('TERMINAL_PARKING_WAIT_EXPIRED',vehicle_id=vehicle,origin=origin,
                            target=target,waited_s=waited,physical_leases_changed=False)
                self.parking_terminal_wait_logged_at[vehicle]=self.now
            return False
        if self.now-self.parking_terminal_wait_logged_at.get(vehicle,-math.inf)>=5.:
            others=set(self.moves)
            blockers=[n for n in sorted(others) if self._blocks_committed_route(
                target,vehicle,ignore=frozenset(others-{n}))]
            self._event('TERMINAL_PARKING_WAIT',vehicle_id=vehicle,origin=origin,target=target,
                        blocked_by=blockers,waited_s=waited,maximum_wait_s=30.,
                        candidate_eta_s=cost,available_eta_s=available,physical_leases_changed=False)
            self.parking_terminal_wait_logged_at[vehicle]=self.now
        return True

    def _parking_approach_clear(self,route,vehicle):
        # A booked parking place will become a stopped body. Do not choose an
        # ingress that depends on that place remaining empty while en route.
        if any(self.holders.get(node) not in (None,vehicle)
            and self.network.services[node]['kind']=='waiting'
            for node in route.get('must_be_empty_nodes',[])):return False
        if not self.offroad_busy_wait or not hasattr(self.network,'edges'):return True
        from logistics_park.safety.collision import OBB,overlaps
        bodies=[]
        for node,owner in self.holders.items():
            site=self.network.services[node]
            if owner==vehicle or site['kind'] not in ('loading','unloading') or 'rear_xy_m' not in site:continue
            priority=self.admission_priorities.get(owner,(math.inf,False))
            if self.positions.get(owner)!=node and not priority[1]:continue
            x,y=site['rear_xy_m'];h=math.radians(site['heading_deg'])
            bodies.append(OBB(x+.75*math.cos(h),y+.75*math.sin(h),h,8.344,2.6))
        if not bodies:return True
        # Check every service-access segment, including the destination ingress.
        # A dock body can block that ingress without the route visiting its node;
        # entering the approach can then prevent that same dock body from exiting.
        for key in route['edge_ids']:
            edge=self.network.edges[key]
            if edge['kind']!='service_access':continue
            for body in _access_body_samples(tuple(map(tuple,edge['centerline'])),bool(edge.get('reverse'))):
                if any(overlaps(body,other,.35) for other in bodies):return False
        return True


    def _seals_free_waiting(self,node):
        """Parking here would deny another free waiting slot its only exit.

        Waiting slots chain on a spur: Q10 sits between Q9 and the road, so a
        truck parked at Q10 strands an empty Q9 for good. The chain is already
        declared as departure_requires_empty on the inner slot.
        """
        site=self.network.services.get(node)
        if not site or site['kind']!='waiting':return False
        for name,other in self.network.services.items():
            if other['kind']!='waiting' or name==node or name in self.holders:continue
            if node in other.get('departure_requires_empty',[]):return True
        return False

    def _blocks_committed_route(self,node,vehicle,ignore=frozenset()):
        if self.offroad_busy_wait and any(m.vehicle_id!=vehicle and m.vehicle_id not in ignore
                and m.waiting and node in self.network.services[m.destination].get('departure_requires_empty',[])
                for m in self.moves.values()):return True
        if self.offroad_busy_wait and any(m.vehicle_id!=vehicle and m.vehicle_id not in ignore
                and node in m.route.get('must_be_empty_nodes',[]) for m in self.moves.values()):return True
        resources=set(self.network.services[node].get('occupation_resources',[]))
        if any(m.vehicle_id!=vehicle and m.vehicle_id not in ignore and (node in self.live_requirements.get(m.vehicle_id,m.route).get('must_be_empty_nodes',[]) or resources.intersection(self.live_requirements.get(m.vehicle_id,m.route).get('required_resources',[]))) for m in self.moves.values()):return True
        if self._seals_free_waiting(node):return True
        # A stopped body can seal a pending task's only exit without any move
        # to inspect: waiting slots share one spur, so parking in front of a
        # staged truck can strand it (series10_normal_05, Q10 in front of Q9).
        if self.network.services[node]['kind']=='waiting':
            for other,assignment in self.assignments.items():
                if other==vehicle or other in ignore or other in self.moves:continue
                source=self.positions.get(other);target=self._target(assignment)
                if source is None or source==node or target not in self.network.services:continue
                if self.network.services[source]['kind']!='waiting':continue
                if not self.network.route(source,target):continue
                if not self.network.route(source,target,occupied_nodes=frozenset({node})):return True
        return False

    def can_divert_waiting(self,move,destination,*,persistent_block=False):
        policy=self.efficiency_policy
        if policy is None:return True
        if not policy.parking_enabled(destination):return False
        history=self.waiting_history.get(move.command_id,[])
        if any(q==destination and self.now-at<policy.waiting_history_s for q,at in history):return False
        if move.waiting:
            last=history[-1][1] if history else self.now
            if not persistent_block or self.now-last<policy.waiting_commit_s:return False
        return True

    def divert_to_waiting(self,vehicle,destination,route,*,reassign_from=None,persistent_block=False):
        """Retain command identity, original task, cargo and source clearance."""
        old=self.moves[vehicle]
        if reassign_from and self.efficiency_policy:
            raise ValueError('Committed waiting target cannot be lent')
        if not self.can_divert_waiting(old,destination,persistent_block=persistent_block):
            raise ValueError('Committed waiting target or recent revisit')
        ignore=frozenset({reassign_from}) if reassign_from else frozenset()
        if reassign_from:
            lender=self.moves.get(reassign_from)
            if (self.holders.get(destination)!=reassign_from or self.positions.get(reassign_from)==destination
                    or reassign_from not in self.waiting_lenders or not lender or not lender.waiting
                    or lender.destination!=destination):raise ValueError('Waiting promise is not safely transferable')
        if (destination==old.destination or (destination in self.holders and not reassign_from) or self.network.services[destination]['kind']!='waiting'
                or self._blocks_committed_route(destination,vehicle,ignore)):
            raise ValueError('Unavailable diversion destination')
        if any(n in self.holders and self.holders[n]!=vehicle for n in self.network.services[destination].get('departure_requires_empty',[])):
            raise ValueError('Diversion waiting exit is blocked')
        if self.holders.get(old.destination)==vehicle:del self.holders[old.destination]
        self.holders[destination]=vehicle
        if reassign_from:self._event('WAITING_RESERVATION_REASSIGNED',vehicle_id=vehicle,previous_unentered_holder=reassign_from,node=destination)
        new=replace(old,destination=destination,route=route,waiting=True)
        self.moves[vehicle]=new
        self.waiting_history.setdefault(old.command_id,[]).append((destination,self.now))
        self._event('FRONT_VEHICLE_DIVERTED',vehicle_id=vehicle,task_id=old.task_id,previous_destination=old.destination,waiting_node=destination,command_id=old.command_id)
        return new

    def _destination_queue(self,target):
        return sorted((a for a in self.assignments.values() if self._target(a)==target and a.queued_at is not None),
                      key=lambda a:(a.queued_at,a.work.release_time,a.work.task_id))

    def _issue(self, assignment, destination, route, waiting):
        vehicle=assignment.vehicle_id;origin=self.positions[vehicle]
        busy=bool(self._blockers(destination,vehicle))
        if busy and (waiting or not self.follow_to_busy_destination):
            raise ValueError('Destination reserved twice')
        static=None
        if self.traffic_routing is not None:
            static=self.network.route(origin,destination,occupied_nodes=self._occupied(vehicle),transit_warehouses=self._traffic_warehouses(origin,destination))
        self.sequence+=1
        move=DispatchMove(self.sequence,assignment.work.task_id,vehicle,assignment.visit_id,origin,destination,route,waiting)
        priority=self._admission_priority(assignment,self.admission_priorities)
        entry_eta=priority[3] if len(priority)>3 else priority[0]
        near=(not self.efficiency_policy or waiting or self.network.services[destination]['kind']=='waiting'
              or entry_eta<=self.efficiency_policy.berth_near_eta_s)
        selected=self.approach_intents.get(destination)
        if not busy and near and (not self.efficiency_policy or waiting or selected in (None,vehicle)):
            self.holders[destination]=vehicle
        self.moves[vehicle]=move;assignment.state='MOVING'
        if waiting:self.waiting_history[move.command_id]=[(destination,self.now)]
        self._event('MOVE_REQUESTED',vehicle_id=vehicle,task_id=move.task_id,command_id=move.command_id,
                    origin=origin,destination=destination,waiting=waiting,edge_ids=list(route['edge_ids']),estimated_travel_time_s=route['estimated_travel_time_s'],congestion_delay_s=route.get('congestion_delay_s',0),
                    traffic_route_changed=bool(static and static['edge_ids']!=route['edge_ids']),
                    freeflow_choice_edges=static['edge_ids'] if static else None)
        return move

    def _start_service(self,a):
        a.state='SERVICING';a.service_until=self.now+self.service_s;a.queued_at=None
        self._event('SERVICING',vehicle_id=a.vehicle_id,task_id=a.work.task_id,
                    operation=a.operation,site=self._target(a))

    def tick(self, now):
        self._time(now);commands=[]
        for vehicle,a in list(self.assignments.items()):
            if a.state!='SERVICING' or now<a.service_until:continue
            self._event(a.operation+'_COMPLETE',vehicle_id=vehicle,task_id=a.work.task_id,site=self._target(a))
            if a.operation=='LOADING':
                a.cargo_units=1;a.operation='UNLOADING';a.state='READY';a.service_until=None
            else:
                a.cargo_units=0;self.status[a.work.task_id]=WorkStatus.COMPLETE
                self.fleet.finish_task(vehicle,a.visit_id,a.work.task_id,now)
                self._event('TRANSPORT_COMPLETE',vehicle_id=vehicle,task_id=a.work.task_id)
                del self.assignments[vehicle]
        submitted={task_id:i for i,task_id in enumerate(self.tasks)}
        pending=sorted((t for t in self.tasks.values() if self.status[t.task_id]==WorkStatus.PENDING and t.release_time<=now),
                       key=lambda t:(-t.priority,t.release_time,
                           submitted[t.task_id] if self.scheduling.submission_order_dispatch else t.task_id))
        pending=self._continuation_pending(pending)
        rolling_used=False
        pairs=[]
        if self.global_scheduler is not None and pending:
            # Existing active tasks and physical leases are never reassigned.
            if now-self.rolling_plan_at>=self.scheduling.planning_interval_s:
                pairs=self.global_scheduler.plan(self,pending)
                self.rolling_plan_at=now
                self._event('ROLLING_PLAN',**(self.global_scheduler.last_plan or {}))
                rolling_used=pairs is not None
                if pairs is None:pairs=[]
            else:rolling_used=True
        if not rolling_used and self.scheduling.batch_dispatch:pairs=self._batch_pairs(pending)
        for work,vehicle,eta in pairs:
            v=self.fleet.vehicles[vehicle]
            self.fleet.assign_task(vehicle,v.visit_id,work.task_id,now)
            self.assignments[vehicle]=Assignment(work,vehicle,v.visit_id)
            self.status[work.task_id]=WorkStatus.ACTIVE
            self._event('TASK_DISPATCHED',task_id=work.task_id,vehicle_id=vehicle,visit_id=v.visit_id,estimated_pickup_delay_s=eta)
        for work in ([] if rolling_used or self.scheduling.batch_dispatch else pending):
            # Limit work in flight at a loading dock, including tasks already
            # approaching or servicing it. Completed loading releases the slot;
            # physical berth ownership and movement permissions remain separate.
            if self.scheduling.max_loading_inflight and sum(
                    a.operation=='LOADING' and a.work.loading_node==work.loading_node
                    for a in self.assignments.values())>=self.scheduling.max_loading_inflight:
                continue
            candidates=[]
            for vehicle in self.fleet.available_vehicle_ids:
                if vehicle not in self.positions or vehicle in self.moves:continue
                origin=self.positions[vehicle];busy=bool(self._blockers(work.loading_node,vehicle))
                route=self._route(origin,work.loading_node,vehicle,future=busy)
                if not route:continue
                if busy and not self.follow_to_busy_destination and not self._waiting_options(origin,work.loading_node,vehicle):continue
                candidates.append((self._pickup_eta(work,vehicle,route),vehicle))
            if not candidates:continue
            eta,vehicle=min(candidates);v=self.fleet.vehicles[vehicle]
            self.fleet.assign_task(vehicle,v.visit_id,work.task_id,now)
            self.assignments[vehicle]=Assignment(work,vehicle,v.visit_id)
            self.status[work.task_id]=WorkStatus.ACTIVE
            self._event('TASK_DISPATCHED',task_id=work.task_id,vehicle_id=vehicle,visit_id=v.visit_id,estimated_pickup_delay_s=eta)
        if self.offroad_busy_wait:
            self.promote_approaches(self.admission_priorities)
        ordered=sorted(self.assignments.values(),key=lambda a:(a.queued_at is None,a.queued_at or 0,a.work.release_time,a.work.task_id))
        if self.global_scheduler is not None and self.scheduling.calendar_departure_wait and now-self.departure_plan_at>=self.scheduling.planning_interval_s:
            self.departure_not_before=self.global_scheduler.ready_departures(self)
            self.departure_plan_at=now
        for a in ordered:
            if a.state not in ('READY','WAITING'):continue
            vehicle=a.vehicle_id;origin=self.positions[vehicle];target=self._target(a)
            if origin==target:
                self._start_service(a);continue
            queue=self._destination_queue(target)
            # The selected admission owner and queue release must agree.
            # A remote promise cannot block its own owner behind another waiter.
            owns_admission=self.offroad_busy_wait and self.holders.get(target)==vehicle
            first=owns_admission or not queue or queue[0].vehicle_id==vehicle
            busy=bool(self._blockers(target,vehicle))
            if self.efficiency_policy:
                selected=self.approach_intents.get(target)
                first=owns_admission or selected==vehicle
                busy=busy or selected not in (None,vehicle)
            route=self._route(origin,target,vehicle) if not busy and first else None
            if route is None and self.follow_to_busy_destination:
                if self.efficiency_policy and self.network.services[origin]['kind']=='staging':
                    occupied=any(self.positions.get(owner)==target or self.admission_priorities.get(owner,(0,False))[1]
                                 for owner in self._blockers(target,vehicle))
                    # Startup positions are ordinary road bodies, not parking
                    # buffers. Move toward a safe admission gate of a free bay
                    # without taking a permit away from the selected queue head.
                    approach=self._route(origin,target,vehicle,future=True) if not occupied else None
                    if approach:
                        self._event('STAGING_TO_ADMISSION_GATE',vehicle_id=vehicle,target=target)
                        commands.extend(self._checked_issue(a,target,approach,False));continue
                origin_kind=self.network.services[origin]['kind']
                timed_departure=(origin_kind=='waiting' or self.efficiency_policy and origin_kind in ('loading','unloading'))
                if busy and timed_departure:
                    approach=self._route(origin,target,vehicle,future=True)
                    release=max((self.clearance_estimates.get(b,math.inf) for b in self._blockers(target,vehicle)),default=math.inf)
                    eta=approach.get('freeflow_travel_time_s',approach['estimated_travel_time_s']) if approach else 0.
                    if self.offroad_busy_wait and first and approach and math.isfinite(release) and now+eta>=release+3.:
                        self._event('WAITING_ETA_DEPARTURE' if origin_kind=='waiting' else 'DOCK_ETA_DEPARTURE',vehicle_id=vehicle,target=target,arrival_estimate_s=now+eta,clearance_estimate_s=release)
                        commands.extend(self._checked_issue(a,target,approach,False));continue
                    # Already off the road: retain this real parking place
                    # until the berth is available instead of feeding its queue.
                    if origin_kind=='waiting':
                        # A far loaded tail may leave its buffer toward a
                        # proved road hold while earlier trucks unload. It
                        # receives no target lease or queue-head priority.
                        if (self.scheduling.tail_prepositioning and self.efficiency_policy
                                and self.efficiency_policy.proactive_admission
                                and a.operation=='UNLOADING'
                                and all(status!=WorkStatus.PENDING for status in self.status.values())
                                and getattr(self,'departure_guard',None) is not None):
                            priority=self._admission_priority(a,self.admission_priorities)
                            entry_eta=priority[3] if len(priority)>3 else priority[0]
                            if math.isfinite(entry_eta) and entry_eta>self.efficiency_policy.berth_near_eta_s:
                                issued=self._checked_issue(a,target,approach,False) if approach else []
                                if issued:
                                    self._event('WAITING_TAIL_PREPOSITION',vehicle_id=vehicle,
                                        origin=origin,target=target,
                                        queue_head=self.approach_intents.get(target),entry_eta_s=entry_eta)
                                    commands.extend(issued);continue
                        if a.queued_at is None:a.queued_at=now
                        continue
                # A road queue beside a busy berth can obstruct the owner's
                # departure. Use a reachable, unoccupied off-road place first.
                options=self._waiting_options(origin,target,vehicle) if busy and a.state!='WAITING' else []
                if options:
                    _,q,approach=options[0]
                    if a.queued_at is None:a.queued_at=now
                    commands.extend(self._checked_issue(a,q,approach,True));continue
                if self.efficiency_policy and origin_kind in ('loading','unloading'):
                    # A remote queue intent must not retain the source dock
                    # while another truck is already waiting to use it. Leave
                    # only toward a far admission line on a legal standard
                    # route; queue order and local entry protection still hold.
                    source_waiters=self._source_waiters(origin,vehicle)
                    priority=self._admission_priority(a,self.admission_priorities)
                    gate_eta=priority[3] if len(priority)>3 else priority[0]
                    approach=self._route(origin,target,vehicle,future=True)
                    if source_waiters and approach and math.isfinite(gate_eta) and gate_eta>self.efficiency_policy.berth_near_eta_s:
                        self._event('DOCK_SOURCE_CLEARANCE_DEPARTURE',vehicle_id=vehicle,origin=origin,
                            target=target,source_waiters=sorted(source_waiters),
                            queue_head=self.approach_intents.get(target),entry_eta_s=gate_eta)
                        commands.extend(self._checked_issue(a,target,approach,False));continue
                if (self.scheduling.tail_prepositioning and self.efficiency_policy
                        and self.efficiency_policy.proactive_admission
                        and origin_kind=='loading' and a.operation=='UNLOADING'
                        and all(status!=WorkStatus.PENDING for status in self.status.values())
                        and getattr(self,'departure_guard',None) is not None):
                    # During the final assigned workload, overlap a loaded
                    # journey with another truck's unloading. The queue intent
                    # still controls the target berth, and the existing guard
                    # must prove a real hold clear of every occupied dock exit.
                    priority=self._admission_priority(a,self.admission_priorities)
                    entry_eta=priority[3] if len(priority)>3 else priority[0]
                    if math.isfinite(entry_eta) and entry_eta>self.efficiency_policy.berth_near_eta_s:
                        approach=self._route(origin,target,vehicle,future=True)
                        issued=self._checked_issue(a,target,approach,False) if approach else []
                        if issued:
                            self._event('DOCK_TAIL_PREPOSITION',vehicle_id=vehicle,origin=origin,
                                target=target,queue_head=self.approach_intents.get(target),entry_eta_s=entry_eta)
                            commands.extend(issued);continue
                if self.offroad_busy_wait:
                    if a.queued_at is None:a.queued_at=now
                    if not self.efficiency_policy or now-self.hold_logged_at.get(vehicle,-math.inf)>=5.:
                        self._event('BUSY_BAY_OFFROAD_HOLD',vehicle_id=vehicle,target=target,origin=origin)
                        self.hold_logged_at[vehicle]=now
                    continue
                approach=self._route(origin,target,vehicle,future=True)
                if approach:
                    if a.queued_at is None:a.queued_at=now
                    commands.extend(self._checked_issue(a,target,approach,False));continue
            if route:
                if self.traffic_routing is not None:
                    if a.traffic_wait_since is not None:
                        if not self.traffic_routing.may_resume(route,self.now-a.traffic_wait_since):continue
                        self._event('TRAFFIC_WAIT_RELEASED',vehicle_id=vehicle,target=target,waited_s=self.now-a.traffic_wait_since)
                        a.traffic_wait_since=None
                    elif a.state!='WAITING' and route.get('congestion_delay_s',0)>=self.traffic_routing.enter_wait_s:
                        direct=({**route,'estimated_travel_time_s':self._score_route(origin,target,route)}
                                if self.scheduling.waiting_completion_cost else route)
                        choice=self.traffic_routing.waiting_choice(direct,self._waiting_options(origin,target,vehicle))
                        if choice:
                            cost,q,approach=choice;a.traffic_wait_since=self.now
                            if a.queued_at is None:a.queued_at=self.now
                            self._event('TRAFFIC_WAIT_SELECTED',vehicle_id=vehicle,target=target,waiting_node=q,direct_eta_s=route['estimated_travel_time_s'],waiting_estimate_s=cost)
                            commands.extend(self._checked_issue(a,q,approach,True));continue
                commands.extend(self._checked_issue(a,target,route,False));continue
            if a.queued_at is None:a.queued_at=now
            if a.state=='WAITING':continue
            options=self._waiting_options(origin,target,vehicle)
            if options:
                _,q,route=options[0];commands.extend(self._checked_issue(a,q,route,True))
            else:
                # Remain at the already-owned node; no unreserved road queue.
                self._event('WAITING_UNAVAILABLE',vehicle_id=vehicle,task_id=a.work.task_id,target=target)
        # Idle trucks must not permanently occupy service access corridors.
        # Repositioning is a real movement with a source lease and target lease,
        # not deletion of the parked truck from the occupancy model.
        if self.park_idle_vehicles or any(s!=WorkStatus.COMPLETE for s in self.status.values()):
            parking_plan=self._parking_plan() if self.scheduling.parking_lookahead else {}
            for vehicle in self.fleet.available_vehicle_ids:
                if vehicle not in self.positions or vehicle in self.moves:continue
                origin=self.positions[vehicle]
                if self.network.services[origin]['kind']=='waiting':continue
                needed={node for t in self.tasks.values() if self.status[t.task_id]!=WorkStatus.COMPLETE
                        for node in (t.loading_node,t.unloading_node)}
                blocks_target=any(vehicle in self._blockers(node,'') for node in needed)
                blocks_route=False
                for other,a in self.assignments.items():
                    if a.state not in ('READY','WAITING') or other not in self.positions:continue
                    start=self.positions[other];target=self._target(a)
                    if start==target or self._route(start,target,other):continue
                    if self.network.route(start,target,occupied_nodes=self._occupied(other)-{origin}):
                        blocks_route=True;break
                if not blocks_target and not blocks_route:
                    for work in pending:
                        if self.status[work.task_id]!=WorkStatus.PENDING:continue
                        for other in self.fleet.available_vehicle_ids:
                            if other==vehicle or other not in self.positions:continue
                            start=self.positions[other]
                            if start==work.loading_node or self._route(start,work.loading_node,other):continue
                            if self.network.route(start,work.loading_node,occupied_nodes=self._occupied(other)-{origin}):
                                blocks_route=True;break
                        if blocks_route:break
                if not blocks_target and not blocks_route and not self.park_idle_vehicles:continue
                options=[]
                for q,node in self.network.services.items():
                    if node['kind']!='waiting' or q in self.holders:continue
                    if self.efficiency_policy and not self.efficiency_policy.parking_enabled(q):continue
                    if self.protect_committed_routes and self._blocks_committed_route(q,vehicle):continue
                    if any(n in self.holders for n in node.get('departure_requires_empty',[])):continue
                    route=self._route(origin,q,vehicle)
                    if not route or not self._parking_approach_clear(route,vehicle):continue
                    guard=getattr(self,'parking_departure_guard',getattr(self,'departure_guard',None))
                    if guard is not None and not guard(vehicle,origin,q,route,True):continue
                    options.append((route['estimated_travel_time_s'],q,route))
                if not options:continue
                terminal=self._last_terminal_parking_vehicle(vehicle)
                if terminal and self._wait_for_terminal_parking(vehicle,origin,options):continue
                future_targets=getattr(self,'parking_future_targets',{}) if self.scheduling.parking_tail_reserve else {}
                planned=parking_plan.get(vehicle,future_targets.get(vehicle))
                future_sites={q for owner,q in future_targets.items() if owner!=vehicle}
                unreserved=[item for item in options if item[1] not in future_sites
                    and not set(self.network.services[item[1]].get('departure_requires_empty',[])) & future_sites]
                if future_targets:
                    source_needed=blocks_target and any(m.vehicle_id!=vehicle and m.destination==origin
                        for m in self.moves.values())
                    if unreserved:options=unreserved
                    elif not source_needed:continue
                # For the last stationary truck there is no matching tradeoff.
                # Re-evaluate live options after a blocker clears, even if the
                # five-second parking-plan cache still names a farther site.
                _,q,route=min(options,key=lambda v:(self._score_route(origin,v[1],v[2]),v[1])
                    if terminal else (v[1]!=planned,v[0],v[1]));v=self.fleet.vehicles[vehicle]
                self.parking_terminal_wait_since.pop(vehicle,None)
                self.parking_terminal_wait_logged_at.pop(vehicle,None)
                if q in future_sites:
                    self.parking_future_targets={owner:node for owner,node in future_targets.items() if node!=q}
                    self.parking_plan_at=-math.inf
                    self._event('TAIL_PARKING_PREFERENCE_YIELDED',vehicle_id=vehicle,node=q,
                        reason='clear_actual_source_for_inbound_vehicle',physical_leases_revoked=False)
                self.sequence+=1;task_id='reposition-'+str(self.sequence)
                self.fleet.assign_task(vehicle,v.visit_id,task_id,now)
                move=DispatchMove(self.sequence,task_id,vehicle,v.visit_id,origin,q,route,True,'REPOSITION')
                self.holders[q]=vehicle;self.moves[vehicle]=move;commands.append(move)
                self._event('IDLE_REPOSITION_REQUESTED',vehicle_id=vehicle,origin=origin,destination=q)
        return commands

    def _checked_issue(self,a,destination,route,waiting):
        suggestion=self.departure_not_before.get(a.vehicle_id)
        origin=self.positions[a.vehicle_id]
        key=(a.vehicle_id,a.work.task_id,a.operation,origin)
        if (self.global_scheduler is not None and not waiting and suggestion
                and suggestion[:3]==(a.work.task_id,a.operation,origin)
                and self.now-self.departure_plan_at<=self.scheduling.planning_interval_s
                and self.now<suggestion[3] and not self._source_waiters(origin,a.vehicle_id)):
            since=self.departure_plan_wait_since.setdefault(key,self.now)
            if self.now-since<20.:
                if self.now-self.hold_logged_at.get(key,-math.inf)>=5.:
                    self.hold_logged_at[key]=self.now
                    self._event('ROLLING_SOURCE_WAIT',vehicle_id=a.vehicle_id,origin=origin,
                                target=destination,not_before=suggestion[3],maximum_wait_s=20.,physical_leases_changed=False)
                return []
        guard=getattr(self,'departure_guard',None)
        if guard is None or guard(a.vehicle_id,self.positions[a.vehicle_id],destination,route,waiting):
            self.departure_rejected_at.pop((a.vehicle_id,destination),None)
            return [self._issue(a,destination,route,waiting)]
        self.departure_rejected_at[a.vehicle_id,destination]=self.now
        # A blocked departure should choose an actual off-road buffer now,
        # rather than leaving the source and searching after it gets stuck.
        if not waiting:
            origin=self.positions[a.vehicle_id]
            for _,q,approach in self._waiting_options(origin,destination,a.vehicle_id):
                if guard(a.vehicle_id,origin,q,approach,True):
                    self._event('PREDEPARTURE_WAITING_SELECTED',vehicle_id=a.vehicle_id,node=q,target=destination)
                    return [self._issue(a,q,approach,True)]
        return []

    def ready_to_finish(self):
        """Task completion includes real stopped parking when that policy is enabled."""
        if self.moves or any(s!=WorkStatus.COMPLETE for s in self.status.values()):return False
        if not self.park_idle_vehicles:return True
        return all(vehicle in self.positions and self.network.services[self.positions[vehicle]]['kind']=='waiting'
                   for vehicle in self.fleet.available_vehicle_ids)

    def _admission_priority(self,a,priorities):
        if a.vehicle_id in (priorities or {}):return priorities[a.vehicle_id]
        move=self.moves.get(a.vehicle_id)
        if move:return (math.inf,False)
        origin=self.positions.get(a.vehicle_id)
        route=self._route(origin,self._target(a),a.vehicle_id,future=True) if origin is not None else None
        return (route['estimated_travel_time_s'] if route else math.inf,False)

    def _admission_rank(self,a,priorities):
        eta=self._admission_priority(a,priorities)[0]
        waited=max(0.,self.now-a.queued_at) if a.queued_at is not None else 0.
        # A bounded credit balances long waits without letting a distant truck
        # indefinitely monopolize a berth ahead of imminent arrivals.
        credit=min(30.,.1*waited) if self.offroad_busy_wait else 0.
        return (eta-credit,a.queued_at is None,a.queued_at or 0.,a.work.task_id,a.vehicle_id)

    def _actual_source_blocked(self,vehicle,target):
        row=self.actual_departure_blockers.get((vehicle,target))
        return bool(self.scheduling.rolling_scheduler and row and row['blocked_by']
                    and 0<=self.now-row['time']<5. and self.positions.get(vehicle)==row['source'])

    def promote_approaches(self,priorities=None):
        if not self.follow_to_busy_destination:return
        if priorities is not None:self.admission_priorities=dict(priorities)
        by_target={};previous_intents=self.approach_intents;self.approach_intents={}
        intent_since=getattr(self,'berth_intent_since',None)
        if intent_since is None:self.berth_intent_since=intent_since={}
        for a in self.assignments.values():
            move=self.moves.get(a.vehicle_id)
            if move:by_target.setdefault(move.destination,[]).append(a)
            elif a.state in ('READY','WAITING') and a.vehicle_id in self.positions and (self.offroad_busy_wait or priorities and a.vehicle_id in priorities):
                target=self._target(a)
                if self.offroad_busy_wait and not self._route(self.positions[a.vehicle_id],target,a.vehicle_id,
                        future=self.efficiency_policy is not None):continue
                by_target.setdefault(target,[]).append(a)
        for target,candidates in by_target.items():
            resources=set(self.network.services[target].get('occupation_resources',[]))
            candidate_ids={a.vehicle_id for a in candidates}
            if self.scheduling.connected_clearance:
                # Adjacent loading/unloading bays share one clearance space.
                # A follower at a waiting site must not acquire the other
                # bay and then freeze the lane leader at its admission line.
                candidate_ids.update(a.vehicle_id for node,rows in by_target.items()
                    if (resources.intersection(self.network.services[node].get('occupation_resources',[]))
                        or self.network.services[target].get('warehouse')
                        and self.network.services[target].get('warehouse')==self.network.services[node].get('warehouse'))
                    for a in rows)
            reservations={node:owner for node,owner in self.holders.items()
                          if node==target or resources.intersection(self.network.services[node].get('occupation_resources',[]))}
            if self.scheduling.connected_clearance and priorities is not None:
                warehouse=self.network.services[target].get('warehouse')
                for node,owner in self.holders.items():
                    if not warehouse or self.network.services[node].get('warehouse')!=warehouse:continue
                    owner_priority=priorities.get(owner,())
                    owner_leaders=owner_priority[2] if len(owner_priority)>2 else frozenset()
                    if any(a.vehicle_id in owner_leaders or owner in (
                            self._admission_priority(a,priorities)[2]
                            if len(self._admission_priority(a,priorities))>2 else frozenset()) for a in candidates):
                        reservations[node]=owner
            # A shared dock promise may belong to the other bay at this warehouse.
            # Only future destination promises can move; parked/source bodies and
            # trucks already committed to their final approach keep their leases.
            protected=set()
            for node,owner in reservations.items():
                move=self.moves.get(owner)
                if (self.positions.get(owner)==node or (not move and not self.offroad_busy_wait) or (move and move.destination!=node and not self.offroad_busy_wait)
                        or priorities is None or priorities.get(owner,(0,not self.offroad_busy_wait))[1]):
                    protected.add(owner)
            if self.scheduling.rolling_scheduler:
                for node,owner in list(reservations.items()):
                    if owner not in protected and owner not in self.moves and self._actual_source_blocked(owner,node):
                        if self.holders.get(node)==owner:del self.holders[node]
                        del reservations[node]
                        self._event('SOURCE_EXIT_BLOCKED_PROMISE_RELEASED',vehicle_id=owner,node=node,
                                    physical_occupancy_retained=True)
            if (self.scheduling.connected_clearance and self.efficiency_policy and not protected
                    and any(a.vehicle_id in self.moves and self.moves[a.vehicle_id].destination==target for a in candidates)):
                for node,owner in list(reservations.items()):
                    if (node==target and owner not in self.moves
                            and self.now-self.departure_rejected_at.get((owner,target),-math.inf)<5.):
                        del self.holders[node];del reservations[node]
                        self._event('UNENTERED_DEPARTURE_BLOCKED_PROMISE_RELEASED',vehicle_id=owner,node=node,physical_occupancy_retained=True)
            candidates.sort(key=lambda a:self._admission_rank(a,priorities))
            if self.efficiency_policy and self.network.services[target]['kind'] in ('loading','unloading'):
                # A physical owner still keeps its bay. Separately select the
                # next queue intent so a proven departing owner can be followed
                # from a waiting place without stealing its actual occupancy.
                eligible=[a for a in candidates if a.vehicle_id in protected] or candidates
                if not protected and self.scheduling.rolling_scheduler:
                    eligible=[a for a in eligible if not self._actual_source_blocked(a.vehicle_id,target)]
                    if not eligible:continue
                if self.scheduling.connected_clearance and not protected:
                    # A parked truck whose real departure check just failed
                    # cannot freeze an already approaching truck at the berth
                    # gate. Only its future intent may yield; occupied source
                    # leases and entered destination permissions stay intact.
                    approaching=[a for a in eligible if a.vehicle_id in self.moves
                        and self.moves[a.vehicle_id].destination==target]
                    ready=[a for a in eligible if not (
                        a.vehicle_id not in self.moves
                        and self.now-self.departure_rejected_at.get((a.vehicle_id,target),-math.inf)<5.)]
                    if approaching and ready:eligible=ready
                # Queue intent is a permission dependency even while the
                # dock remains physically occupied. A faster ETA must not
                # select a follower whose same-dock leader cannot pass it.
                front=[a for a in eligible if not (
                    (self._admission_priority(a,priorities)[2]
                     if len(self._admission_priority(a,priorities))>2 else frozenset())
                    & {candidate.vehicle_id for candidate in candidates})]
                if front:eligible=front
                margin=self.scheduling.berth_intent_hysteresis_s
                incumbent=next((a for a in eligible if a.vehicle_id==previous_intents.get(target)),None)
                if (margin and incumbent is not None
                        and self.now-intent_since.get(target,self.now)<2*margin):
                    best_score=self._admission_rank(eligible[0],priorities)[0]
                    incumbent_score=self._admission_rank(incumbent,priorities)[0]
                    if (math.isfinite(incumbent_score) and math.isfinite(best_score)
                            and incumbent_score<=best_score+margin):
                        # Only stabilize an eligible queue intent. Body owners,
                        # lane leaders and blocked departures were filtered above.
                        # The normal exit checks below still authorize each grant.
                        eligible=[incumbent]+[a for a in eligible if a is not incumbent]
                        candidates=[incumbent]+[a for a in candidates if a is not incumbent]
                selected=eligible[0].vehicle_id
                self.approach_intents[target]=selected
                if previous_intents.get(target)!=selected:
                    self._event('BERTH_QUEUE_INTENT',vehicle_id=selected,node=target,
                                eta_s=self._admission_priority(eligible[0],priorities)[0])
            for a in candidates:
                vehicle=a.vehicle_id
                if not protected and vehicle not in self.moves and self._actual_source_blocked(vehicle,target):continue
                if (self.scheduling.connected_clearance and self.efficiency_policy
                        and not protected and vehicle not in self.moves
                        and self.now-self.departure_rejected_at.get((vehicle,target),-math.inf)<5.
                        and (any(c.vehicle_id in self.moves and self.moves[c.vehicle_id].destination==target for c in candidates)
                             or self._actual_source_blocked(vehicle,target))):
                    if self.holders.get(target)==vehicle:
                        del self.holders[target]
                        self._event('UNENTERED_DEPARTURE_BLOCKED_PROMISE_RELEASED',vehicle_id=vehicle,node=target,physical_occupancy_retained=True)
                    continue
                if protected-{vehicle}:continue
                previous={owner for owner in reservations.values() if owner!=vehicle}
                exit_blockers=self._blockers(target,vehicle)-previous
                if exit_blockers:
                    if self.holders.get(target)==vehicle and vehicle not in protected:
                        del self.holders[target]
                        self._event('BERTH_UNENTERED_PROMISE_EXIT_BLOCKED',vehicle_id=vehicle,node=target,
                                    blocked_by=sorted(exit_blockers),physical_occupancy_retained=True)
                    continue
                priority=self._admission_priority(a,priorities)
                leaders=priority[2] if len(priority)>2 else frozenset()
                if leaders & candidate_ids:continue
                score=self._admission_rank(a,priorities)[0]
                if priorities is not None and any(score+.5>self._admission_rank(self.assignments[owner],priorities)[0]
                        and not (len(priorities.get(owner,()))>2 and vehicle in priorities[owner][2])
                        for owner in previous if owner in self.assignments):continue
                if self.efficiency_policy and self.network.services[target]['kind'] in ('loading','unloading'):
                    selected_before=self.approach_intents.get(target)
                    self.approach_intents[target]=vehicle
                    if selected_before!=vehicle:
                        self._event('BERTH_QUEUE_INTENT',vehicle_id=vehicle,node=target,eta_s=priority[0])
                    entry_eta=priority[3] if len(priority)>3 else priority[0]
                    if not priority[1] and entry_eta>self.efficiency_policy.berth_near_eta_s:
                        for node,owner in reservations.items():
                            if owner not in protected and self.holders.get(node)==owner:
                                del self.holders[node]
                                self._event('REMOTE_BERTH_PROMISE_RELEASED',vehicle_id=owner,node=node,eta_s=priority[0])
                        break
                holder=self.holders.get(target)
                for node,owner in reservations.items():
                    if owner==vehicle:continue
                    if self.holders.get(node)!=owner:continue
                    del self.holders[node]
                    if node!=target:
                        self._event('UNENTERED_DOCK_RESERVATION_REASSIGNED',vehicle_id=vehicle,
                                    node=target,previous_node=node,previous_unentered_holder=owner)
                if holder!=vehicle:
                    self.holders[target]=vehicle
                    self._event('APPROACH_BERTH_GRANTED',vehicle_id=vehicle,node=target,previous_unentered_holder=holder)
                break

        for target,vehicle in self.approach_intents.items():
            if previous_intents.get(target)!=vehicle or target not in intent_since:
                intent_since[target]=self.now
        for target in list(intent_since):
            if target not in self.approach_intents:del intent_since[target]

    def departed(self, command_id, vehicle, visit_id, now, *, tail_clear):
        self._time(now);move=self.moves[vehicle]
        if (self.fleet.vehicles[vehicle].visit_id!=visit_id
                or move.command_id!=command_id or move.visit_id!=visit_id or not tail_clear
                or command_id in self.departures):raise ValueError('Actual tail-clear departure required')
        if self.holders.get(move.origin)!=vehicle:raise ValueError('Source lease lost')
        del self.holders[move.origin];del self.positions[vehicle];self.departures.add(command_id)
        self._event('SOURCE_CLEARED',vehicle_id=vehicle,node=move.origin,command_id=command_id)

    def arrived(self, command_id, vehicle, visit_id, now, *, stopped):
        self._time(now);move=self.moves[vehicle]
        if (self.fleet.vehicles[vehicle].visit_id!=visit_id
                or move.command_id!=command_id or move.visit_id!=visit_id or not stopped
                or command_id not in self.departures):raise ValueError('Stopped arrival after actual departure required')
        if self.holders.get(move.destination)!=vehicle:raise ValueError('Arrival without destination lease')
        self.positions[vehicle]=move.destination;del self.moves[vehicle];self.departures.remove(command_id)
        if move.purpose=='REPOSITION':
            self.fleet.finish_task(vehicle,visit_id,move.task_id,now)
        else:
            a=self.assignments[vehicle];a.state='WAITING' if move.waiting else 'READY'
            if move.waiting and a.traffic_wait_since is not None:a.traffic_wait_since=now
        self._event('NODE_ARRIVED',vehicle_id=vehicle,node=move.destination,task_id=move.task_id,waiting=move.waiting)

    def resource_snapshot(self,vehicle):
        """Task-layer leases only; aggregate with road/gate managers for exits."""
        return frozenset('berth:'+n for n,v in self.holders.items() if v==vehicle)

    def release_for_exit(self,vehicle,visit_id,now,*,tail_clear):
        """Gate backend releases the source berth after actual tail clearance."""
        self._time(now);v=self.fleet.vehicles[vehicle]
        if (v.visit_id!=visit_id or v.phase!=FleetPhase.EXITING or not tail_clear
                or vehicle in self.assignments or vehicle in self.moves):
            raise ValueError('Exit must be authorized and physically clear of its source node')
        node=self.positions[vehicle]
        if self.holders.get(node)!=vehicle:raise ValueError('Source lease lost')
        del self.positions[vehicle];del self.holders[node]
        self._event('EXIT_SOURCE_CLEARED',vehicle_id=vehicle,node=node)
