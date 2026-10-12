"""Greedy ETA task dispatch above lane/road reservation and vehicle control."""
from dataclasses import dataclass,replace
from enum import Enum
import math

from logistics_park.common.messages import finite
from .fleet_lifecycle import FleetPhase, FleetRegistry


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
    def __init__(self, network, fleet: FleetRegistry, service_s=5.,protect_committed_routes=False,traffic_routing=None,follow_to_busy_destination=False,park_idle_vehicles=False,offroad_busy_wait=False,efficiency_policy=None):
        finite(service_s)
        if service_s<0:raise ValueError('Negative service duration')
        self.network=network;self.fleet=fleet;self.service_s=service_s
        self.tasks={};self.status={};self.assignments={};self.positions={}
        self.holders={};self.moves={};self.departures=set();self.events=[]
        self.follow_to_busy_destination=follow_to_busy_destination
        self.park_idle_vehicles=park_idle_vehicles
        self.offroad_busy_wait=offroad_busy_wait;self.clearance_estimates={};self.recovery_estimates={}
        self.admission_priorities={}
        from .efficiency_policy import EfficiencyPolicy
        self.efficiency_policy=EfficiencyPolicy(**efficiency_policy) if efficiency_policy is not None else None
        self.approach_intents={};self.waiting_history={}
        self.hold_logged_at={}
        self.live_requirements={}
        self.waiting_lenders={}
        self.now=0.;self.sequence=0;self.queue_cache={};self.protect_committed_routes=protect_committed_routes;self.traffic_routing=traffic_routing

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
        return self.network.route(origin,target,occupied_nodes=occupied,**kwargs)

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
                options.append((approach['estimated_travel_time_s']+onward['estimated_travel_time_s'],q,approach))
        return sorted(options,key=lambda item:((item[2]['waiting_detour_m'],item[0],item[1]) if self.offroad_busy_wait else (item[0],item[1])))

    def _parking_approach_clear(self,route,vehicle):
        # A booked parking place will become a stopped body. Do not choose an
        # ingress that depends on that place remaining empty while en route.
        if any(self.holders.get(node) not in (None,vehicle)
            and self.network.services[node]['kind']=='waiting'
            for node in route.get('must_be_empty_nodes',[])):return False
        if not self.offroad_busy_wait or not hasattr(self.network,'edges'):return True
        from logistics_park.map.geometry import Polyline
        from logistics_park.safety.collision import OBB,overlaps
        bodies=[]
        for node,owner in self.holders.items():
            site=self.network.services[node]
            if owner==vehicle or site['kind'] not in ('loading','unloading') or 'rear_xy_m' not in site:continue
            priority=self.admission_priorities.get(owner,(math.inf,False))
            if self.positions.get(owner)!=node and not priority[1]:continue
            x,y=site['rear_xy_m'];h=math.radians(site['heading_deg'])
            bodies.append(OBB(x+.75*math.cos(h),y+.75*math.sin(h),h,8.344,2.6))
        # A real/committed adjacent dock body can block the immediate parking
        # exit even when the graph does not visit that dock node explicitly.
        for key in route['edge_ids']:
            edge=self.network.edges[key]
            if edge['kind']!='service_access':break
            line=Polyline(tuple(map(tuple,edge['centerline'])))
            for index in range(math.ceil(line.length/.5)+1):
                (x,y),h=line.at(min(index*.5,line.length));h+=math.pi if edge.get('reverse') else 0
                body=OBB(x+.75*math.cos(h),y+.75*math.sin(h),h,8.344,2.6)
                if any(overlaps(body,other,.35) for other in bodies):return False
        return True

    def _blocks_committed_route(self,node,vehicle,ignore=frozenset()):
        if self.offroad_busy_wait and any(m.vehicle_id!=vehicle and m.vehicle_id not in ignore
                and m.waiting and node in self.network.services[m.destination].get('departure_requires_empty',[])
                for m in self.moves.values()):return True
        if self.offroad_busy_wait and any(m.vehicle_id!=vehicle and m.vehicle_id not in ignore
                and node in m.route.get('must_be_empty_nodes',[]) for m in self.moves.values()):return True
        resources=set(self.network.services[node].get('occupation_resources',[]))
        return any(m.vehicle_id!=vehicle and m.vehicle_id not in ignore and (node in self.live_requirements.get(m.vehicle_id,m.route).get('must_be_empty_nodes',[]) or resources.intersection(self.live_requirements.get(m.vehicle_id,m.route).get('required_resources',[]))) for m in self.moves.values())

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
        pending=sorted((t for t in self.tasks.values() if self.status[t.task_id]==WorkStatus.PENDING and t.release_time<=now),
                       key=lambda t:(-t.priority,t.release_time,t.task_id))
        for work in pending:
            candidates=[]
            for vehicle in self.fleet.available_vehicle_ids:
                if vehicle not in self.positions or vehicle in self.moves:continue
                origin=self.positions[vehicle];busy=bool(self._blockers(work.loading_node,vehicle))
                route=self._route(origin,work.loading_node,vehicle,future=busy)
                if not route:continue
                if busy and not self.follow_to_busy_destination and not self._waiting_options(origin,work.loading_node,vehicle):continue
                wait=self.service_s*(int(busy)+len(self._destination_queue(work.loading_node)))
                candidates.append((route['estimated_travel_time_s']+wait,vehicle))
            if not candidates:continue
            eta,vehicle=min(candidates);v=self.fleet.vehicles[vehicle]
            self.fleet.assign_task(vehicle,v.visit_id,work.task_id,now)
            self.assignments[vehicle]=Assignment(work,vehicle,v.visit_id)
            self.status[work.task_id]=WorkStatus.ACTIVE
            self._event('TASK_DISPATCHED',task_id=work.task_id,vehicle_id=vehicle,visit_id=v.visit_id,estimated_pickup_delay_s=eta)
        if self.offroad_busy_wait:
            self.promote_approaches(self.admission_priorities)
        ordered=sorted(self.assignments.values(),key=lambda a:(a.queued_at is None,a.queued_at or 0,a.work.release_time,a.work.task_id))
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
                        choice=self.traffic_routing.waiting_choice(route,self._waiting_options(origin,target,vehicle))
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
                    if route and self._parking_approach_clear(route,vehicle):options.append((route['estimated_travel_time_s'],q,route))
                if not options:continue
                _,q,route=min(options,key=lambda v:(v[0],v[1]));v=self.fleet.vehicles[vehicle]
                self.sequence+=1;task_id='reposition-'+str(self.sequence)
                self.fleet.assign_task(vehicle,v.visit_id,task_id,now)
                move=DispatchMove(self.sequence,task_id,vehicle,v.visit_id,origin,q,route,True,'REPOSITION')
                self.holders[q]=vehicle;self.moves[vehicle]=move;commands.append(move)
                self._event('IDLE_REPOSITION_REQUESTED',vehicle_id=vehicle,origin=origin,destination=q)
        return commands

    def _checked_issue(self,a,destination,route,waiting):
        guard=getattr(self,'departure_guard',None)
        if guard is None or guard(a.vehicle_id,self.positions[a.vehicle_id],destination,route,waiting):
            return [self._issue(a,destination,route,waiting)]
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

    def promote_approaches(self,priorities=None):
        if not self.follow_to_busy_destination:return
        if priorities is not None:self.admission_priorities=dict(priorities)
        by_target={};previous_intents=self.approach_intents;self.approach_intents={}
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
            reservations={node:owner for node,owner in self.holders.items()
                          if node==target or resources.intersection(self.network.services[node].get('occupation_resources',[]))}
            # A shared dock promise may belong to the other bay at this warehouse.
            # Only future destination promises can move; parked/source bodies and
            # trucks already committed to their final approach keep their leases.
            protected=set()
            for node,owner in reservations.items():
                move=self.moves.get(owner)
                if (self.positions.get(owner)==node or (not move and not self.offroad_busy_wait) or (move and move.destination!=node and not self.offroad_busy_wait)
                        or priorities is None or priorities.get(owner,(0,not self.offroad_busy_wait))[1]):
                    protected.add(owner)
            candidates.sort(key=lambda a:self._admission_rank(a,priorities))
            if self.efficiency_policy and self.network.services[target]['kind'] in ('loading','unloading'):
                # A physical owner still keeps its bay. Separately select the
                # next queue intent so a proven departing owner can be followed
                # from a waiting place without stealing its actual occupancy.
                eligible=[a for a in candidates if a.vehicle_id in protected] or candidates
                # Queue intent is a permission dependency even while the
                # dock remains physically occupied. A faster ETA must not
                # select a follower whose same-dock leader cannot pass it.
                candidate_ids={a.vehicle_id for a in candidates}
                front=[a for a in eligible if not (
                    (self._admission_priority(a,priorities)[2]
                     if len(self._admission_priority(a,priorities))>2 else frozenset())
                    & candidate_ids)]
                if front:eligible=front
                selected=eligible[0].vehicle_id
                self.approach_intents[target]=selected
                if previous_intents.get(target)!=selected:
                    self._event('BERTH_QUEUE_INTENT',vehicle_id=selected,node=target,
                                eta_s=self._admission_priority(eligible[0],priorities)[0])
            for a in candidates:
                vehicle=a.vehicle_id
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
                if leaders & {candidate.vehicle_id for candidate in candidates}:continue
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
                            if owner not in protected:
                                del self.holders[node]
                                self._event('REMOTE_BERTH_PROMISE_RELEASED',vehicle_id=owner,node=node,eta_s=priority[0])
                        break
                holder=self.holders.get(target)
                for node,owner in reservations.items():
                    if owner==vehicle:continue
                    del self.holders[node]
                    if node!=target:
                        self._event('UNENTERED_DOCK_RESERVATION_REASSIGNED',vehicle_id=vehicle,
                                    node=target,previous_node=node,previous_unentered_holder=owner)
                if holder!=vehicle:
                    self.holders[target]=vehicle
                    self._event('APPROACH_BERTH_GRANTED',vehicle_id=vehicle,node=target,previous_unentered_holder=holder)
                break

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
