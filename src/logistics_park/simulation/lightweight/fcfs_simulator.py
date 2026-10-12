"""Concurrent transport on temporary fixtures, guarded by local FCFS junctions."""
import json
from dataclasses import dataclass
from math import hypot, sqrt
from pathlib import Path

from logistics_park.common.enums import TaskStatus
from logistics_park.common.messages import ControlCommand, VehicleState, finite
from logistics_park.map.geometry import wrap_angle
from logistics_park.map.map_matching import match_pose, update_route_match
from logistics_park.safety.collision import OBB, overlaps
from logistics_park.scheduling.fcfs import EntranceRequest, FCFSScheduler
from logistics_park.scheduling.road_capacity import RoadCapacity
from .task_simulator import TaskSimulator


@dataclass(frozen=True)
class TrafficRules:
    request_distance: float = .35
    stop_offset: float = .12
    entry_gap: float = .20
    standstill_gap: float = .05
    time_headway: float = .4

    def __post_init__(self):
        finite(*self.__dict__.values())
        if min(self.__dict__.values()) <= 0 or self.request_distance <= self.stop_offset:
            raise ValueError('invalid traffic rules')


@dataclass(frozen=True)
class IntersectionControl:
    intersection_id: str
    bounds: tuple[float,float,float,float]
    connector_exits: dict[str,str]

    def __post_init__(self):
        finite(*self.bounds)
        x0,y0,x1,y1=self.bounds
        if not self.intersection_id or x1<=x0 or y1<=y0 or not self.connector_exits:
            raise ValueError('invalid intersection control')

    @property
    def box(self):
        x0,y0,x1,y1=self.bounds
        return OBB((x0+x1)/2,(y0+y1)/2,0,x1-x0,y1-y0)


class FCFSTrafficSimulator(TaskSimulator):
    result_prefix='PHASE_4_FCFS_CLOSED_LOOP'
    execution_policy='concurrent_roads_local_fcfs'

    def __init__(self, graph, config, stations, vehicle_nodes, tasks,
                 intersections: list[IntersectionControl], rules: TrafficRules = TrafficRules(),
                 initial_edges: dict[str,str] | None = None):
        super().__init__(graph,config,stations,vehicle_nodes,tasks)
        self.rules=rules
        radius=hypot(config.vehicle.length/2+config.safety_margin, config.vehicle.width/2+config.safety_margin)
        if rules.stop_offset < radius+config.safety_margin+.005:
            raise ValueError('stop offset does not protect the whole vehicle envelope')
        self.intersections={item.intersection_id:item for item in intersections}
        if len(self.intersections)!=len(intersections):
            raise ValueError('duplicate intersection controls')
        self.connector_controls={}
        for spec in intersections:
            for connector,exit_edge in spec.connector_exits.items():
                if connector in self.connector_controls or not graph.is_connected(connector,exit_edge):
                    raise ValueError('invalid controlled connector or exit')
                self.connector_controls[connector]=spec
                for x,y in graph.edges[connector].centerline.points:
                    x0,y0,x1,y1=spec.bounds
                    if not x0-1e-8<=x<=x1+1e-8 or not y0-1e-8<=y<=y1+1e-8:
                        raise ValueError('connector lies outside its controlled junction')
        self.capacity=RoadCapacity(graph)
        self.scheduler=FCFSScheduler(tuple(self.intersections),self.capacity)
        self.aligning_vehicles=set()
        self.pending_departures={}
        self.road_capacity_steps=self.intersection_steps=self.fifo_steps=self.stop_line_steps=0
        self.max_queue_length=0
        self.max_active_tasks=0
        self.max_translating_vehicles=0
        self.fifo_orders={}
        initial_edges=initial_edges or {}
        if any(vehicle not in self.states for vehicle in initial_edges):
            raise ValueError('unknown vehicle in initial road assignments')
        for state in self.states.values():
            candidates=(initial_edges[state.vehicle_id],) if state.vehicle_id in initial_edges else tuple(graph.outgoing[vehicle_nodes[state.vehicle_id]]) or None
            match=match_pose(graph,state.x,state.y,state.heading,candidates)
            if match.projection.distance>1e-6:
                raise ValueError('initial vehicle not on its assigned road')
            state.edge_id=match.edge_id
            state.s=match.projection.s
            state.lateral_offset=match.projection.lateral_offset
            state.heading_error=match.heading_error
        self.capacity.update(self.states)
        if self.capacity.violations:
            raise ValueError('initial road capacity exceeded')
        if any(overlaps(self.box(state),spec.box,config.safety_margin)
               for state in self.states.values() for spec in intersections):
            raise ValueError('initial vehicle inside an unowned junction')

    def _is_driving(self, state: VehicleState) -> bool:
        task_id=self.manager.active.get(state.vehicle_id)
        return bool(task_id and state.vehicle_id not in self.pending_departures
                    and self.manager.records[task_id].status==TaskStatus.EN_ROUTE)

    def _tasks(self, now: float) -> None:
        dispatches=self.manager.dispatch_ready(now)
        for dispatch in dispatches:
            self.pending_departures[dispatch.vehicle_id]=dispatch
        self._start_departures()
        self.max_active_tasks=max(self.max_active_tasks,sum(self._is_driving(state) for state in self.states.values()))

    def _start_departures(self) -> None:
        self.capacity.update(self.states)
        for vehicle_id,dispatch in list(self.pending_departures.items()):
            state=self.states[vehicle_id]
            if not dispatch.route.edges:
                raise ValueError('transport route cannot be empty')
            self.graph.validate_route(dispatch.route.edges)
            first_edge=dispatch.route.edges[0]
            if not self._can_enter(first_edge,vehicle_id) or not self.capacity.reserve(vehicle_id,first_edge):
                continue
            state.route,state.route_index=dispatch.route.edges,0
            state.edge_id=state.route[0]
            state.s=0
            update_route_match(self.graph,state)
            self.aligning_vehicles.add(state.vehicle_id)
            del self.pending_departures[vehicle_id]
            self.capacity.update(self.states)

    def _can_enter(self, edge_id: str, vehicle_id: str) -> bool:
        for occupant in self.capacity.occupants[edge_id]-{vehicle_id}:
            if self.states[occupant].s < self.rules.entry_gap:
                return False
        return True

    def _request_junctions(self) -> None:
        for state in sorted(self.states.values(),key=lambda item:(item.edge_id or '',-item.s,item.vehicle_id)):
            if not self._is_driving(state) or state.route_index+1>=len(state.route):
                continue
            next_edge=state.route[state.route_index+1]
            spec=self.connector_controls.get(next_edge)
            if spec is None or self.graph.edges[state.edge_id].length-state.s > self.rules.request_distance:
                continue
            if state.route_index+2>=len(state.route) or state.route[state.route_index+2]!=spec.connector_exits[next_edge]:
                raise ValueError('controlled crossing requires its configured downstream edge')
            self.scheduler.request(EntranceRequest(state.vehicle_id,spec.intersection_id,state.edge_id,
                                                  next_edge,spec.connector_exits[next_edge],self.now,state.s))
        self.scheduler.advance(self.now,self._can_enter)
        self.max_queue_length=max(self.max_queue_length,max((len(queue) for queue in self.scheduler.queues.values()),default=0))

    def _gap_cap(self, distance: float) -> float:
        distance=max(0,distance)
        b=self.config.vehicle.deceleration
        reaction=1/self.config.control_hz+self.config.dt
        stopping=sqrt((b*reaction)**2+2*b*distance)-b*reaction
        return min(stopping,distance/self.rules.time_headway)

    def _following_cap(self, state: VehicleState) -> float:
        cap=float('inf')
        offset=-state.s
        for edge_id in state.route[state.route_index:]:
            for other in self.states.values():
                if other.vehicle_id==state.vehicle_id or other.edge_id!=edge_id:
                    continue
                distance=offset+other.s
                if distance>1e-8:
                    cap=min(cap,self._gap_cap(distance-self.config.vehicle.length-self.rules.standstill_gap))
            offset+=self.graph.edges[edge_id].length
        return cap

    def _control(self, sequence: int) -> None:
        self._start_departures()
        self.max_active_tasks=max(self.max_active_tasks,sum(self._is_driving(state) for state in self.states.values()))
        self.capacity.update(self.states)
        self._request_junctions()
        for state in self.states.values():
            if not self._is_driving(state):
                self.commands[state.vehicle_id]=ControlCommand(state.vehicle_id,sequence,self.now,0,0)
                continue
            if state.vehicle_id in self.aligning_vehicles:
                heading=self.graph.edges[state.edge_id].centerline.at(0)[1]
                error=wrap_angle(heading-state.heading)
                if abs(error)>.05:
                    omega=max(-self.config.vehicle.omega_max,min(self.config.vehicle.omega_max,3*error))
                    self.commands[state.vehicle_id]=ControlCommand(state.vehicle_id,sequence,self.now,0,omega)
                    continue
                self.aligning_vehicles.remove(state.vehicle_id)
            cap=self._following_cap(state)
            edge=self.graph.edges[state.edge_id]
            if state.route_index+1<len(state.route):
                next_id=state.route[state.route_index+1]
                distance=edge.length-state.s
                if distance<self.rules.request_distance:
                    cap=min(cap,self.graph.edges[next_id].speed_limit)
                    spec=self.connector_controls.get(next_id)
                    if spec:
                        allowed=self.scheduler.allows(state.vehicle_id,spec.intersection_id,next_id)
                    else:
                        allowed=self._can_enter(next_id,state.vehicle_id) and self.capacity.reserve(state.vehicle_id,next_id)
                    if not allowed:
                        cap=min(cap,self._gap_cap(distance-self.rules.stop_offset-.003))
            self.commands[state.vehicle_id]=self.tracker.command(self.graph,state,sequence,cap)

    def _arrivals_and_departures(self, now: float) -> None:
        self.capacity.update(self.states)
        for intersection_id,grant in list(self.scheduler.owners.items()):
            state=self.states[grant.request.vehicle_id]
            inside=overlaps(self.box(state),self.intersections[intersection_id].box,self.config.safety_margin)
            if inside:
                self.scheduler.mark_entered(intersection_id,state.vehicle_id,now)
            elif grant.entered_at is not None:
                self.scheduler.complete(intersection_id,state.vehicle_id,now)
        for vehicle,(_,node_id) in list(self.manager.departures.items()):
            state=self.states[vehicle]
            point=self.graph.nodes[node_id].position
            clearance=self.config.vehicle.length+2*self.config.safety_margin+self.config.arrival_tolerance
            if hypot(state.x-point[0],state.y-point[1])>clearance:
                self.manager.clear_departure(vehicle)
        for state in self.states.values():
            if not self._is_driving(state):
                continue
            record=self.manager.records[self.manager.active[state.vehicle_id]]
            target=self.graph.nodes[record.target_node].position
            if (state.route_index==len(state.route)-1 and state.speed<=self.config.stopped_speed
                    and hypot(state.x-target[0],state.y-target[1])<=self.config.arrival_tolerance):
                self.manager.arrive(state.vehicle_id,record.target_node,now)
                self.commands[state.vehicle_id]=ControlCommand(state.vehicle_id,0,now,0,0)

    def _audit(self, previous: dict[str,VehicleState]) -> None:
        super()._audit(previous)
        self.max_translating_vehicles=max(self.max_translating_vehicles,
                                         sum(state.speed>self.config.stopped_speed for state in self.states.values()))
        self.road_capacity_steps+=self.capacity.violations
        for state in self.states.values():
            for spec in self.intersections.values():
                if overlaps(self.box(state),spec.box,self.config.safety_margin):
                    owner=self.scheduler.owners.get(spec.intersection_id)
                    if owner is None or owner.request.vehicle_id!=state.vehicle_id:
                        self.intersection_steps+=1
            if not self._is_driving(state) or state.route_index+1>=len(state.route):
                continue
            connector=state.route[state.route_index+1]
            spec=self.connector_controls.get(connector)
            if spec and not self.scheduler.allows(state.vehicle_id,spec.intersection_id,connector):
                if state.s>self.graph.edges[state.edge_id].length-self.rules.stop_offset+1e-5:
                    self.stop_line_steps+=1
        # Retain a road queue's order across ticks, including different transitions.
        queues={}
        for state in self.states.values():
            queues.setdefault(state.edge_id,[]).append(state)
        for edge_id,queue in queues.items():
            ordered=[state.vehicle_id for state in sorted(queue,key=lambda item:(item.s,item.vehicle_id))]
            old=self.fifo_orders.get(edge_id,[])
            common=set(old)&set(ordered)
            if [key for key in old if key in common]!=[key for key in ordered if key in common]:
                self.fifo_steps+=1
            self.fifo_orders[edge_id]=ordered

    def _extra_acceptance_checks(self) -> bool:
        return (self.road_capacity_steps==self.intersection_steps==self.fifo_steps==self.stop_line_steps==0
                and not self.scheduler.owners and all(not queue for queue in self.scheduler.queues.values()))

    def _record(self) -> None:
        start=len(self.rows)
        super()._record()
        for row in self.rows[start:]:
            road=self.capacity.road_state(row['edge_id'])
            row['road_current_count']=road.current_vehicle_count
            row['road_committed_count']=road.committed_vehicle_count
            row['owned_intersections']=','.join(key for key,owner in self.scheduler.owners.items()
                                                if owner.request.vehicle_id==row['vehicle_id'])

    def run(self) -> dict:
        result=super().run()
        grants=[event for event in self.scheduler.events if event['event']=='GRANTED']
        delays=[event['time']-event['requested_at'] for event in grants]
        result.update({'road_capacity_violation_edge_steps':self.road_capacity_steps,
                       'illegal_intersection_occupancy_vehicle_steps':self.intersection_steps,
                       'fifo_violation_edge_steps':self.fifo_steps,'stop_line_violation_vehicle_steps':self.stop_line_steps,
                       'junction_grants':len(grants),'junction_clears':sum(event['event']=='CLEARED' for event in self.scheduler.events),
                       'max_simultaneous_intersection_owners':self.scheduler.max_owners,
                       'capacity_blocked_attempts':self.scheduler.capacity_blocked_attempts,
                       'entry_gap_blocked_attempts':self.scheduler.entry_blocked_attempts,
                       'mean_junction_waiting_time_s':sum(delays)/len(delays) if delays else 0,
                       'max_junction_waiting_time_s':max(delays,default=0),'max_junction_queue_length':self.max_queue_length,
                       'max_active_transport_tasks':self.max_active_tasks,
                       'max_translating_vehicles':self.max_translating_vehicles,
                       'throughput_tasks_per_min':result['completed_tasks']/self.now*60 if self.now else 0,
                       'road_capacity_maxima':dict(self.capacity.max_effective)})
        return result

    def write_artifacts(self, output: Path, result: dict) -> None:
        super().write_artifacts(output,result)
        with (output/'traffic_events.jsonl').open('w',encoding='utf-8') as stream:
            for event in self.scheduler.events:
                stream.write(json.dumps(event,sort_keys=True)+'\n')
