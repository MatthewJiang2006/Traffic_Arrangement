"""Phase 3 transport execution, with one moving vehicle before Phase 4.

Vehicles in off-road berths remain in the collision audit while idle, queued,
servicing or complete. Service uses a logical resource, without teleportation.
"""
import csv
import json
import logging
from dataclasses import asdict, dataclass, field
from itertools import combinations
from math import hypot
from pathlib import Path

from logistics_park.common.enums import TaskStatus
from logistics_park.common.messages import ControlCommand, Task, VehicleState, finite
from logistics_park.common.time_utils import PeriodicTimer
from logistics_park.map.geometry import wrap_angle
from logistics_park.map.map_matching import update_route_match
from logistics_park.map.road_graph import RoadGraph
from logistics_park.navigation.pure_pursuit import PurePursuit, TrackingConfig
from logistics_park.safety.collision import OBB, overlaps
from logistics_park.tasks.service_station import StationConfig
from logistics_park.tasks.task_manager import TaskManager
from .kinematics import VehicleLimits, step

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class TaskSimulationConfig:
    dt: float = 0.02
    duration: float = 300.0
    control_hz: float = 20.0
    task_hz: float = 5.0
    logging_hz: float = 20.0
    max_lateral_error: float = 0.06
    arrival_tolerance: float = 0.012
    stopped_speed: float = 0.005
    safety_margin: float = 0.015
    algorithm: str = "dijkstra"
    vehicle: VehicleLimits = field(default_factory=VehicleLimits)
    tracking: TrackingConfig = field(default_factory=TrackingConfig)

    def __post_init__(self) -> None:
        finite(self.dt, self.duration, self.control_hz, self.task_hz, self.logging_hz,
               self.max_lateral_error, self.arrival_tolerance, self.stopped_speed, self.safety_margin)
        if min(self.dt, self.duration, self.control_hz, self.task_hz, self.logging_hz,
               self.max_lateral_error, self.arrival_tolerance, self.stopped_speed) <= 0 or self.safety_margin < 0:
            raise ValueError("invalid task simulation configuration")
        if max(self.control_hz, self.task_hz, self.logging_hz)*self.dt > 1+1e-9:
            raise ValueError("module rate exceeds simulation rate")
        if abs(self.duration/self.dt-round(self.duration/self.dt)) > 1e-8 or 1/self.control_hz+self.dt >= 0.15:
            raise ValueError("duration/command TTL incompatible with simulation step")
        if self.algorithm not in ("dijkstra", "astar"):
            raise ValueError("invalid routing algorithm")

    @classmethod
    def load(cls, path: str | Path):
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        data["vehicle"] = VehicleLimits(**data.get("vehicle", {}))
        data["tracking"] = TrackingConfig(**data.get("tracking", {}))
        return cls(**data)


class TaskSimulator:
    result_prefix = "PHASE_3_TASK_CLOSED_LOOP"
    execution_policy = "single_vehicle_road_execution"

    def _is_driving(self, state: VehicleState) -> bool:
        return state.vehicle_id == self.moving

    def _extra_acceptance_checks(self) -> bool:
        return True

    def __init__(self, graph: RoadGraph, config: TaskSimulationConfig,
                 stations: list[StationConfig], vehicle_nodes: dict[str, str], tasks: list[Task]):
        self.graph, self.config = graph, config
        if not tasks:
            raise ValueError("acceptance requires at least one task")
        if len(set(vehicle_nodes.values())) != len(vehicle_nodes):
            raise ValueError("vehicles require distinct initial parking nodes")
        self.manager = TaskManager(graph, stations, vehicle_nodes, config.algorithm)
        for task in tasks:
            self.manager.submit(task)
        self.states = {vehicle: VehicleState(vehicle, *graph.nodes[node].position, 0)
                       for vehicle,node in vehicle_nodes.items()}
        self.tracker = PurePursuit(config.tracking)
        self.moving: str | None = None
        self.aligning = False
        self.commands: dict[str, ControlCommand] = {}
        self.rows: list[dict] = []
        self.collision_steps = self.envelope_steps = self.limit_steps = self.capacity_steps = 0
        self.max_error = self.max_acceleration = 0.0
        self.max_moving_vehicles = 0
        self.now = 0.0
        self._ran = False
        self._validate_berths(stations)
        if any(overlaps(self.box(a),self.box(b),config.safety_margin)
               for a,b in combinations(self.states.values(),2)):
            raise ValueError("unsafe initial vehicle placement")

    def box(self, state: VehicleState) -> OBB:
        cfg = self.config.vehicle
        return OBB(state.x,state.y,state.heading,cfg.length,cfg.width)

    def _validate_berths(self, stations: list[StationConfig]) -> None:
        # Allow any parking heading: enclosing radii guarantee berth separation.
        radius = hypot(self.config.vehicle.length/2 + self.config.safety_margin,
                       self.config.vehicle.width/2 + self.config.safety_margin)
        nodes = [node for station in stations for node in station.berth_nodes]
        for a,b in combinations(nodes,2):
            pa,pb = self.graph.nodes[a].position,self.graph.nodes[b].position
            if hypot(pa[0]-pb[0],pa[1]-pb[1]) <= 2*(radius+self.config.arrival_tolerance):
                raise ValueError("station berths are too close")

    def _tasks(self, now: float) -> None:
        self.manager.advance(now)
        if self.moving is not None:
            return
        dispatches = self.manager.dispatch_ready(now,max_dispatches=1)
        if not dispatches:
            return
        dispatch = dispatches[0]
        state = self.states[dispatch.vehicle_id]
        state.route, state.route_index = dispatch.route.edges, 0
        if not state.route:
            raise ValueError("transport route cannot be empty")
        self.graph.validate_route(state.route)
        state.edge_id = state.route[0]
        state.s = 0.0
        update_route_match(self.graph,state)
        self.moving = dispatch.vehicle_id
        self.aligning = True

    def _control(self, sequence: int) -> None:
        cfg = self.config
        for state in self.states.values():
            if state.vehicle_id != self.moving:
                self.commands[state.vehicle_id] = ControlCommand(state.vehicle_id,sequence,self.now,0,0)
                continue
            if self.aligning:
                heading = self.graph.edges[state.edge_id].centerline.at(0)[1]
                error = wrap_angle(heading-state.heading)
                if abs(error) > 0.05:
                    omega = max(-cfg.vehicle.omega_max,min(cfg.vehicle.omega_max,3*error))
                    self.commands[state.vehicle_id] = ControlCommand(state.vehicle_id,sequence,self.now,0,omega)
                    continue
                self.aligning = False
            cap = cfg.vehicle.v_max
            current = self.graph.edges[state.edge_id]
            if state.route_index+1 < len(state.route) and current.length-state.s < 0.15:
                cap = min(cap,self.graph.edges[state.route[state.route_index+1]].speed_limit)
            self.commands[state.vehicle_id] = self.tracker.command(self.graph,state,sequence,cap)

    def _arrivals_and_departures(self, now: float) -> None:
        for vehicle,(station_id,node_id) in list(self.manager.departures.items()):
            state = self.states[vehicle]
            point = self.graph.nodes[node_id].position
            clearance = self.config.vehicle.length + 2*self.config.safety_margin + self.config.arrival_tolerance
            if hypot(state.x-point[0],state.y-point[1]) > clearance:
                self.manager.clear_departure(vehicle)
        if self.moving is None:
            return
        state = self.states[self.moving]
        record = self.manager.records[self.manager.active[self.moving]]
        target = self.graph.nodes[record.target_node].position
        if (state.route_index == len(state.route)-1 and state.speed <= self.config.stopped_speed
                and hypot(state.x-target[0],state.y-target[1]) <= self.config.arrival_tolerance):
            self.manager.arrive(state.vehicle_id,record.target_node,now)
            self.moving = None
            self.commands[state.vehicle_id] = ControlCommand(state.vehicle_id,0,now,0,0)

    def _audit(self, previous: dict[str,VehicleState]) -> None:
        cfg = self.config
        moving_count = sum(state.speed > cfg.stopped_speed or abs(state.omega) > cfg.stopped_speed
                           for state in self.states.values())
        self.max_moving_vehicles = max(self.max_moving_vehicles, moving_count)
        for state in self.states.values():
            acceleration = (state.speed-previous[state.vehicle_id].speed)/cfg.dt
            self.max_acceleration = max(self.max_acceleration,abs(acceleration))
            if state.edge_id:
                self.max_error = max(self.max_error,abs(state.lateral_offset))
                if state.speed > self.graph.edges[state.edge_id].speed_limit+1e-8:
                    self.limit_steps += 1
            if (state.speed > cfg.vehicle.v_max+1e-8 or abs(state.omega) > cfg.vehicle.omega_max+1e-8
                    or acceleration > cfg.vehicle.a_max+1e-8 or acceleration < -cfg.vehicle.deceleration-1e-8):
                self.limit_steps += 1
        for a,b in combinations(self.states.values(),2):
            self.collision_steps += int(overlaps(self.box(a),self.box(b)))
            self.envelope_steps += int(overlaps(self.box(a),self.box(b),cfg.safety_margin))
        for station in self.manager.stations.values():
            servicing = sum(item.service_started_at is not None and not item.service_completed for item in station.occupants.values())
            waiting = sum(item.arrived_at is not None and item.service_started_at is None and not item.service_completed for item in station.occupants.values())
            if (len(station.occupants) > len(station.config.berth_nodes)
                    or servicing > station.config.service_capacity or waiting > station.config.waiting_capacity):
                self.capacity_steps += 1

    def _record(self) -> None:
        for state in self.states.values():
            row = asdict(state)
            row["timestamp"] = self.now
            task_id = self.manager.active.get(state.vehicle_id)
            row["task_id"] = task_id or ""
            row["task_status"] = self.manager.records[task_id].status.value if task_id else "IDLE"
            self.rows.append(row)

    def run(self) -> dict:
        if self._ran:
            raise RuntimeError("create a new simulator for each run")
        self._ran = True
        cfg = self.config
        control = PeriodicTimer(1/cfg.control_hz)
        tasks = PeriodicTimer(1/cfg.task_hz)
        logging_timer = PeriodicTimer(1/cfg.logging_hz)
        sequence = 0
        steps = round(cfg.duration/cfg.dt)
        for tick in range(steps+1):
            self.now = tick*cfg.dt
            if tasks.ready(self.now):
                self._tasks(self.now)
            if logging_timer.ready(self.now):
                self._record()
            if self.manager.all_complete or tick == steps:
                break
            if control.ready(self.now):
                self._control(sequence)
                sequence += 1
            previous = self.states.copy()
            for vehicle,state in previous.items():
                updated = step(state,self.commands[vehicle],cfg.dt,cfg.vehicle)
                updated.timestamp = (tick+1)*cfg.dt
                if updated.route and self._is_driving(updated):
                    update_route_match(self.graph,updated)
                self.states[vehicle] = updated
            self._arrivals_and_departures((tick+1)*cfg.dt)
            self._audit(previous)
        if self.rows[-1]["timestamp"] != self.now:
            self._record()
        completed = sum(record.status == TaskStatus.TASK_COMPLETE for record in self.manager.records.values())
        failed = sum(record.status == TaskStatus.FAILED for record in self.manager.records.values())
        passed = (self.manager.all_complete and self.collision_steps == 0 and self.envelope_steps == 0
                  and self.limit_steps == 0 and self.capacity_steps == 0 and self.max_error <= cfg.max_lateral_error
                  and self._extra_acceptance_checks())
        result = {
            "status": self.result_prefix + ("_PASS" if passed else "_FAIL"),
            "algorithm": cfg.algorithm,"vehicle_count": len(self.states),"task_count":len(self.manager.records),
            "completed_tasks":completed,"failed_tasks":failed,"unfinished_tasks":len(self.manager.records)-completed-failed,
            "simulation_time":self.now,"collision_pair_steps":self.collision_steps,
            "safety_envelope_violation_pair_steps":self.envelope_steps,"limit_violation_vehicle_steps":self.limit_steps,
            "capacity_violation_station_steps":self.capacity_steps,"max_lateral_error_m":self.max_error,
            "max_abs_acceleration_m_s2":self.max_acceleration,"admission_denial_attempts":self.manager.admission_denials,
            "max_moving_vehicles":self.max_moving_vehicles,"execution_policy":self.execution_policy,"stations":{
                name:{"max_reserved":station.max_reserved,"max_servicing":station.max_servicing,
                      "max_waiting":station.max_waiting,"service_capacity":station.config.service_capacity,
                      "waiting_capacity":station.config.waiting_capacity}
                for name,station in self.manager.stations.items()},
        }
        logger.info("Transport acceptance %s",json.dumps(result,sort_keys=True))
        return result

    def write_artifacts(self, output: Path, result: dict) -> None:
        output.mkdir(parents=True,exist_ok=True)
        (output/"summary.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
        (output/"effective_config.json").write_text(json.dumps(asdict(self.config),indent=2)+"\n",encoding="utf-8")
        with (output/"trajectory.csv").open("w",newline="",encoding="utf-8") as stream:
            writer = csv.DictWriter(stream,fieldnames=list(self.rows[0]))
            writer.writeheader()
            writer.writerows(self.rows)
        with (output/"tasks.jsonl").open("w",encoding="utf-8") as stream:
            for record in self.manager.records.values():
                stream.write(json.dumps(asdict(record),sort_keys=True)+"\n")
        with (output/"events.jsonl").open("w",encoding="utf-8") as stream:
            for event in self.manager.events:
                stream.write(json.dumps(event,sort_keys=True)+"\n")
