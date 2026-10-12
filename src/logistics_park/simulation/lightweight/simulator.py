"""Ideal-state minimal closed loop. No task or intersection scheduler yet."""
import csv
import json
import logging
from dataclasses import asdict, dataclass, field
from itertools import combinations
from math import isfinite
from pathlib import Path

from logistics_park.common.messages import ControlCommand, VehicleState, finite
from logistics_park.common.time_utils import PeriodicTimer
from logistics_park.map.map_matching import update_route_match
from logistics_park.map.road_graph import RoadGraph
from logistics_park.navigation.pure_pursuit import PurePursuit, TrackingConfig
from logistics_park.safety.collision import OBB, overlaps
from logistics_park.safety.following_guard import following_speed_cap
from .kinematics import VehicleLimits, step

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class SimulationConfig:
    dt: float
    duration: float
    control_hz: float
    logging_hz: float
    max_lateral_error: float
    safety_margin: float
    minimum_laps: int
    spawn_spacing: float
    route: tuple[str, ...]
    vehicle: VehicleLimits = field(default_factory=VehicleLimits)
    tracking: TrackingConfig = field(default_factory=TrackingConfig)

    def __post_init__(self) -> None:
        finite(self.dt, self.duration, self.control_hz, self.logging_hz,
               self.max_lateral_error, self.safety_margin, self.spawn_spacing)
        if min(self.dt, self.duration, self.control_hz, self.logging_hz,
               self.max_lateral_error, self.spawn_spacing) <= 0 or self.safety_margin < 0 or self.minimum_laps < 1:
            raise ValueError("invalid simulation configuration")
        if max(self.control_hz, self.logging_hz)*self.dt > 1 + 1e-9:
            raise ValueError("module rate cannot exceed simulation rate")
        if abs(self.duration/self.dt-round(self.duration/self.dt)) > 1e-8:
            raise ValueError("duration must be a multiple of dt")

    @classmethod
    def load(cls, path: str | Path):
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        data["route"] = tuple(data["route"])
        data["vehicle"] = VehicleLimits(**data["vehicle"])
        data["tracking"] = TrackingConfig(**data["tracking"])
        return cls(**data)


class Simulator:
    def __init__(self, graph: RoadGraph, config: SimulationConfig, vehicle_count: int):
        if not 1 <= vehicle_count <= 4:
            raise ValueError("minimal acceptance supports 1 to 4 vehicles")
        graph.validate_route(config.route, cyclic=True)
        if 1/config.control_hz + config.dt >= 0.15:
            raise ValueError("control period exceeds command TTL")
        self.graph, self.config = graph, config
        self.route_length = sum(graph.edges[e].length for e in config.route)
        self.states: dict[str, VehicleState] = {}
        self.initial_progress: dict[str, float] = {}
        self.wraps: dict[str, int] = {}
        if config.spawn_spacing*(vehicle_count-1) >= self.route_length:
            raise ValueError("spawn spacing exceeds route")
        for number in range(vehicle_count):
            progress = number*config.spawn_spacing
            remaining, index = progress, 0
            while remaining >= graph.edges[config.route[index]].length:
                remaining -= graph.edges[config.route[index]].length
                index += 1
            edge = graph.edges[config.route[index]]
            point, heading = edge.centerline.at(remaining)
            vehicle_id = f"car_{number:02d}"
            state = VehicleState(vehicle_id, *point, heading, edge_id=edge.edge_id, s=remaining,
                                 route=config.route, route_index=index)
            self.states[vehicle_id] = state
            self.initial_progress[vehicle_id] = progress
            self.wraps[vehicle_id] = 0
        self.commands: dict[str, ControlCommand] = {}
        self.tracker = PurePursuit(config.tracking)
        self.rows: list[dict] = []
        self.collision_steps = self.envelope_steps = self.fifo_steps = 0
        self.limit_steps = 0
        self.max_error = self.max_acceleration = 0.0
        self.control_ticks = 0
        self._ran = False
        if any(overlaps(self.box(a), self.box(b), config.safety_margin)
               for a, b in combinations(self.states.values(), 2)):
            raise ValueError("unsafe initial spawn")

    def box(self, state: VehicleState) -> OBB:
        return OBB(state.x, state.y, state.heading, self.config.vehicle.length, self.config.vehicle.width)

    def progress(self, state: VehicleState) -> float:
        return (self.wraps[state.vehicle_id]*self.route_length
                + sum(self.graph.edges[e].length for e in state.route[:state.route_index]) + state.s)

    def _control(self, sequence: int) -> None:
        queues: dict[str, list[VehicleState]] = {}
        for state in self.states.values():
            queues.setdefault(state.edge_id, []).append(state)
        for queue in queues.values():
            queue.sort(key=lambda state: state.s, reverse=True)
            for i, state in enumerate(queue):
                leader = queue[i-1] if i else None
                cap = following_speed_cap(state, leader, self.config.vehicle.length,
                                          deceleration=self.config.vehicle.deceleration,
                                          reaction_time=1/self.config.control_hz+self.config.dt)
                # Respect upcoming edge limits before crossing the edge boundary.
                next_edge = self.graph.edges[state.route[(state.route_index+1) % len(state.route)]]
                edge = self.graph.edges[state.edge_id]
                if edge.length-state.s < 0.15:
                    cap = min(cap, next_edge.speed_limit)
                self.commands[state.vehicle_id] = self.tracker.command(self.graph, state, sequence, cap, cyclic=True)

    def _audit(self, previous: dict[str, VehicleState]) -> None:
        limits = self.config.vehicle
        for state in self.states.values():
            self.max_error = max(self.max_error, abs(state.lateral_offset))
            acceleration = (state.speed-previous[state.vehicle_id].speed)/self.config.dt
            self.max_acceleration = max(self.max_acceleration, abs(acceleration))
            if state.speed > self.graph.edges[state.edge_id].speed_limit+1e-8 or abs(state.omega) > limits.omega_max+1e-8:
                self.limit_steps += 1
            if acceleration > limits.a_max+1e-8 or acceleration < -limits.deceleration-1e-8:
                self.limit_steps += 1
        for a, b in combinations(self.states.values(), 2):
            if overlaps(self.box(a), self.box(b)):
                self.collision_steps += 1
            if overlaps(self.box(a), self.box(b), self.config.safety_margin):
                self.envelope_steps += 1
            old_a, old_b = previous[a.vehicle_id], previous[b.vehicle_id]
            if a.edge_id == b.edge_id == old_a.edge_id == old_b.edge_id and (a.s-b.s)*(old_a.s-old_b.s) < 0:
                self.fifo_steps += 1

    def run(self) -> dict:
        if self._ran:
            raise RuntimeError("create a new simulator for each run")
        self._ran = True
        cfg = self.config
        control_timer = PeriodicTimer(1/cfg.control_hz)
        log_timer = PeriodicTimer(1/cfg.logging_hz)
        steps = round(cfg.duration/cfg.dt)
        for tick in range(steps+1):
            now = tick*cfg.dt
            if log_timer.ready(now) or tick == steps:
                for state in self.states.values():
                    row = asdict(state)
                    row["timestamp"] = now
                    row["progress"] = self.progress(state)
                    self.rows.append(row)
            if tick == steps:
                break
            if control_timer.ready(now):
                self._control(self.control_ticks)
                self.control_ticks += 1
            previous = self.states.copy()
            for vehicle_id, state in previous.items():
                updated = step(state, self.commands[vehicle_id], cfg.dt, cfg.vehicle)
                updated.timestamp = (tick+1)*cfg.dt
                if update_route_match(self.graph, updated, cyclic=True):
                    self.wraps[vehicle_id] += 1
                self.states[vehicle_id] = updated
            self._audit(previous)
        laps = {key: (self.progress(state)-self.initial_progress[key])/self.route_length
                for key, state in self.states.items()}
        passed = (all(isfinite(value) and value >= cfg.minimum_laps for value in laps.values())
                  and self.max_error <= cfg.max_lateral_error and self.collision_steps == 0
                  and self.envelope_steps == 0 and self.fifo_steps == 0 and self.limit_steps == 0)
        result = {
            "status": "LIGHTWEIGHT_SIM_MINIMAL_CLOSED_LOOP_PASS" if passed else "LIGHTWEIGHT_SIM_MINIMAL_CLOSED_LOOP_FAIL",
            "vehicle_count": len(self.states), "simulation_time": cfg.duration,
            "dt": cfg.dt, "control_ticks": self.control_ticks,
            "collision_pair_steps": self.collision_steps,
            "safety_envelope_violation_pair_steps": self.envelope_steps,
            "fifo_violation_pair_steps": self.fifo_steps,
            "limit_violation_vehicle_steps": self.limit_steps,
            "max_lateral_error_m": self.max_error, "max_abs_acceleration_m_s2": self.max_acceleration,
            "completed_laps": laps,
        }
        logger.info("Acceptance %s", json.dumps(result, sort_keys=True))
        return result

    def write_artifacts(self, output: Path, result: dict) -> None:
        output.mkdir(parents=True, exist_ok=True)
        (output/"summary.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
        (output/"effective_config.json").write_text(json.dumps(asdict(self.config), indent=2)+"\n", encoding="utf-8")
        with (output/"trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(self.rows[0]))
            writer.writeheader()
            writer.writerows(self.rows)
