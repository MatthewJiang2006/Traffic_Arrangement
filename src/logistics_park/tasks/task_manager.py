"""Simulation-independent task lifecycle and destination admission."""
from dataclasses import dataclass

from logistics_park.common.enums import TaskStatus
from logistics_park.common.messages import Task, finite
from logistics_park.map.road_graph import RoadGraph
from logistics_park.routing.shortest_path import NoRouteError, Route, shortest_path
from .service_station import ServiceStation, StationConfig


@dataclass
class TaskRecord:
    task: Task
    status: TaskStatus = TaskStatus.PENDING
    route: Route | None = None
    target_node: str | None = None
    dispatched_at: float | None = None
    arrived_at: float | None = None
    service_started_at: float | None = None
    completed_at: float | None = None
    failure_reason: str | None = None


@dataclass(frozen=True)
class Dispatch:
    task_id: str
    vehicle_id: str
    route: Route


class TaskManager:
    def __init__(self, graph: RoadGraph, stations: list[StationConfig],
                 vehicle_nodes: dict[str, str], algorithm: str = "dijkstra"):
        if algorithm not in ("dijkstra", "astar"):
            raise ValueError("unsupported routing algorithm")
        self.graph, self.algorithm = graph, algorithm
        self.stations = {config.station_id: ServiceStation(config) for config in stations}
        if len(self.stations) != len(stations):
            raise ValueError("duplicate stations")
        berths = [node for config in stations for node in config.berth_nodes]
        if len(berths) != len(set(berths)) or any(node not in graph.nodes for node in berths):
            raise ValueError("unknown or shared station berth")
        self.berth_station = {node: config.station_id for config in stations for node in config.berth_nodes}
        self.vehicle_nodes = vehicle_nodes.copy()
        if not vehicle_nodes or any(node not in graph.nodes for node in vehicle_nodes.values()):
            raise ValueError("unknown vehicle start node")
        self.locations = {vehicle: self.berth_station.get(node, node) for vehicle, node in vehicle_nodes.items()}
        for vehicle, node in vehicle_nodes.items():
            if node in self.berth_station:
                self.stations[self.berth_station[node]].park_initial(vehicle, node)
        self.records: dict[str, TaskRecord] = {}
        self.active: dict[str, str] = {}
        self.departures: dict[str, tuple[str, str]] = {}
        self.events: list[dict] = []
        self.now = 0.0
        self.admission_denials = 0

    def submit(self, task: Task) -> None:
        endpoints = (set(self.graph.nodes) - set(self.berth_station)) | set(self.stations)
        if (task.task_id in self.records or task.vehicle_id not in self.vehicle_nodes
                or task.origin not in endpoints or task.destination not in endpoints
                or task.status != TaskStatus.PENDING):
            raise ValueError("invalid or duplicate pending task")
        self.records[task.task_id] = TaskRecord(task)
        self.events.append({"time": self.now, "task_id": task.task_id, "vehicle_id": task.vehicle_id,
                            "status": TaskStatus.PENDING.value})

    def _transition(self, record: TaskRecord, status: TaskStatus, now: float, **details) -> None:
        record.status = status
        self.events.append({"time": now, "task_id": record.task.task_id,
                            "vehicle_id": record.task.vehicle_id, "status": status.value, **details})

    def _complete(self, record: TaskRecord, now: float) -> None:
        record.completed_at = now
        vehicle_id = record.task.vehicle_id
        self.vehicle_nodes[vehicle_id] = record.target_node
        self.locations[vehicle_id] = record.task.destination
        self.active.pop(vehicle_id)
        self._transition(record, TaskStatus.TASK_COMPLETE, now)

    def advance(self, now: float) -> None:
        finite(now)
        if now < self.now-1e-9:
            raise ValueError("task time must be monotonic")
        self.now = now
        for station in self.stations.values():
            started, completed = station.advance(now)
            for item in started:
                record = self.records[item.task_id]
                record.service_started_at = now
                self._transition(record, TaskStatus.SERVICING, now)
            for item in completed:
                self._complete(self.records[item.task_id], now)

    def dispatch_ready(self, now: float, max_dispatches: int | None = None) -> list[Dispatch]:
        self.advance(now)
        if max_dispatches is not None and max_dispatches < 0:
            raise ValueError("dispatch limit cannot be negative")
        heads = {}
        for record in self.records.values():
            if record.status == TaskStatus.PENDING:
                heads.setdefault(record.task.vehicle_id, record)
        pending = sorted(heads.values(),
                         key=lambda record: (record.task.release_time, -record.task.priority, record.task.task_id))
        dispatches = []
        # Submission order defines each vehicle's itinerary.
        for record in pending:
            task = record.task
            if (task.release_time > now+1e-9 or task.vehicle_id in self.active
                    or (max_dispatches is not None and len(dispatches) >= max_dispatches)):
                continue
            if task.origin != self.locations[task.vehicle_id]:
                record.failure_reason = "task origin does not match vehicle location"
                self._transition(record, TaskStatus.FAILED, now, reason=record.failure_reason)
                continue
            if task.origin == task.destination:
                record.failure_reason = "transport task requires distinct endpoints"
                self._transition(record, TaskStatus.FAILED, now, reason=record.failure_reason)
                continue
            station = self.stations.get(task.destination)
            if task.destination in self.berth_station and station is None:
                raise ValueError("tasks must address the station, not a private berth")
            if station and not station.available_berths:
                station.reserve(task.vehicle_id, task.task_id)  # Count the denied attempt.
                self.admission_denials += 1
                continue
            targets = station.available_berths if station else (task.destination,)
            route = None
            for target in targets:
                try:
                    route = shortest_path(self.graph, self.vehicle_nodes[task.vehicle_id], target, self.algorithm)
                    break
                except NoRouteError:
                    continue
            if route is None:
                record.failure_reason = f"no legal route to available destination berth: {task.destination}"
                self._transition(record, TaskStatus.FAILED, now, reason=record.failure_reason)
                continue
            if station:
                station.reserve(task.vehicle_id, task.task_id, target)
            record.route, record.target_node, record.dispatched_at = route, target, now
            origin_station = self.berth_station.get(self.vehicle_nodes[task.vehicle_id])
            if origin_station:
                self.departures[task.vehicle_id] = (origin_station, self.vehicle_nodes[task.vehicle_id])
            self.active[task.vehicle_id] = task.task_id
            self._transition(record, TaskStatus.EN_ROUTE, now, route=list(route.edges), target_node=target)
            dispatches.append(Dispatch(task.task_id, task.vehicle_id, route))
        return dispatches

    def clear_departure(self, vehicle_id: str) -> None:
        """Release origin berth only after the physical vehicle has cleared it."""
        station_id, _ = self.departures.pop(vehicle_id)
        self.stations[station_id].release(vehicle_id)

    def arrive(self, vehicle_id: str, node_id: str, now: float) -> None:
        self.advance(now)
        record = self.records[self.active[vehicle_id]]
        if record.status != TaskStatus.EN_ROUTE or node_id != record.target_node:
            raise ValueError("arrival does not match dispatched target")
        if vehicle_id in self.departures:
            raise ValueError("vehicle has not cleared its origin berth")
        record.arrived_at = now
        self.vehicle_nodes[vehicle_id] = node_id
        self._transition(record, TaskStatus.ARRIVED, now)
        station = self.stations.get(record.task.destination)
        if station:
            station.arrive(vehicle_id, now)
            self.advance(now)
        else:
            record.service_started_at = now
            self._transition(record, TaskStatus.SERVICING, now)
            self._complete(record, now)

    @property
    def all_complete(self) -> bool:
        return bool(self.records) and all(record.status == TaskStatus.TASK_COMPLETE for record in self.records.values())
