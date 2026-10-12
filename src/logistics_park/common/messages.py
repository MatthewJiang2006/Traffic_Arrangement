"""Shared SI-unit messages, independent of simulation and hardware adapters."""
from dataclasses import dataclass, field
from math import isfinite

from .enums import Maneuver, ReservationStatus, TaskStatus

Point = tuple[float, float]


def finite(*values: float) -> None:
    if not all(isfinite(value) for value in values):
        raise ValueError("numeric values must be finite")


@dataclass
class VehicleState:
    vehicle_id: str
    x: float
    y: float
    heading: float
    speed: float = 0.0
    omega: float = 0.0
    edge_id: str | None = None
    s: float = 0.0
    lateral_offset: float = 0.0
    heading_error: float = 0.0
    route: tuple[str, ...] = ()
    route_index: int = 0
    timestamp: float = 0.0
    confidence: float = 1.0

    def __post_init__(self) -> None:
        finite(self.x, self.y, self.heading, self.speed, self.omega, self.s,
               self.lateral_offset, self.heading_error, self.timestamp, self.confidence)
        if not self.vehicle_id or not 0 <= self.confidence <= 1:
            raise ValueError("vehicle ID and confidence must be valid")
        if self.s < 0 or self.timestamp < 0 or self.route_index < 0:
            raise ValueError("road progress and time cannot be negative")
        if self.route and self.route_index >= len(self.route):
            raise ValueError("route_index is outside route")


@dataclass(frozen=True)
class ControlCommand:
    vehicle_id: str
    sequence_id: int
    timestamp: float
    v_target: float
    omega_target: float
    ttl: float = 0.15

    def __post_init__(self) -> None:
        finite(self.timestamp, self.v_target, self.omega_target, self.ttl)
        if not self.vehicle_id or self.sequence_id < 0 or self.timestamp < 0 or self.ttl <= 0:
            raise ValueError("invalid command metadata")

    def is_valid_at(self, now: float, last_sequence: int = -1) -> bool:
        return (isfinite(now) and self.timestamp <= now < self.timestamp + self.ttl
                and self.sequence_id > last_sequence)


@dataclass(frozen=True)
class Task:
    task_id: str
    vehicle_id: str
    origin: str
    destination: str
    release_time: float = 0.0
    priority: int = 0
    status: TaskStatus = TaskStatus.PENDING

    def __post_init__(self) -> None:
        finite(self.release_time)
        if not all((self.task_id, self.vehicle_id, self.origin, self.destination)) or self.release_time < 0:
            raise ValueError("invalid task")


@dataclass
class RoadState:
    edge_id: str
    storage_capacity: int
    current_vehicle_count: int = 0
    committed_vehicle_count: int = 0
    queue_length: int = 0
    expected_delay: float = 0.0
    closed: bool = False

    def __post_init__(self) -> None:
        finite(self.expected_delay)
        if not self.edge_id or min(self.storage_capacity, self.current_vehicle_count,
                                   self.committed_vehicle_count, self.queue_length) < 0 or self.expected_delay < 0:
            raise ValueError("invalid road state")

    @property
    def occupancy_ratio(self) -> float:
        return self.current_vehicle_count / self.storage_capacity if self.storage_capacity else float("inf")

    @property
    def predicted_occupancy_ratio(self) -> float:
        return ((self.current_vehicle_count + self.committed_vehicle_count) / self.storage_capacity
                if self.storage_capacity else float("inf"))

    @property
    def downstream_available_capacity(self) -> int:
        return max(0, self.storage_capacity - self.current_vehicle_count - self.committed_vehicle_count)

    @property
    def is_saturated(self) -> bool:
        return self.closed or self.downstream_available_capacity == 0


@dataclass(frozen=True)
class Movement:
    movement_id: str
    incoming_edge: str
    outgoing_edge: str
    maneuver: Maneuver
    reference_path: tuple[Point, ...]
    conflict_zones: tuple[str, ...]
    speed_limit: float

    def __post_init__(self) -> None:
        finite(self.speed_limit, *(v for p in self.reference_path for v in p))
        if not all((self.movement_id, self.incoming_edge, self.outgoing_edge)) or len(self.reference_path) < 2 or self.speed_limit <= 0:
            raise ValueError("invalid movement")


@dataclass(frozen=True)
class ConflictZone:
    zone_id: str
    polygon: tuple[Point, ...]

    def __post_init__(self) -> None:
        finite(*(v for p in self.polygon for v in p))
        area = sum(a[0] * b[1] - b[0] * a[1]
                   for a, b in zip(self.polygon, self.polygon[1:] + self.polygon[:1]))
        if not self.zone_id or len(self.polygon) < 3 or abs(area) < 1e-12:
            raise ValueError("zone must have a nondegenerate polygon")


@dataclass
class Reservation:
    vehicle_id: str
    intersection_id: str
    movement_id: str
    expected_entry_time: float
    entry_window_start: float
    entry_window_end: float
    zone_intervals: dict[str, tuple[float, float]] = field(default_factory=dict)
    status: ReservationStatus = ReservationStatus.REQUESTED

    def __post_init__(self) -> None:
        finite(self.expected_entry_time, self.entry_window_start, self.entry_window_end)
        if not all((self.vehicle_id, self.intersection_id, self.movement_id)):
            raise ValueError("reservation IDs cannot be empty")
        if not 0 <= self.entry_window_start <= self.expected_entry_time <= self.entry_window_end:
            raise ValueError("target must be inside the entry window")
        for zone_id, (start, end) in self.zone_intervals.items():
            finite(start, end)
            if not zone_id or start < 0 or end <= start:
                raise ValueError("zone interval must have positive duration")
