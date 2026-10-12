"""Off-road berths with a configurable shared service resource.

Service capacity models workers/loading equipment; a vehicle remains in its
own berth while waiting and being serviced. Moving to a separate loading bay
is not simulated in Phase 3.
"""
from dataclasses import dataclass

from logistics_park.common.messages import finite


@dataclass(frozen=True)
class StationConfig:
    station_id: str
    berth_nodes: tuple[str, ...]
    service_capacity: int = 1
    waiting_capacity: int = 2
    service_time: float = 5.0

    def __post_init__(self) -> None:
        finite(self.service_time)
        if (not self.station_id or type(self.service_capacity) is not int
                or type(self.waiting_capacity) is not int or self.service_capacity < 1
                or self.waiting_capacity < 0 or self.service_time < 0):
            raise ValueError("invalid service station")
        if len(self.berth_nodes) != self.service_capacity+self.waiting_capacity or len(set(self.berth_nodes)) != len(self.berth_nodes):
            raise ValueError("one distinct berth is required for each service/waiting place")


@dataclass
class Admission:
    vehicle_id: str
    berth_node: str
    task_id: str
    arrived_at: float | None = None
    service_started_at: float | None = None
    service_completed: bool = False


class ServiceStation:
    def __init__(self, config: StationConfig):
        self.config = config
        # Includes in-transit commitments and completed vehicles still parked.
        self.occupants: dict[str, Admission] = {}
        self.max_reserved = 0
        self.max_servicing = 0
        self.max_waiting = 0
        self.denied_admissions = 0

    @property
    def available_berths(self) -> tuple[str, ...]:
        used = {item.berth_node for item in self.occupants.values()}
        return tuple(node for node in self.config.berth_nodes if node not in used)

    def reserve(self, vehicle_id: str, task_id: str, berth_node: str | None = None) -> Admission | None:
        if vehicle_id in self.occupants:
            raise ValueError("vehicle already holds a station berth")
        available = self.available_berths
        if berth_node is not None and berth_node not in available:
            raise ValueError("requested berth is unavailable")
        berth = berth_node or next(iter(available), None)
        if berth is None:
            self.denied_admissions += 1
            return None
        admission = Admission(vehicle_id, berth, task_id)
        self.occupants[vehicle_id] = admission
        self.max_reserved = max(self.max_reserved, len(self.occupants))
        return admission

    def park_initial(self, vehicle_id: str, berth_node: str) -> None:
        if berth_node not in self.config.berth_nodes or any(item.berth_node == berth_node for item in self.occupants.values()):
            raise ValueError("initial station berth unavailable")
        self.occupants[vehicle_id] = Admission(vehicle_id, berth_node, "", service_completed=True)
        self.max_reserved = max(self.max_reserved, len(self.occupants))

    def release(self, vehicle_id: str) -> None:
        del self.occupants[vehicle_id]

    def arrive(self, vehicle_id: str, now: float) -> None:
        admission = self.occupants[vehicle_id]
        if admission.arrived_at is not None:
            raise ValueError("duplicate station arrival")
        admission.arrived_at = now

    def advance(self, now: float) -> tuple[list[Admission], list[Admission]]:
        """Complete due services first, then promote waiting arrivals FIFO."""
        completed = []
        for item in self.occupants.values():
            if (item.service_started_at is not None and not item.service_completed
                    and now + 1e-9 >= item.service_started_at+self.config.service_time):
                item.service_completed = True
                completed.append(item)
        servicing = sum(item.service_started_at is not None and not item.service_completed
                        for item in self.occupants.values())
        waiting = sorted((item for item in self.occupants.values()
                          if item.arrived_at is not None and item.service_started_at is None
                          and not item.service_completed), key=lambda item: (item.arrived_at, item.task_id))
        started = waiting[:self.config.service_capacity-servicing]
        for item in started:
            item.service_started_at = now
        self.max_servicing = max(self.max_servicing, servicing+len(started))
        self.max_waiting = max(self.max_waiting, len(waiting)-len(started))
        # Zero-duration stations finish in the same simulation tick.
        if self.config.service_time == 0:
            for item in started:
                item.service_completed = True
                completed.append(item)
        return started, completed
