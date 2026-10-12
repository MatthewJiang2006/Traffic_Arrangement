"""Dynamic fleet contract for gate admission, task dispatch and departure.

Gate reservations and physical boundary observations are supplied by the
scheduler/backend. This registry neither moves vehicles nor frees road leases.
"""
from dataclasses import dataclass, replace
from enum import Enum
from typing import Protocol
from uuid import uuid4

from logistics_park.common.messages import finite


class FleetPhase(str, Enum):
    OUTSIDE = 'OUTSIDE'
    ENTRY_REQUESTED = 'ENTRY_REQUESTED'
    ENTERING = 'ENTERING'
    IDLE = 'IDLE'
    ON_TASK = 'ON_TASK'
    DRAINING = 'DRAINING'
    EXIT_REQUESTED = 'EXIT_REQUESTED'
    EXITING = 'EXITING'
    DEPARTED = 'DEPARTED'


class GateDirection(str, Enum):
    ENTRY = 'ENTRY'
    EXIT = 'EXIT'


@dataclass(frozen=True)
class GatePermit:
    permit_id: str
    vehicle_id: str
    visit_id: str
    gate_id: str
    direction: GateDirection
    valid_after: float
    valid_until: float

    def __post_init__(self):
        finite(self.valid_after, self.valid_until)
        if (not all((self.permit_id, self.vehicle_id, self.visit_id, self.gate_id))
                or not isinstance(self.direction, GateDirection)
                or not 0 <= self.valid_after < self.valid_until):
            raise ValueError('Invalid gate permit')


@dataclass(frozen=True)
class FleetVehicle:
    vehicle_id: str
    visit_id: str
    phase: FleetPhase
    updated_at: float
    entry_gate: str | None = None
    exit_gate: str | None = None
    task_id: str | None = None
    owned_resources: frozenset[str] = frozenset()
    resources_observed_at: float | None = None


class DynamicFleetBackend(Protocol):
    """Backend hooks to implement later for lightweight, UE and hardware.

    Attach observes/registers an existing vehicle or creates a test actor through
    the backend's own policy. Detach happens after confirmed physical departure;
    it stops command delivery and observation, not necessarily actor destruction.
    """
    def attach_vehicle(self, vehicle_id: str, visit_id: str) -> None: ...
    def detach_vehicle(self, vehicle_id: str, visit_id: str) -> None: ...


class FleetRegistry:
    def __init__(self):
        self.vehicles: dict[str, FleetVehicle] = {}
        self.events: list[dict] = []
        self.visits: dict[str, FleetVehicle] = {}

    @property
    def available_vehicle_ids(self):
        return tuple(sorted(v.vehicle_id for v in self.vehicles.values() if v.phase == FleetPhase.IDLE))

    @property
    def park_vehicle_ids(self):
        phases = {FleetPhase.ENTERING, FleetPhase.IDLE, FleetPhase.ON_TASK, FleetPhase.DRAINING,
                  FleetPhase.EXIT_REQUESTED, FleetPhase.EXITING}
        return tuple(sorted(v.vehicle_id for v in self.vehicles.values() if v.phase in phases))

    def _read(self, vehicle_id, visit_id, now, phases=None):
        finite(now)
        row = self.vehicles[vehicle_id]
        if row.visit_id != visit_id:
            raise ValueError('Stale vehicle visit')
        if now < row.updated_at or now < 0:
            raise ValueError('Out-of-order fleet event')
        if phases is not None and row.phase not in phases:
            raise ValueError('Invalid fleet transition from ' + row.phase.value)
        return row

    def _update(self, row, now, event, **changes):
        row = replace(row, updated_at=now, **changes)
        self.vehicles[row.vehicle_id] = row
        self.visits[row.visit_id] = row
        self.events.append(dict(event=event, time=now, vehicle_id=row.vehicle_id,
                                visit_id=row.visit_id, phase=row.phase.value,
                                task_id=row.task_id, entry_gate=row.entry_gate, exit_gate=row.exit_gate))
        return row

    def register_vehicle(self, vehicle_id: str, now: float):
        finite(now)
        if not vehicle_id or now < 0:
            raise ValueError('Invalid vehicle registration')
        old = self.vehicles.get(vehicle_id)
        if old and (old.phase != FleetPhase.DEPARTED or now < old.updated_at):
            raise ValueError('Vehicle already registered or registration is out of order')
        row = FleetVehicle(vehicle_id, uuid4().hex, FleetPhase.OUTSIDE, now)
        return self._update(row, now, 'VEHICLE_REGISTERED')

    def request_entry(self, vehicle_id, visit_id, gate_id, now):
        row = self._read(vehicle_id, visit_id, now, {FleetPhase.OUTSIDE})
        if not gate_id:raise ValueError('Entry gate required')
        return self._update(row, now, 'ENTRY_REQUESTED', phase=FleetPhase.ENTRY_REQUESTED, entry_gate=gate_id)

    def adopt_present_vehicle(self, vehicle_id, now, *, fully_inside, resources):
        """Bootstrap a vehicle already in the park, without forging a gate trip."""
        if not fully_inside:raise ValueError('Existing vehicle must be observed fully inside')
        if isinstance(resources, str):raise ValueError('Expected a collection of resource IDs')
        resources = frozenset(resources)
        if any(not isinstance(r,str) or not r for r in resources):raise ValueError('Invalid resource IDs')
        row = self.register_vehicle(vehicle_id, now)
        return self._update(row, now, 'PRESENT_VEHICLE_ADOPTED', phase=FleetPhase.IDLE,
                            owned_resources=resources, resources_observed_at=now)

    def _authorize(self, permit, now, phase, direction):
        row = self._read(permit.vehicle_id, permit.visit_id, now, {phase})
        gate = row.entry_gate if direction == GateDirection.ENTRY else row.exit_gate
        if permit.direction != direction or permit.gate_id != gate or not permit.valid_after <= now < permit.valid_until:
            raise ValueError('Gate permit does not match vehicle, gate, direction or time')
        return row

    def authorize_entry(self, permit: GatePermit, now):
        row = self._authorize(permit, now, FleetPhase.ENTRY_REQUESTED, GateDirection.ENTRY)
        return self._update(row, now, 'ENTRY_AUTHORIZED', phase=FleetPhase.ENTERING)

    def confirm_entry(self, vehicle_id, visit_id, now, *, fully_inside):
        row = self._read(vehicle_id, visit_id, now, {FleetPhase.ENTERING})
        if not fully_inside:raise ValueError('Actual full-body entry must be confirmed')
        return self._update(row, now, 'VEHICLE_ENTERED', phase=FleetPhase.IDLE)

    def assign_task(self, vehicle_id, visit_id, task_id, now):
        row = self._read(vehicle_id, visit_id, now, {FleetPhase.IDLE})
        if not task_id or any(v.task_id == task_id for v in self.vehicles.values()):
            raise ValueError('Task ID required and cannot be assigned twice')
        return self._update(row, now, 'TASK_ASSIGNED', phase=FleetPhase.ON_TASK, task_id=task_id)

    def request_exit(self, vehicle_id, visit_id, gate_id, now):
        row = self._read(vehicle_id, visit_id, now, {FleetPhase.IDLE, FleetPhase.ON_TASK})
        if not gate_id:raise ValueError('Exit gate required')
        phase = FleetPhase.DRAINING if row.task_id else FleetPhase.EXIT_REQUESTED
        return self._update(row, now, 'EXIT_REQUESTED', phase=phase, exit_gate=gate_id)

    def finish_task(self, vehicle_id, visit_id, task_id, now):
        row = self._read(vehicle_id, visit_id, now, {FleetPhase.ON_TASK, FleetPhase.DRAINING})
        if row.task_id != task_id:raise ValueError('Wrong task completion')
        phase = FleetPhase.EXIT_REQUESTED if row.exit_gate else FleetPhase.IDLE
        return self._update(row, now, 'TASK_COMPLETED', phase=phase, task_id=None)

    def authorize_exit(self, permit: GatePermit, now):
        row = self._authorize(permit, now, FleetPhase.EXIT_REQUESTED, GateDirection.EXIT)
        return self._update(row, now, 'EXIT_AUTHORIZED', phase=FleetPhase.EXITING, resources_observed_at=None)

    def set_owned_resources(self, vehicle_id, visit_id, resources, now):
        row = self._read(vehicle_id, visit_id, now)
        if isinstance(resources, str):raise ValueError('Expected a collection of resource IDs')
        resources = frozenset(resources)
        if row.phase in {FleetPhase.OUTSIDE, FleetPhase.DEPARTED} or any(not isinstance(r,str) or not r for r in resources):
            raise ValueError('Invalid resource ownership snapshot')
        return self._update(row, now, 'RESOURCE_OWNERSHIP_UPDATED', owned_resources=resources, resources_observed_at=now)

    def confirm_exit(self, vehicle_id, visit_id, now, *, fully_outside):
        row = self._read(vehicle_id, visit_id, now, {FleetPhase.EXITING})
        if not fully_outside or row.owned_resources or row.task_id or row.resources_observed_at != now:
            raise ValueError('Full-body departure and a fresh empty resource snapshot must be confirmed')
        return self._update(row, now, 'VEHICLE_DEPARTED', phase=FleetPhase.DEPARTED)
