"""FIFO reservations for narrow shared station access areas.

Unlike a junction scheduler, a shared station channel has no movement
priority: one vehicle owns the whole channel until its rear has cleared it.
The resource is deliberately geometry-agnostic so the same state machine can
be used by the lightweight simulator and by the UE runtime.
"""
from dataclasses import dataclass

from logistics_park.common.messages import finite


@dataclass(frozen=True)
class SharedResourceRequest:
    channel_id: str
    vehicle_id: str
    requested_at: float
    sequence: int


class SharedResourceReservations:
    """Manage exclusive, FIFO access to named shared channels."""

    def __init__(self, channel_ids: tuple[str, ...] | list[str]):
        if not channel_ids or len(set(channel_ids)) != len(channel_ids) or any(not c for c in channel_ids):
            raise ValueError("shared channels must have unique non-empty IDs")
        self.channels = tuple(channel_ids)
        self.owners: dict[str, str] = {}
        self.queues: dict[str, dict[str, SharedResourceRequest]] = {c: {} for c in self.channels}
        self.events: list[dict] = []
        self._sequence = 0
        self.max_owners = 0

    def request(self, channel_id: str, vehicle_id: str, now: float) -> bool:
        """Request access; return whether the vehicle owns the channel now."""
        self._validate(channel_id, vehicle_id, now)
        owner = self.owners.get(channel_id)
        if owner == vehicle_id:
            return True
        queue = self.queues[channel_id]
        if vehicle_id not in queue:
            self._sequence += 1
            queue[vehicle_id] = SharedResourceRequest(channel_id, vehicle_id, now, self._sequence)
            self.events.append({"time": now, "event": "REQUESTED", "channel_id": channel_id,
                                "vehicle_id": vehicle_id})
        self._grant_next(channel_id, now)
        return self.owners.get(channel_id) == vehicle_id

    def allows(self, channel_id: str, vehicle_id: str) -> bool:
        self._validate(channel_id, vehicle_id, None)
        return self.owners.get(channel_id) == vehicle_id

    def release(self, channel_id: str, vehicle_id: str, now: float, *, tail_clear: bool) -> bool:
        """Release only after the complete vehicle footprint clears the area."""
        self._validate(channel_id, vehicle_id, now)
        if self.owners.get(channel_id) != vehicle_id:
            raise ValueError("only the channel owner can release it")
        if not tail_clear:
            self.events.append({"time": now, "event": "TAIL_NOT_CLEAR", "channel_id": channel_id,
                                "vehicle_id": vehicle_id})
            return False
        del self.owners[channel_id]
        self.events.append({"time": now, "event": "RELEASED", "channel_id": channel_id,
                            "vehicle_id": vehicle_id})
        self._grant_next(channel_id, now)
        return True

    def cancel_request(self,channel_id,vehicle_id,now):
        """Defer a queued request; an owner needs observed-clear release."""
        self._validate(channel_id,vehicle_id,now)
        if self.owners.get(channel_id)==vehicle_id:raise ValueError('Owner requires physical clearance')
        removed=self.queues[channel_id].pop(vehicle_id,None)
        if removed is not None:
            self.events.append(dict(time=now,event='REQUEST_DEFERRED',channel_id=channel_id,vehicle_id=vehicle_id))
        return removed is not None

    def queued(self, channel_id: str) -> tuple[str, ...]:
        if channel_id not in self.queues:
            raise KeyError(channel_id)
        return tuple(request.vehicle_id for request in sorted(
            self.queues[channel_id].values(), key=lambda request: request.sequence))

    def _grant_next(self, channel_id: str, now: float) -> None:
        if channel_id in self.owners or not self.queues[channel_id]:
            return
        request = min(self.queues[channel_id].values(), key=lambda item: item.sequence)
        del self.queues[channel_id][request.vehicle_id]
        self.owners[channel_id] = request.vehicle_id
        self.max_owners = max(self.max_owners, len(self.owners))
        self.events.append({"time": now, "event": "GRANTED", "channel_id": channel_id,
                            "vehicle_id": request.vehicle_id, "requested_at": request.requested_at})

    def _validate(self, channel_id: str, vehicle_id: str, now: float | None) -> None:
        if channel_id not in self.queues or not vehicle_id:
            raise ValueError("unknown shared channel or vehicle")
        if now is not None:
            finite(now)
            if now < 0:
                raise ValueError("negative shared resource time")