"""Joint rolling admission for exclusive corridors with observed release.

Forecast windows rank uncommitted requests. They never expire physical leases.
An outlet guard sees the same observation snapshot as the vehicle controller.
"""
from dataclasses import dataclass
from .shared_resource import SharedResourceReservations
from logistics_park.common.messages import finite

@dataclass(frozen=True)
class CorridorRequest:
    vehicle_id: str
    channels: tuple[str,...]
    requested_at: float
    predicted_s: float
    sequence: int

class RollingResourceReservations(SharedResourceReservations):
    def __init__(self,channels,aging_rate=.5):
        super().__init__(channels)
        finite(aging_rate)
        if aging_rate<=0:raise ValueError('Positive aging required')
        self.aging_rate=aging_rate;self.pending={};self.windows={};self.guard=lambda request:True

    def _grant_next(self,channel_id,now):
        # Joint requests are reconsidered together against a fresh snapshot.
        # A release must not silently grant one half of somebody's bundle.
        return

    def request(self,channel_id,vehicle_id,now):
        return self.request_many((channel_id,),vehicle_id,now,predicted_s=1.)

    def request_many(self,channels,vehicle_id,now,*,predicted_s):
        channels=tuple(sorted(set(channels)))
        finite(predicted_s)
        if not channels or predicted_s<=0:raise ValueError('Nonempty positive-duration request required')
        for channel in channels:self._validate(channel,vehicle_id,now)
        if all(self.owners.get(c)==vehicle_id for c in channels):
            self.pending.pop(vehicle_id,None)
            return True
        old=self.pending.get(vehicle_id)
        if old is None:self._sequence+=1
        request=CorridorRequest(vehicle_id,channels,old.requested_at if old else now,predicted_s,old.sequence if old else self._sequence)
        self.pending[vehicle_id]=request
        self.advance(now)
        return all(self.owners.get(c)==vehicle_id for c in channels)

    def advance(self,now):
        finite(now)
        if now<0:raise ValueError('Negative rolling time')
        ordered=sorted(self.pending.values(),key=lambda r:(r.predicted_s-self.aging_rate*(now-r.requested_at),r.sequence))
        for r in ordered:
            if not all(self.owners.get(c) in (None,r.vehicle_id) for c in r.channels) or not self.guard(r):continue
            for channel in r.channels:
                if self.owners.get(channel)!=r.vehicle_id:
                    self.owners[channel]=r.vehicle_id
                    self.windows[channel]=dict(vehicle_id=r.vehicle_id,start=now,predicted_end=now+r.predicted_s,committed=True)
                    self.events.append(dict(time=now,event='GRANTED',channel_id=channel,vehicle_id=r.vehicle_id,requested_at=r.requested_at,joint_channels=list(r.channels)))
            self.max_owners=max(self.max_owners,len(self.owners));del self.pending[r.vehicle_id]

    def release(self,channel_id,vehicle_id,now,*,tail_clear):
        result=super().release(channel_id,vehicle_id,now,tail_clear=tail_clear)
        if result:self.windows.pop(channel_id,None)
        return result

    def queued(self,channel_id):
        if channel_id not in self.channels:raise KeyError(channel_id)
        return tuple(r.vehicle_id for r in sorted(self.pending.values(),key=lambda r:r.sequence) if channel_id in r.channels)
