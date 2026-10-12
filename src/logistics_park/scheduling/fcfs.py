"""Exclusive junction FCFS baseline; no conflict-zone time windows yet."""
from dataclasses import dataclass
from typing import Callable

from logistics_park.common.messages import finite
from .road_capacity import RoadCapacity


@dataclass
class EntranceRequest:
    vehicle_id: str
    intersection_id: str
    incoming_edge: str
    connector_edge: str
    outgoing_edge: str
    requested_at: float
    lane_s: float


@dataclass
class EntranceGrant:
    request: EntranceRequest
    granted_at: float
    entered_at: float | None = None


class FCFSScheduler:
    def __init__(self, intersection_ids: tuple[str,...], capacity: RoadCapacity):
        if len(set(intersection_ids)) != len(intersection_ids):
            raise ValueError('duplicate intersection IDs')
        self.capacity=capacity
        self.queues: dict[str,dict[str,EntranceRequest]]={key:{} for key in intersection_ids}
        self.owners: dict[str,EntranceGrant]={}
        self.events: list[dict]=[]
        self.max_owners=0
        self.capacity_blocked_attempts=0
        self.entry_blocked_attempts=0

    def request(self, request: EntranceRequest) -> None:
        finite(request.requested_at,request.lane_s)
        if (request.intersection_id not in self.queues or not request.vehicle_id
                or any(edge not in self.capacity.capacities for edge in (request.incoming_edge,request.connector_edge,request.outgoing_edge))):
            raise ValueError('unknown request vehicle, road or junction')
        if request.requested_at < 0 or request.lane_s < 0:
            raise ValueError('invalid request time or road position')
        queue=self.queues[request.intersection_id]
        owner=self.owners.get(request.intersection_id)
        if owner and owner.request.vehicle_id==request.vehicle_id:
            return
        existing=queue.get(request.vehicle_id)
        if existing:
            if (existing.incoming_edge,existing.connector_edge,existing.outgoing_edge)!=(request.incoming_edge,request.connector_edge,request.outgoing_edge):
                raise ValueError('queued movement cannot change')
            existing.lane_s=request.lane_s
            return
        queue[request.vehicle_id]=request
        self.events.append({'time':request.requested_at,'event':'REQUESTED',
                            'intersection_id':request.intersection_id,'vehicle_id':request.vehicle_id,
                            'incoming_edge':request.incoming_edge,'connector_edge':request.connector_edge,
                            'outgoing_edge':request.outgoing_edge})

    def advance(self, now: float, can_enter: Callable[[str,str],bool] | None = None) -> None:
        finite(now)
        if now < 0:
            raise ValueError('negative scheduler time')
        for intersection_id,queue in self.queues.items():
            if intersection_id in self.owners or not queue:
                continue
            # A lane's rear cannot precede its front even when earlier submitted.
            leaders={}
            for request in queue.values():
                old=leaders.get(request.incoming_edge)
                if old is None or (request.lane_s,request.vehicle_id)>(old.lane_s,old.vehicle_id):
                    leaders[request.incoming_edge]=request
            request=min(leaders.values(),key=lambda item:(item.requested_at,item.vehicle_id))
            if now+1e-9 < request.requested_at:
                continue
            if can_enter is not None and not can_enter(request.outgoing_edge,request.vehicle_id):
                self.entry_blocked_attempts+=1
                continue
            if not self.capacity.reserve(request.vehicle_id,request.outgoing_edge):
                self.capacity_blocked_attempts+=1
                continue
            grant=EntranceGrant(request,now)
            self.owners[intersection_id]=grant
            del queue[request.vehicle_id]
            self.events.append({'time':now,'event':'GRANTED','intersection_id':intersection_id,
                                'vehicle_id':request.vehicle_id,'requested_at':request.requested_at,
                                'incoming_edge':request.incoming_edge,'connector_edge':request.connector_edge,
                                'outgoing_edge':request.outgoing_edge})
        self.max_owners=max(self.max_owners,len(self.owners))

    def allows(self, vehicle_id: str, intersection_id: str, connector_edge: str) -> bool:
        grant=self.owners.get(intersection_id)
        return bool(grant and grant.request.vehicle_id==vehicle_id and grant.request.connector_edge==connector_edge)

    def mark_entered(self, intersection_id: str, vehicle_id: str, now: float) -> None:
        finite(now)
        grant=self.owners[intersection_id]
        if grant.request.vehicle_id!=vehicle_id or now+1e-9<grant.granted_at:
            raise ValueError('unauthorized junction entry')
        if grant.entered_at is None:
            grant.entered_at=now
            self.events.append({'time':now,'event':'ENTERED','intersection_id':intersection_id,'vehicle_id':vehicle_id})

    def complete(self, intersection_id: str, vehicle_id: str, now: float) -> None:
        finite(now)
        grant=self.owners[intersection_id]
        if grant.request.vehicle_id!=vehicle_id or grant.entered_at is None or now+1e-9<grant.entered_at:
            raise ValueError('invalid junction completion')
        del self.owners[intersection_id]
        self.events.append({'time':now,'event':'CLEARED','intersection_id':intersection_id,'vehicle_id':vehicle_id})
