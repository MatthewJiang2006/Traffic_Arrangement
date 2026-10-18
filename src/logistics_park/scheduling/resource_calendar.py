"""Soft service opportunities. These intervals never grant physical leases."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Interval:
    start: float
    end: float
    owner: str


class ResourceCalendar:
    def __init__(self,capacities=None):
        self.intervals={};self.capacities=dict(capacities or {})
        if any(type(v) is not int or v<1 for v in self.capacities.values()):raise ValueError('Invalid resource capacity')

    def copy(self):
        other=ResourceCalendar(self.capacities)
        other.intervals={key:list(rows) for key,rows in self.intervals.items()}
        return other

    def reserve(self,resources,start,duration,owner):
        if not math.isfinite(start) or not math.isfinite(duration) or start<0 or duration<0:
            raise ValueError('Invalid calendar interval')
        interval=Interval(start,start+duration,owner)
        for resource in set(resources):
            self.intervals.setdefault(resource,[]).append(interval)
        return interval.end

    def earliest(self,resources,start,duration,*,ignore=None):
        if not math.isfinite(start) or not math.isfinite(duration) or start<0 or duration<0:
            raise ValueError('Invalid calendar query')
        rows={resource:[r for r in self.intervals.get(resource,[]) if r.owner!=ignore] for resource in set(resources)}
        while True:
            shifts=[]
            for resource,intervals in rows.items():
                points={start}|{r.start for r in intervals if start<r.start<start+duration}
                for at in points:
                    active=[r for r in intervals if r.start<=at<r.end]
                    if duration and len({r.owner for r in active})>=self.capacities.get(resource,1):
                        shifts.append(min(r.end for r in active))
            if not shifts:return start
            start=max(shifts)

    def trajectory(self,spans,start,owner,*,latest_start=None):
        """Shift departure until the whole route fits, without planned road stops."""
        while True:
            shift=0.
            for offset,resources,duration in spans:
                at=start+offset
                shift=max(shift,self.earliest(resources,at,duration,ignore=owner)-at)
            if not shift:break
            start+=shift
            if latest_start is not None and start>latest_start:return None
        for offset,resources,duration in spans:self.reserve(resources,start+offset,duration,owner)
        return start

    def schedule(self,resources,start,duration,owner):
        begin=self.earliest(resources,start,duration,ignore=owner)
        return begin,self.reserve(resources,begin,duration,owner)
