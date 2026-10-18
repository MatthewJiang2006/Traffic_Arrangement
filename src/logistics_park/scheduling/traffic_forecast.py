"""Time-indexed road demand, distinct from actual road capacity and permission."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class OccupancyInterval:
    vehicle: str
    alias: str
    start: float
    end: float
    unknown_exit: bool=False


class TrafficForecast:
    def __init__(self,horizon_s=60.):
        if not math.isfinite(horizon_s) or horizon_s<=0:raise ValueError('Invalid forecast horizon')
        self.horizon_s=horizon_s;self.now=None;self.intervals=[];self.version=0;self.capacities={}

    def update(self,states,executions,now,*,holds=None,speed_scales=None,blocked_vehicles=frozenset(),eta_service=None):
        if not math.isfinite(now) or now<0 or self.now is not None and now<self.now:
            raise ValueError('Invalid forecast time')
        if self.now is not None and now-self.now<.5:return
        rows=[];holds=holds or {};scales=speed_scales or {}
        for name,state in states.items():
            ex=executions.get(name)
            if ex is None or ex.finished:continue
            plan=ex.plan;index=plan.edge_index(ex.progress);elapsed=0.
            self.capacities.update(plan.model.raw.get('physical_capacities',{}))
            blocked=abs(state.speed)<.15 and (name in blocked_vehicles or holds.get(name,math.inf)<=ex.progress+.5)
            for i in range(index,len(plan.edges)):
                edge=plan.model.edges[plan.edges[i]]
                distance=max(0.,plan.edge_ends[i]-max(ex.progress,plan.edge_starts[i]))
                scale=scales.get('reverse' if edge.get('reverse') else edge['kind'],1.)
                speed=max(.1,edge['speed_limit']*max(.1,min(1.,scale)))
                duration=distance/speed
                if edge['kind']=='road':
                    # Only known motion produces a finite predicted departure.
                    if blocked:
                        if i==index:rows.append(OccupancyInterval(name,edge['capacity_alias'],now,now+self.horizon_s,True))
                        break
                    entering=elapsed;leaving=elapsed+duration
                    if eta_service is not None:
                        entering=eta_service.query(name,ex,state,now,max(ex.progress,plan.edge_starts[i]),
                            hold=holds.get(name),speed_scales=scales).eta_s
                        leaving=eta_service.query(name,ex,state,now,min(plan.length,plan.edge_ends[i]+4.5),
                            hold=holds.get(name),speed_scales=scales).eta_s
                        if not math.isfinite(entering) or entering>=self.horizon_s:break
                    rows.append(OccupancyInterval(name,edge['capacity_alias'],now+entering,
                                                  now+min(self.horizon_s,leaving),not math.isfinite(leaving)))
                elapsed+=duration
                if elapsed>=self.horizon_s:break
        self.now=now;self.intervals=rows;self.version+=1

    def occupancy(self,alias,at,*,exclude=None):
        if self.now is None or not math.isfinite(at) or not self.now<=at<=self.now+self.horizon_s:
            return None
        rows=[r for r in self.intervals if r.alias==alias and r.vehicle!=exclude and r.start<=at<r.end]
        return dict(count=len({r.vehicle for r in rows}),unknown_count=len({r.vehicle for r in rows if r.unknown_exit}))

    def route_delay(self,network,route,vehicle,departure):
        if self.now is None:return 0.
        time=departure;delay=0.
        for key in route.get('edge_ids',[]):
            edge=network.edges[key];duration=edge['estimated_travel_time_s']
            if edge['kind']=='road':
                predicted=self.occupancy(edge['capacity_alias'],time,exclude=vehicle)
                if predicted:
                    # Advisory nonnegative congestion cost, never a permission.
                    extra=.5*duration*(predicted['count']/max(1,self.capacities.get(edge['capacity_alias'],1)))**2
                    delay+=extra;time+=extra
            time+=duration
        return delay
