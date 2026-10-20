"""Advisory conflict windows; actual entry remains guarded by LocalTraffic."""
import bisect
from .maneuver_contract import holding_line
import math
from dataclasses import dataclass
from logistics_park.common.messages import VehicleState
from logistics_park.safety.collision import overlaps


def maneuver_groups(plan):
    if hasattr(plan, '_local_groups'):
        return plan._local_groups
    groups=[];index=0;edges=plan.model.edges
    kinds={'junction','uturn','lane_change','service_access'}
    while index<len(plan.edges):
        edge=edges[plan.edges[index]]
        if edge['kind'] not in kinds:index+=1;continue
        start=index
        def extend(i,row):
            while i+1<len(plan.edges):
                nxt=edges[plan.edges[i+1]]
                if nxt['capacity_alias']!=row['capacity_alias'] and not (row['kind']=='service_access' and nxt['kind']=='service_access'):break
                i+=1
            return i
        index=extend(index,edge)
        while index+1<len(plan.edges):
            probe=index+1
            while probe<len(plan.edges) and edges[plan.edges[probe]]['kind']=='road':probe+=1
            if probe==len(plan.edges) or plan.edge_starts[probe]-plan.edge_ends[index]>=13.8:break
            following=edges[plan.edges[probe]]
            if following['kind'] not in kinds:break
            index=extend(probe,following)
        groups.append((start,index,plan.edge_starts[start],plan.edge_ends[index]));index+=1
    plan._local_groups=tuple(groups)
    return plan._local_groups


@dataclass
class ArrivalProfile:
    stations: list
    times: list

    def eta(self, station):
        if station<=self.stations[0]:return 0.
        if station>self.stations[-1]+1e-6:return math.inf
        # Integration stops within an endpoint tolerance. Requests inside that
        # tolerance use the final sample, including a stationary one-point ETA.
        if station>=self.stations[-1]:return self.times[-1]
        i=bisect.bisect_left(self.stations,station)
        a,b=self.stations[i-1:i+1]
        return self.times[i-1]+(self.times[i]-self.times[i-1])*(station-a)/max(1e-9,b-a)


def arrival_profile(ex,state,now,*,limit_s=120.,dt=.2,hold=None,target_speed=None,speed_scales=None):
    """Free-flow ETA using current speed, route caps, acceleration and gears.

    ETA is advisory, not a promise that a blocked/parked vehicle will depart.
    No physical position is modified by this numerical integration.
    """
    plan=ex.plan;s=ex.progress;v=abs(state.speed);t=0.
    profile=ArrivalProfile([s],[0.]);gears=[cp['s'] for cp in plan.checkpoints[ex.checkpoint:] if cp['event']=='GEAR_CHANGE']
    if ex.finished:return profile
    t=max(0.,(getattr(ex,'service_until',None) or now)-now)
    if t:profile.times[0]=t
    while t<limit_s and s<plan.length-1e-6:
        if hold is not None and s>=hold-.05:break
        if gears and abs(s-gears[0])<.12 and v<.15:
            s=min(plan.length,max(s,gears.pop(0)+.05));v=0.;t+=.5
            profile.stations.append(s);profile.times.append(t);continue
        cap=plan.speed_cap(min(plan.length,s+.02))
        if speed_scales:
            from .travel_time import motion_class
            cap*=speed_scales.get(motion_class(plan,s),1.)
        if target_speed is not None:cap=min(cap,max(0.,target_speed))
        if hold is not None:cap=min(cap,math.sqrt(1.2*max(0.,hold-s)))
        next_v=v+max(-.6*dt,min(.6*dt,cap-v))
        distance=.5*(v+next_v)*dt
        if distance<1e-7:break
        s=min(plan.length,s+distance);v=next_v;t+=dt
        profile.stations.append(s);profile.times.append(t)
    return profile


class ConflictTiming:
    def __init__(self,buffer_s=.75):
        self.buffer_s=buffer_s;self.geometry={};self.pairs={};self.windows={};self.order=[]
        self.events=[];self.last_blockers={};self.statistics=dict(window_frames=0,rolling_speed_adjustments=0,unknown_release_frames=0)

    def sweep(self,plan,group,envelope):
        key=(plan.edges,group)
        if key not in self.geometry:
            lo=holding_line(group[2]);hi=min(plan.length,group[3]+11.344)
            count=max(1,math.ceil((hi-lo)/.5))
            samples=[(lo+(hi-lo)*i/count,envelope(VehicleState('timing-probe',*plan.point(lo+(hi-lo)*i/count),plan.heading(lo+(hi-lo)*i/count)))) for i in range(count+1)]
            if len(self.geometry)>=128:self.geometry.clear();self.pairs.clear()
            self.geometry[key]=samples
        return key,self.geometry[key]

    def conflict(self,one,two):
        keys=(one[0],two[0])
        if keys not in self.pairs:
            cells={}
            for station,box in two[1]:cells.setdefault((math.floor(box.x/11),math.floor(box.y/11)),[]).append((station,box))
            first=[];second=[]
            for sa,a in one[1]:
                x,y=math.floor(a.x/11),math.floor(a.y/11)
                for dx in (-1,0,1):
                    for dy in (-1,0,1):
                        for sb,b in cells.get((x+dx,y+dy),()):
                            if (a.x-b.x)**2+(a.y-b.y)**2<121 and overlaps(a,b,.6):
                                first.append(sa);second.append(sb)
            # Advisory samples use an extra 0.25 m margin. Entry safety still
            # uses the original dense 0.25 m full-body sweep and actual poses.
            value=(min(first),max(first),min(second),max(second)) if first else None
            if len(self.pairs)>=512:self.pairs.clear()
            self.pairs[keys]=value
        return self.pairs[keys]

    def prepare(self,states,executions,groups,holds,local,envelope,now):
        self.windows={};requests={};profiles={};sweeps={}
        for name,ex in executions.items():
            if ex.finished:continue
            group=next((g for g in groups.get(name,()) if g[3]+6>=ex.progress),None)
            if group is None:continue
            # Already-cleared retained tails do not reorder the next entrance.
            if group[3]<ex.progress:
                group=next((g for g in groups.get(name,()) if g[2]>ex.progress),None)
                if group is None:continue
            profile=arrival_profile(ex,states[name],now,hold=holds.get(name),
                                    speed_scales=getattr(local,'eta_speed_scales',None))
            profiles[name]=profile;requests[name]=group;sweeps[name]=self.sweep(ex.plan,group,envelope)
        remaining=set(requests);leaders={n:set() for n in remaining}
        for n in remaining:
            ex=executions[n]
            for other in remaining-{n}:
                state=states[other]
                if math.dist((states[n].x,states[n].y),(state.x,state.y))>40:continue
                error,s=ex.plan.project((state.x,state.y),ex.progress,behind=0,ahead=40)
                angle=abs((state.heading-ex.plan.heading(s)+math.pi)%(2*math.pi)-math.pi)
                if error<1.4 and angle<math.pi/3 and s>ex.progress+.1:leaders[n].add(other)
        def priority(n):
            g=requests[n];ex=executions[n]
            entered=(n,g[0]) in local.passages and ex.progress+6>=g[2]
            age=min(30.,max(0.,now-local.wait_since.get(n,now)))
            return (not entered,profiles[n].eta(g[2])-.05*age,n)
        self.order=[]
        while remaining:
            front=[n for n in remaining if not leaders[n]&remaining]
            n=min(front or remaining,key=priority);self.order.append(n);remaining.remove(n)
        for n in self.order:
            group=requests[n];profile=profiles[n];delay=0.;blocked=[]
            for other in self.order[:self.order.index(n)]:
                # Following is governed by physical headway, not a calendar
                # that waits for the entire leader's road itinerary to clear.
                if other in leaders[n]:continue
                conflict=self.conflict(sweeps[n],sweeps[other])
                if conflict is None:continue
                enter,leave,other_enter,other_leave=conflict
                if leave<executions[n].progress or other_leave+.5<executions[other].progress:continue
                clearance=profiles[other].eta(other_leave+.5)
                other_arrival=profiles[other].eta(other_enter)+self.windows[other]['delay_s']
                own_clearance=profile.eta(leave+.5)+delay
                # Different occupations of the same geometric conflict can
                # already be separated in time. Do not force the early truck
                # to wait for a later truck just because it has higher rank.
                if math.isfinite(own_clearance) and own_clearance+self.buffer_s<=other_arrival:continue
                available=clearance+self.windows[other]['delay_s']+self.buffer_s
                arrival=profile.eta(enter)
                if math.isfinite(arrival) and arrival+delay<available:
                    delay=available-arrival;blocked.append(other)
            eta=profile.eta(group[2])
            self.windows[n]=dict(start=group[0],entry_eta_s=eta,clear_eta_s=profile.eta(group[3]+6),
                delay_s=delay,scheduled_entry_s=now+eta+delay,blocked_by=blocked,
                stop_distance_m=max(0.,holding_line(group[2])-executions[n].progress))
            signature=(group[0],tuple(blocked))
            if blocked and math.isfinite(delay):
                self.statistics['window_frames']+=1
                if self.last_blockers.get(n)!=signature:
                    self.events.append(dict(event='CONFLICT_WINDOW_PLANNED',vehicle_id=n,time=now,
                        entry_eta_s=eta,delay_s=delay,blocked_by=blocked))
            elif blocked:self.statistics['unknown_release_frames']+=1
            self.last_blockers[n]=signature

    def rolling_cap(self,name,requested):
        window=self.windows.get(name)
        if not window or window['delay_s']<=.05:return requested
        time_to_gate=window['entry_eta_s']+window['delay_s']
        if not math.isfinite(time_to_gate):return requested  # Unknown release: retain the physical stop-line guard.
        cap=min(requested,window['stop_distance_m']/max(.5,time_to_gate))
        if .1<cap<requested-.1:self.statistics['rolling_speed_adjustments']+=1
        return cap
