"""Detect effective fleet stagnation, including wheel spin and tiny drift."""
import math
from logistics_park.common.messages import finite

class FleetProgressWatch:
    def __init__(self,inactivity_s=15.,displacement_m=.1):
        finite(inactivity_s,displacement_m)
        if inactivity_s<=0 or displacement_m<=0:raise ValueError('Positive progress thresholds required')
        self.inactivity_s=inactivity_s;self.displacement_m=displacement_m
        self.anchors={};self.last_activity=None;self.token=None

    def observe(self,states,now,token,*,scheduled_activity_until=None):
        if self.last_activity is None or set(states)!=set(self.anchors):
            self.anchors={n:(s.x,s.y) for n,s in states.items()};self.last_activity=now
        if token!=self.token:self.token=token;self.last_activity=now
        for name,state in states.items():
            if math.dist(self.anchors[name],(state.x,state.y))>=self.displacement_m:
                self.anchors[name]=(state.x,state.y);self.last_activity=now
        if scheduled_activity_until is not None and scheduled_activity_until>now:self.last_activity=now
        return dict(stalled=now-self.last_activity>=self.inactivity_s,inactive_s=now-self.last_activity,
            threshold_s=self.inactivity_s,displacement_threshold_m=self.displacement_m)

def task_progress(executor,now):
    d=executor.dispatch
    token=(len(executor.completed_moves),tuple(sorted((n,a.work.task_id,a.operation,a.cargo_units) for n,a in d.assignments.items())))
    deadlines=[a.service_until for a in d.assignments.values() if a.state=='SERVICING' and a.service_until is not None and a.service_until>now]
    if not d.assignments:
        deadlines.extend(t.release_time for t in d.tasks.values() if t.release_time>now)
    return token,max(deadlines,default=None)


class LocalProgressWatch:
    """Observe local cycles even while unrelated trucks make progress."""
    def __init__(self,cycle_s=20.,waiting_s=90.,displacement_m=.25):
        finite(cycle_s,waiting_s,displacement_m)
        if min(cycle_s,waiting_s,displacement_m)<=0:raise ValueError('Positive local progress thresholds required')
        self.cycle_s=cycle_s;self.waiting_s=waiting_s;self.displacement_m=displacement_m
        self.anchors={};self.last_motion={};self.cycles={};self.blocked_since={}
        self.motion_samples={};self.startup_grace={};self.chain_motion={}

    def observe(self,states,now,cycles,blocked=(),*,dependencies=()):
        for name,state in states.items():
            point=(state.x,state.y)
            sample=self.motion_samples.get(name)
            self.motion_samples[name]=(point,now)
            displacement=math.dist(self.anchors[name],point) if name in self.anchors else math.inf
            # Recognize a real departure before the coarse quarter-metre marker.
            # Velocity alone (wheel spin), or tiny pose jitter, is not progress.
            starting=displacement>=min(.05,self.displacement_m) and abs(state.speed)>=.15
            if name not in self.anchors or displacement>=self.displacement_m or starting:
                self.anchors[name]=point;self.last_motion[name]=now
                self.startup_grace.pop(name,None)
            elif sample and abs(state.speed)>=.15:
                dt=now-sample[1];delta=math.dist(sample[0],point)
                if dt>0 and delta>=max(.01,abs(state.speed)*dt*.1):
                    # One bounded grace for a freshly measured departure which
                    # has not crossed its coarse anchor yet. Do not renew it
                    # from waiting flags, speed alone, or every small sample.
                    self.startup_grace.setdefault(name,now+2.)
        active={tuple(sorted(c)) for c in cycles if all(n in states for n in c)}
        self.cycles={c:self.cycles.get(c,now) for c in active}
        blocked=set(blocked)&set(states)
        self.blocked_since={n:self.blocked_since.get(n,now) for n in blocked}
        graph={}
        for row in dependencies:
            graph.setdefault(row['vehicle'],set()).add(row['blocked_by'])
        dependency_motion={};chains={}
        for name in blocked:
            visited={name};pending=list(graph.get(name,()))
            while pending:
                other=pending.pop()
                if other in visited:continue
                visited.add(other);pending.extend(graph.get(other,()))
            motions=[self.last_motion[n] for n in visited-{name} if n in self.last_motion]
            dependency_motion[name]=max(motions,default=-math.inf)
            chains[name]=visited
        # A resolved dependency cannot erase progress that was just observed
        # through that chain. Keep its timestamp while this queue remains
        # blocked; a former blocker moving later no longer refreshes it.
        self.chain_motion={n:max(self.chain_motion.get(n,-math.inf),dependency_motion[n]) for n in blocked}
        dependency_motion=self.chain_motion
        stuck_cycles=[list(c) for c,start in self.cycles.items()
            if now-max(start,max(self.last_motion[n] for n in c))>=self.cycle_s]
        stalled=[n for n,start in self.blocked_since.items()
            if now-max(start,self.last_motion[n],dependency_motion[n])>=self.waiting_s
            and not any(self.startup_grace.get(other,-math.inf)>now for other in chains[n])]
        return dict(stalled=bool(stuck_cycles or stalled),cycles=sorted(stuck_cycles),
            vehicles=sorted(stalled),inactive_s={n:now-max(start,self.last_motion[n]) for n,start in self.blocked_since.items()},
            blocking_chain_inactive_s={n:now-max(start,self.last_motion[n],dependency_motion[n]) for n,start in self.blocked_since.items()},
            startup_grace_s={n:max(0.,max((self.startup_grace.get(other,now) for other in chains[n]),default=now)-now)
                             for n in self.blocked_since},
            cycle_threshold_s=self.cycle_s,waiting_threshold_s=self.waiting_s)


def local_progress(executor,states,now,watch):
    from logistics_park.scheduling.wait_dependencies import wait_snapshot
    snapshot=wait_snapshot(executor,states)
    controller=executor.controller
    active=snapshot.get('active_dependencies',snapshot['dependencies'])
    blocked={row['vehicle'] for row in active
        if row['vehicle'] in controller.executions and not controller.executions[row['vehicle']].finished
        and controller.executions[row['vehicle']].service_until is None}
    dependencies=active if executor.dispatch.efficiency_policy else ()
    return watch.observe(states,now,snapshot['cycles'],blocked,dependencies=dependencies),snapshot
