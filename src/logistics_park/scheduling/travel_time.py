"""Observed speed response for advisory ETA; never a movement permission."""
import math


def motion_class(plan,station):
    edge=plan.model.edges[plan.edges[plan.edge_index(station)]]
    return 'reverse' if edge.get('reverse') else edge['kind']


class TravelTimeEstimator:
    def __init__(self):
        self.scale={};self.samples={};self.previous={}

    def observe(self,states,executions,targets,now):
        for name,state in states.items():
            ex=executions[name]
            if ex.finished:self.previous.pop(name,None);continue
            kind=motion_class(ex.plan,ex.progress);speed=abs(state.speed);target=abs(targets.get(name,0.))
            observed_at=state.timestamp if state.timestamp>0 else now
            old=self.previous.get(name)
            if old is not None and observed_at<=old[0]:continue
            self.previous[name]=(observed_at,speed,target,kind,state.x,state.y)
            if old is None:continue
            dt=observed_at-old[0]
            if (not .01<=dt<=2. or old[3]!=kind or target<.5 or speed<.2
                    or abs(speed-old[1])/dt>.3 or abs(target-old[2])>.2):continue
            distance=math.hypot(state.x-old[4],state.y-old[5])
            if distance<.005 or distance>max(1.,speed*dt*3.):continue
            # Unreal's physics velocity can be in game seconds while physics
            # advances slower than wall time under a frame clamp. Scheduling
            # needs measured metres per observation second, not that velocity.
            ratio=max(.1,min(1.,distance/(dt*target)))
            self.scale[kind]=.95*self.scale.get(kind,ratio)+.05*ratio
            self.samples[kind]=self.samples.get(kind,0)+1

    def blockage_delay(self,name,local,now,default_s):
        if local is None:return 0.
        blocked=(local.last_denials.get(name) or local.passage_denials.get(name)
                 or local.paused_by.get(name) or local.maneuver_holds.get(name)
                 or local.clearance_pauses.get(name))
        if not blocked:return 0.
        window=getattr(getattr(local,'timing',None),'windows',{}).get(name,{})
        delay=window.get('delay_s',0.)
        age=max(0.,now-local.wait_since.get(name,now))
        return max(delay if math.isfinite(delay) else 0.,min(30.,default_s+.2*age))

    def snapshot(self):
        return dict(speed_scales=dict(self.scale),sample_counts=dict(self.samples))
