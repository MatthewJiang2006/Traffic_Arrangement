"""Observed speed response for advisory ETA; never a movement permission."""
import math


def motion_class(plan,station):
    edge=plan.model.edges[plan.edges[plan.edge_index(station)]]
    return 'reverse' if edge.get('reverse') else edge['kind']


class TravelTimeEstimator:
    def __init__(self):
        self.scale={};self.samples={};self.previous={}
        self.leg_factors={};self.leg_waits={};self.leg_samples={}

    def estimate_route(self,network,route,origin,target):
        """Measured travel and queue advisory, separate from entry authority."""
        nominal=route.get('freeflow_travel_time_s',route['estimated_travel_time_s'])
        edges=getattr(network,'edges',{})
        keys=route.get('edge_ids',[])
        if keys and all(k in edges for k in keys):
            nominal=sum(edges[k]['estimated_travel_time_s']/max(.25,self.scale.get(
                'reverse' if edges[k].get('reverse') else edges[k]['kind'],1.)) for k in keys)
        factor=self.leg_factors.get(target,1.)
        moving=nominal*factor
        waiting=max(route.get('congestion_delay_s',0.),self.leg_waits.get(target,0.))
        return dict(nominal_s=nominal,moving_s=moving,queue_s=waiting,total_s=moving+waiting)

    def record_leg(self,target,predicted_moving_s,elapsed_s,stopped_s,*,unchanged_route):
        # A detour changes the travelled distance: never treat its extra time
        # as slower physics on the original itinerary.
        if not unchanged_route or predicted_moving_s<=0 or elapsed_s<=0:return
        ratio=max(.7,min(3.,max(0.,elapsed_s-stopped_s)/predicted_moving_s))
        self.leg_factors[target]=.75*self.leg_factors.get(target,1.)+.25*ratio
        wait=max(0.,min(120.,stopped_s))
        self.leg_waits[target]=.75*self.leg_waits.get(target,0.)+.25*wait
        self.leg_samples[target]=self.leg_samples.get(target,0)+1

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
