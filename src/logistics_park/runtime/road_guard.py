"""Limit motion with a road-contained modeled stop; brake on uncertainty.

This is a bounded physical model, with native lease expiry and steering
response variations. It is a command filter, not an unbounded-time proof.
"""
import math

from logistics_park.safety.collision import OBB
from .drivable_surface import convex_hull
from .safety_timing import CONTROL_REACTION_S, EMERGENCY_REACTION_S, BRAKING_DECELERATION_M_S2
from .transport_control import BODY_LENGTH, BODY_WIDTH, BODY_OFFSET


def body_polygon(x,y,heading,margin=0.):
    return list(OBB(x+BODY_OFFSET*math.cos(heading),y+BODY_OFFSET*math.sin(heading),heading,
                    BODY_LENGTH+2*margin,BODY_WIDTH+2*margin).corners())


def sweep_points(previous,current):
    x,y,h=previous;nx,ny,nh=current
    angle=abs((nh-h+math.pi)%(2*math.pi)-math.pi)
    radius=math.hypot(BODY_LENGTH/2+BODY_OFFSET,BODY_WIDTH/2)
    margin=.002+radius*(1-math.cos(angle/2))+math.dist((x,y),(nx,ny))*angle/8
    return body_polygon(x,y,h,margin)+body_polygon(nx,ny,nh,margin)


def swept_body(previous,current):
    return convex_hull(sweep_points(previous,current))


class RoadGuard:
    def __init__(self,surface):
        self.surface=surface;self.previous={};self.context={};self.checked=0;self.limited=0;self.trace={};self.frame_cache={};self.last_delta={}
        self.braking_recovery={};self.braking_recoveries=0

    def braking_path_clear(self,state,target,delta,rear_z,initial_delta=0.,lag=.6,gain=.82,previous_target=0.):
        direction=-1 if (state.speed<-.05 or abs(state.speed)<.05 and target<0) else 1
        # Reserve the last issued speed too: an old snapshot or a pending
        # earlier gas command cannot disappear because this command brakes.
        upper=max(abs(state.speed),abs(target),abs(previous_target))
        velocity=upper+.25 if upper>.05 else upper
        reaction=EMERGENCY_REACTION_S if abs(target)<.08 else CONTROL_REACTION_S
        x,y,h=state.x,state.y,state.heading;wheel=initial_delta;t=0.
        if not self.surface.covers(body_polygon(x,y,h),rear_z):return False
        if abs(delta)<1e-7 and abs(initial_delta)<1e-7:
            distance=direction*(velocity*reaction+velocity*velocity/(2*BRAKING_DECELERATION_M_S2))
            return self.surface.covers(swept_body((x,y,h),(x+distance*math.cos(h),y+distance*math.sin(h),h)),rear_z)
        chunk=[];steps=0
        while velocity>.01:
            dt=.15
            if t<reaction:dt=min(dt,reaction-t)
            desired=gain*delta if t<reaction else 0.
            wheel+=(desired-wheel)*(1-math.exp(-dt/lag))
            acceleration=0. if t<reaction else BRAKING_DECELERATION_M_S2
            dt=min(dt,velocity/acceleration) if acceleration else dt
            distance=direction*(velocity*dt-.5*acceleration*dt*dt)
            yaw=distance*math.tan(wheel)/2.3285
            middle=h+yaw/2
            next_pose=(x+distance*math.cos(middle),y+distance*math.sin(middle),h+yaw)
            # The hull of a union equals the hull of the individual hulls.
            # Keep the same corners/margins and construct only the chunk hull.
            chunk.extend(sweep_points((x,y,h),next_pose));steps+=1
            x,y,h=next_pose;velocity=max(0.,velocity-acceleration*dt);t+=dt
            if steps%4==0 or velocity<=.01:
                if not self.surface.covers(convex_hull(chunk),rear_z):return False
                chunk=[]
        return True

    def command(self,state,target,delta,rear_z,now):
        name=state.vehicle_id
        frame=self.frame_cache.get(name)
        if frame is None or frame[0]!=now:
            frame=(now,{});self.frame_cache[name]=frame
        key=(state.x,state.y,state.heading,state.speed,state.timestamp,target,delta,rear_z)
        cached=frame[1].get(key)
        if cached is not None:
            result,trace=cached
            self.previous[name]=(*self.previous[name][:4],result[0]);self.trace[name]=trace
            self.last_delta[name]=result[1]
            return result
        result=self._command(state,target,delta,rear_z,now)
        frame[1][key]=(result,self.trace[name])
        self.last_delta[name]=result[1]
        return result

    def _command(self,state,target,delta,rear_z,now):
        if not self.surface.covers(body_polygon(state.x,state.y,state.heading),rear_z):
            raise RuntimeError(state.vehicle_id+' complete body outside approved pavement')
        context=self.context.get(state.vehicle_id)
        observed=state.timestamp if state.timestamp>0 else now
        if context is None or context[0]!=now:
            previous=self.previous.get(state.vehicle_id);initial=self.last_delta.get(state.vehicle_id,0.)
            if previous and observed==previous[0] and context is not None:
                initial=context[1]
            if previous and not self.surface.covers(swept_body((previous[2],previous[3],previous[1]),
                                                             (state.x,state.y,state.heading)),rear_z):
                raise RuntimeError(state.vehicle_id+' observed complete body sweep outside approved pavement')
            if previous and observed>previous[0] and abs(state.speed)>.2:
                yaw=(state.heading-previous[1]+math.pi)%(2*math.pi)-math.pi
                initial=math.atan(2.3285*yaw/(state.speed*(observed-previous[0])))
                initial=max(-math.radians(40),min(math.radians(40),initial))
            context=(now,initial,previous[4] if previous else 0.)
            self.context[state.vehicle_id]=context
        _,initial,previous_target=context
        self.previous[state.vehicle_id]=(observed,state.heading,state.x,state.y,target)
        self.checked+=1
        recovery=self.braking_recovery.get(state.vehicle_id)
        if recovery is not None:
            if abs(state.speed)<.08:self.braking_recovery.pop(state.vehicle_id)
            else:
                if now-recovery>20.:raise RuntimeError(state.vehicle_id+' did not stop during bounded braking recovery')
                self.limited+=1
                self.previous[state.vehicle_id]=(*self.previous[state.vehicle_id][:4],0.)
                self.trace[state.vehicle_id]=dict(limited=True,target_m_s=0.,reason='await_observed_stop_after_model_rejection')
                return 0.,0.
        def clear(speed,steering):
            if abs(steering)<1e-7 and abs(initial)<1e-7:
                return self.braking_path_clear(state,speed,steering,rear_z,initial,previous_target=previous_target)
            return all(self.braking_path_clear(state,speed,steering,rear_z,initial,lag,gain,previous_target)
                       for lag,gain in ((.2,.82),(.2,1.),(.6,.82),(.6,1.)))
        if clear(target,delta):
            self.trace[state.vehicle_id]=dict(limited=False,target_m_s=target)
            return target,delta
        self.limited+=1
        sign=-1 if target<0 else 1
        if clear(0.,delta):
            low,high=0.,abs(target)
            for _ in range(5):
                middle=(low+high)/2
                if clear(sign*middle,delta):low=middle
                else:high=middle
            speed=sign*low if low>=.085 else 0.
            self.previous[state.vehicle_id]=(*self.previous[state.vehicle_id][:4],speed)
            self.trace[state.vehicle_id]=dict(limited=True,target_m_s=speed,reason='retain_road_contained_emergency_stop')
            return speed,delta
        if clear(0.,0.):
            self.previous[state.vehicle_id]=(*self.previous[state.vehicle_id][:4],0.)
            self.trace[state.vehicle_id]=dict(limited=True,target_m_s=0.,reason='brake_and_centre_steering')
            return 0.,0.
        # Model infeasibility is not an observed pavement departure. Never
        # issue gas from this state; keep full brake until measured stopped.
        # Current-body and swept-body checks remain active on every frame.
        self.braking_recovery[state.vehicle_id]=now;self.braking_recoveries+=1
        self.previous[state.vehicle_id]=(*self.previous[state.vehicle_id][:4],0.)
        self.trace[state.vehicle_id]=dict(limited=True,target_m_s=0.,reason='model_uncertain_full_brake_recovery')
        return 0.,0.

    def acknowledge_brake(self):
        """Only after a fresh observation following accepted full-brake RPCs."""
        for name,previous in self.previous.items():self.previous[name]=(*previous[:4],0.)

    def acknowledge_commands(self,speed_references):
        """Bind the next frame to the ramped references actually sent to UE.

        A route's desired speed is not the adapter's current speed reference.
        Keep observed speed as a separate lower bound in braking_path_clear.
        """
        for name,speed in speed_references.items():
            previous=self.previous[name];self.previous[name]=(*previous[:4],speed)
