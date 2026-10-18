"""One versioned arrival profile shared by all queries in an observed frame."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ETAResult:
    eta_s: float
    freeflow_s: float
    reachable: bool
    version: int
    reason: str


class ETAService:
    def __init__(self):
        self.now=None;self.version=0;self.profiles={};self.query_count=0;self.profile_count=0

    def query(self,vehicle,execution,state,now,station,*,hold=None,speed_scales=None):
        if not math.isfinite(now) or now<0 or self.now is not None and now<self.now:
            raise ValueError('Out-of-order ETA frame')
        if not math.isfinite(station) or not 0<=station<=execution.plan.length+1e-6:
            raise ValueError('Invalid ETA station')
        self.now=now
        scales=tuple(sorted((key,round(value,2)) for key,value in (speed_scales or {}).items()))
        key=(vehicle,id(execution.plan),execution.checkpoint,execution.finished,scales)
        cached=self.profiles.get(key)
        if (cached is None or now-cached[1]>=.5 or abs(execution.progress-cached[2])>2.
                or abs(state.speed-cached[3])>.3):
            from .conflict_timing import arrival_profile
            self.version+=1
            cached=(arrival_profile(execution,state,now,limit_s=600.,dt=.5,speed_scales=speed_scales),
                    now,execution.progress,state.speed,self.version)
            self.profiles[key]=cached
            if len(self.profiles)>256:self.profiles={key:cached}
            self.profile_count+=1
        elapsed=max(0.,now-cached[1]) if abs(state.speed)>.1 else 0.
        free=max(0.,cached[0].eta(station)-elapsed);self.query_count+=1
        blocked=hold is not None and station>hold+.05
        finite=math.isfinite(free)
        return ETAResult(math.inf if blocked else free,free,finite and not blocked,cached[4],
                         'held_before_target' if blocked else ('outside_prediction_horizon' if not finite else 'freeflow'))
