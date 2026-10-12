"""Bounded off-road buffering and short-lived berth admission policy."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class EfficiencyPolicy:
    waiting_extra_m: float = 60.
    waiting_eta_s: float = 30.
    waiting_commit_s: float = 60.
    waiting_history_s: float = 120.
    diversion_gain_s: float = 5.
    berth_near_eta_s: float = 15.
    blocked_delay_s: float = 10.
    unavailable_waiting: tuple = ('Q26', 'Q28')
    proactive_admission: bool = False

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            if name in ('unavailable_waiting','proactive_admission'):continue
            value=getattr(self,name)
            if not math.isfinite(value) or value<=0:raise ValueError('Invalid efficiency policy '+name)
        if not isinstance(self.proactive_admission,bool):raise ValueError('Invalid proactive admission flag')

    def parking_enabled(self,node):
        return node not in self.unavailable_waiting

    def affordable(self,remaining_m,extra_m,eta_s):
        return (remaining_m>=0 and extra_m<=self.waiting_extra_m
                and math.isfinite(eta_s) and eta_s<=self.waiting_eta_s)
