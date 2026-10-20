"""Canonical action boundaries and full-body sweep sampling, in rear-axle metres."""
from dataclasses import dataclass
import math
from logistics_park.common.messages import VehicleState

HOLD_BEFORE_ENTRY_M = 7.8
TAIL_CLEAR_M = 6.
SWEEP_STEP_M = .25


def holding_line(entry):
    return max(0., entry - HOLD_BEFORE_ENTRY_M)


@dataclass(frozen=True)
class ActionContract:
    start: int
    end: int
    entry: float
    exit: float
    outlet: float

    @classmethod
    def for_group(cls, plan, group, compact=False):
        start, end, entry, exit_s = group
        ordinary = compact and not any(
            plan.model.edges[k].get('reverse') or plan.model.edges[k]['kind'] == 'uturn'
            for k in plan.edges[start:end + 1])
        return cls(start, end, entry, exit_s,
                   min(plan.length, exit_s + (TAIL_CLEAR_M if ordinary else 11.344)))

    @property
    def stop(self):
        return holding_line(self.entry)

    def signature(self, plan):
        return (plan.edges[self.start:self.end + 1], self.entry, self.exit)


def sample_sweep(plan, lo, hi, envelope, name='transition'):
    count = max(1, math.ceil(max(0., hi - lo) / SWEEP_STEP_M))
    return [(s, envelope(VehicleState(name, *plan.point(s), plan.heading(s))))
            for s in (lo + (hi - lo) * i / count for i in range(count + 1))]
