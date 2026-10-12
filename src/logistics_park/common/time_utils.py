from dataclasses import dataclass
from .messages import finite


@dataclass
class PeriodicTimer:
    """Deterministic deadlines, including rates not divisible by the base rate."""
    period: float
    next_time: float = 0.0

    def __post_init__(self) -> None:
        finite(self.period, self.next_time)
        if self.period <= 0 or self.next_time < 0:
            raise ValueError("invalid timer")

    def ready(self, now: float) -> bool:
        if now + 1e-10 < self.next_time:
            return False
        while self.next_time <= now + 1e-10:
            self.next_time += self.period
        return True
