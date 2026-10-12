from math import sqrt

from logistics_park.common.messages import VehicleState


def following_speed_cap(rear: VehicleState, front: VehicleState | None, vehicle_length: float,
                        standstill_gap: float = 0.05, time_headway: float = 0.4,
                        deceleration: float = 0.7, reaction_time: float = 0.06) -> float:
    """Same-edge FIFO cap using arc length, time headway and stopping distance."""
    if min(vehicle_length, time_headway, deceleration) <= 0 or min(standstill_gap, reaction_time) < 0:
        raise ValueError("invalid following parameters")
    if front is None or front.edge_id != rear.edge_id:
        return float("inf")
    if front.s <= rear.s:
        raise ValueError("leader must be ahead on the same directed edge")
    gap = max(0.0, front.s - rear.s - vehicle_length - standstill_gap)
    # Treat a stopped leader as the limiting case, including control reaction delay.
    stopping_cap = sqrt((deceleration*reaction_time)**2 + 2*deceleration*gap) - deceleration*reaction_time
    return min(gap/time_headway, stopping_cap)
