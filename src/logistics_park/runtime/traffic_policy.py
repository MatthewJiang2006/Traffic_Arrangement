"""Motion limits for normal following and deliberate stop-line waiting."""
import math
from .safety_timing import CONTROL_REACTION_S, braking_speed,MIN_ACTUATED_SPEED_M_S


def stop_line_speed(distance, reaction_s=CONTROL_REACTION_S):
    """Reach a standstill with a 0.25m buffer, without calendar-based crawling."""
    available=max(0.,distance-.25)
    cap=braking_speed(available,reaction_s=reaction_s)
    # Settle at a traffic hold instead of issuing perpetual sub-actuation crawl.
    return 0. if cap<.15 else cap


def following_speed(separation, leader_speed, *, special=False):
    # Relative braking permits a compact moving queue; stopped leaders retain
    # the ego's full braking distance. Reverse/U-turn trucks need their old room.
    gap=3. if special else 1.2
    clearance=max(0.,separation-8.344-gap)
    if special:
        cap=min(braking_speed(clearance),clearance/1.5)
        return cap if cap>=MIN_ACTUATED_SPEED_M_S else 0.
    v=max(0.,leader_speed)
    cap=min(braking_speed(clearance,v),v+clearance/.7)
    return cap if cap>=MIN_ACTUATED_SPEED_M_S else 0.


def needs_maneuver_room(ex):
    plan=ex.plan
    i=plan.edge_index(ex.progress)
    for j in range(i,len(plan.edges)):
        if plan.edge_starts[j]-ex.progress>15:break
        edge=plan.model.edges[plan.edges[j]]
        if edge.get('reverse') or edge['kind']=='uturn':return True
    return False
