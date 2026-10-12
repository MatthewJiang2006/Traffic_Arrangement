"""Motion limits for normal following and deliberate stop-line waiting."""
import math


def stop_line_speed(distance, reaction_s=.2):
    """Reach a standstill with a 0.25m buffer, without calendar-based crawling."""
    available=max(0.,distance-.25)
    cap=max(0.,math.sqrt((.6*reaction_s)**2+1.2*available)-.6*reaction_s)
    # Settle at a traffic hold instead of issuing perpetual sub-actuation crawl.
    return 0. if cap<.15 else cap


def following_speed(separation, leader_speed, *, special=False):
    # Relative braking permits a compact moving queue; stopped leaders retain
    # the ego's full braking distance. Reverse/U-turn trucks need their old room.
    gap=3. if special else 1.2
    clearance=max(0.,separation-8.344-gap)
    if special:return min(math.sqrt(1.2*clearance),clearance/1.5)
    v=max(0.,leader_speed)
    return min(math.sqrt(v*v+1.2*clearance),v+clearance/.7)


def needs_maneuver_room(ex):
    plan=ex.plan
    i=plan.edge_index(ex.progress)
    for j in range(i,len(plan.edges)):
        if plan.edge_starts[j]-ex.progress>15:break
        edge=plan.model.edges[plan.edges[j]]
        if edge.get('reverse') or edge['kind']=='uturn':return True
    return False
