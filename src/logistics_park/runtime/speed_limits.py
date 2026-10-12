"""UE target speeds and approach braking, in metres and seconds."""
import math
from dataclasses import dataclass


def bend_stations(path, stations, minimum_curvature=0.015):
    """Locate bends from polyline tangents; straight sections remain uncapped."""
    yaw = [math.atan2(b[1]-a[1], b[0]-a[0]) for a, b in zip(path, path[1:])]
    return [stations[i] for i in range(1, len(yaw))
            if abs((yaw[i]-yaw[i-1]+math.pi) % (2*math.pi)-math.pi)
            / max(.001, (stations[i+1]-stations[i-1])/2) > minimum_curvature]


def edge_speed_limit(edge, model, limits):
    """Opt-in replacement of the legacy 2 m/s straight-junction cap."""
    original=edge['speed_limit']
    if (model.raw.get('curvature_turn_speeds',False) and edge.get('maneuver') in ('LEFT','RIGHT')
            and edge['kind']=='junction' and not edge.get('reverse')):
        line=model.physical_paths.get(edge['capacity_alias'])
        if line is None:return original
        points=line.points;curvature=0.
        for a,b,c in zip(points,points[1:],points[2:]):
            ab,bc,ac=math.dist(a,b),math.dist(b,c),math.dist(a,c)
            cross=abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))
            if ab*bc*ac>1e-8:curvature=max(curvature,2*cross/(ab*bc*ac))
        return min(limits.turn_m_s,math.sqrt(1.2/max(.001,curvature)))
    if (not model.raw.get('freeflow_straight_junctions',False)
            or edge['kind']!='junction' or edge.get('maneuver')!='STRAIGHT'
            or original!=2.):return original
    line=model.physical_paths.get(edge['capacity_alias'])
    if line is None:return original
    points=line.points;stations=[0.]
    for a,b in zip(points,points[1:]):stations.append(stations[-1]+math.dist(a,b))
    if bend_stations(points,stations):return original
    return limits.straight_m_s


@dataclass(frozen=True)
class SpeedLimits:
    straight_m_s: float = 3.0
    turn_m_s: float = 1.5
    uturn_m_s: float = 1.0
    approach_deceleration_m_s2: float = 0.6

    def __post_init__(self):
        values = (self.straight_m_s, self.turn_m_s, self.uturn_m_s,
                  self.approach_deceleration_m_s2)
        if any(not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError('speed limits and deceleration must be finite and positive')
        if not self.uturn_m_s <= self.turn_m_s <= self.straight_m_s:
            raise ValueError('require U-turn <= turn <= straight target speed')

    def approach(self, corner_speed, distance_m):
        """Limit speed early enough to reach the corner target before entry."""
        return min(self.straight_m_s, math.sqrt(
            corner_speed ** 2 + 2 * self.approach_deceleration_m_s2 * max(0., distance_m)))
