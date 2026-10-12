from dataclasses import dataclass
from math import cos, sin

from logistics_park.common.messages import Point, finite


@dataclass(frozen=True)
class OBB:
    x: float
    y: float
    heading: float
    length: float
    width: float

    def __post_init__(self) -> None:
        finite(self.x, self.y, self.heading, self.length, self.width)
        if min(self.length, self.width) <= 0:
            raise ValueError("OBB dimensions must be positive")

    def corners(self, margin: float = 0.0) -> tuple[Point, ...]:
        finite(margin)
        if margin < 0:
            raise ValueError("margin cannot be negative")
        c, s = cos(self.heading), sin(self.heading)
        return tuple((self.x + dx*c - dy*s, self.y + dx*s + dy*c)
                     for dx, dy in ((-self.length/2-margin, -self.width/2-margin),
                                    (self.length/2+margin, -self.width/2-margin),
                                    (self.length/2+margin, self.width/2+margin),
                                    (-self.length/2-margin, self.width/2+margin)))


def overlaps(a: OBB, b: OBB, margin: float = 0.0) -> bool:
    """SAT on both boxes' axes. Touching counts as unsafe."""
    finite(margin)
    if margin < 0:raise ValueError('margin cannot be negative')
    # Project centers and half extents directly; this is the same four-axis
    # SAT as projecting all eight corners, without allocating corner lists.
    ca,sa=cos(a.heading),sin(a.heading);cb,sb=cos(b.heading),sin(b.heading)
    parallel=abs(ca*cb+sa*sb);perpendicular=abs(ca*sb-sa*cb)
    al,aw=a.length/2+margin,a.width/2+margin
    bl,bw=b.length/2+margin,b.width/2+margin
    dx,dy=b.x-a.x,b.y-a.y
    if abs(dx*ca+dy*sa)>al+bl*parallel+bw*perpendicular+1e-12:return False
    if abs(-dx*sa+dy*ca)>aw+bl*perpendicular+bw*parallel+1e-12:return False
    if abs(dx*cb+dy*sb)>bl+al*parallel+aw*perpendicular+1e-12:return False
    if abs(-dx*sb+dy*cb)>bw+al*perpendicular+aw*parallel+1e-12:return False
    return True


def separating_gap(a: OBB,b: OBB,margin: float=0.) -> float:
    """Largest signed separating-axis gap; positive means disjoint boxes."""
    finite(margin)
    if margin<0:raise ValueError('margin cannot be negative')
    ca,sa=cos(a.heading),sin(a.heading);cb,sb=cos(b.heading),sin(b.heading)
    parallel=abs(ca*cb+sa*sb);perpendicular=abs(ca*sb-sa*cb)
    al,aw=a.length/2+margin,a.width/2+margin;bl,bw=b.length/2+margin,b.width/2+margin
    dx,dy=b.x-a.x,b.y-a.y
    return max(abs(dx*ca+dy*sa)-al-bl*parallel-bw*perpendicular,
        abs(-dx*sa+dy*ca)-aw-bl*perpendicular-bw*parallel,
        abs(dx*cb+dy*sb)-bl-al*parallel-aw*perpendicular,
        abs(-dx*sb+dy*cb)-bw-al*perpendicular-aw*parallel)
