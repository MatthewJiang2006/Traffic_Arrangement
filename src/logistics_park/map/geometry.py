"""Polyline geometry used by matching and path tracking."""
from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin

from logistics_park.common.messages import Point, finite


def wrap_angle(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


@dataclass(frozen=True)
class Projection:
    s: float
    point: Point
    heading: float
    lateral_offset: float
    distance: float


@dataclass(frozen=True)
class Polyline:
    points: tuple[Point, ...]

    def __post_init__(self) -> None:
        finite(*(value for point in self.points for value in point))
        if len(self.points) < 2 or any(hypot(b[0]-a[0], b[1]-a[1]) < 1e-9
                                      for a, b in zip(self.points, self.points[1:])):
            raise ValueError("polyline requires at least two distinct consecutive points")

    @property
    def length(self) -> float:
        return sum(hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(self.points, self.points[1:]))

    def at(self, s: float) -> tuple[Point, float]:
        finite(s)
        remaining = max(0.0, min(s, self.length))
        for a, b in zip(self.points, self.points[1:]):
            length = hypot(b[0]-a[0], b[1]-a[1])
            if remaining <= length + 1e-10:
                fraction = min(1.0, remaining / length)
                return ((a[0] + fraction*(b[0]-a[0]), a[1] + fraction*(b[1]-a[1])),
                        atan2(b[1]-a[1], b[0]-a[0]))
            remaining -= length
        a, b = self.points[-2:]
        return b, atan2(b[1]-a[1], b[0]-a[0])

    def project(self, x: float, y: float) -> Projection:
        finite(x, y)
        offset, best = 0.0, None
        for a, b in zip(self.points, self.points[1:]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            length = hypot(dx, dy)
            fraction = max(0.0, min(1.0, ((x-a[0])*dx + (y-a[1])*dy) / length**2))
            px, py = a[0] + fraction*dx, a[1] + fraction*dy
            heading = atan2(dy, dx)
            candidate = Projection(offset + fraction*length, (px, py), heading,
                                   -(x-px)*sin(heading) + (y-py)*cos(heading), hypot(x-px, y-py))
            if best is None or candidate.distance < best.distance:
                best = candidate
            offset += length
        assert best is not None
        return best
