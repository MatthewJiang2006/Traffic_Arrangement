from dataclasses import dataclass, replace
from math import cos, sin

from logistics_park.common.messages import ControlCommand, VehicleState, finite
from logistics_park.map.geometry import wrap_angle


@dataclass(frozen=True)
class VehicleLimits:
    v_max: float = 0.5
    omega_max: float = 3.0
    a_max: float = 0.5
    deceleration: float = 0.7
    length: float = 0.12
    width: float = 0.08
    wheel_track: float = 0.07

    def __post_init__(self) -> None:
        finite(*self.__dict__.values())
        if min(self.__dict__.values()) <= 0:
            raise ValueError("all vehicle limits must be positive")


def wheel_speeds(v: float, omega: float, wheel_track: float) -> tuple[float, float]:
    if wheel_track <= 0:
        raise ValueError("wheel track must be positive")
    return v - omega*wheel_track/2, v + omega*wheel_track/2


def body_speeds(left: float, right: float, wheel_track: float) -> tuple[float, float]:
    if wheel_track <= 0:
        raise ValueError("wheel track must be positive")
    return (left+right)/2, (right-left)/wheel_track


def step(state: VehicleState, command: ControlCommand, dt: float, limits: VehicleLimits) -> VehicleState:
    finite(dt)
    if dt <= 0 or command.vehicle_id != state.vehicle_id:
        raise ValueError("invalid simulation step")
    target = max(0.0, min(limits.v_max, command.v_target)) if command.is_valid_at(state.timestamp) else 0.0
    omega = max(-limits.omega_max, min(limits.omega_max, command.omega_target)) if command.is_valid_at(state.timestamp) else 0.0
    speed = max(state.speed-limits.deceleration*dt, min(state.speed+limits.a_max*dt, target))
    distance = (state.speed+speed)/2 * dt
    # Exact integration of constant yaw rate, using the step's mean linear speed.
    if abs(omega) < 1e-9:
        x, y = state.x + distance*cos(state.heading), state.y + distance*sin(state.heading)
    else:
        radius = distance/(omega*dt)
        x = state.x + radius*(sin(state.heading+omega*dt)-sin(state.heading))
        y = state.y - radius*(cos(state.heading+omega*dt)-cos(state.heading))
    return replace(state, x=x, y=y, heading=wrap_angle(state.heading+omega*dt),
                   speed=speed, omega=omega, timestamp=state.timestamp+dt)
