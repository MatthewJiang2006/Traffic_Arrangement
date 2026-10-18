"""Shared reaction budget for native lease expiry and physical braking."""
import math

NATIVE_CONTROL_TIMEOUT_S = .5
MAX_NATIVE_FRAME_S = .1
MAX_SOURCE_STATE_AGE_S = .5
MAX_BATCH_STATE_AGE_S = MAX_SOURCE_STATE_AGE_S - .1
CONTROL_REACTION_S = 1.2
EMERGENCY_REACTION_S = MAX_SOURCE_STATE_AGE_S + NATIVE_CONTROL_TIMEOUT_S + MAX_NATIVE_FRAME_S
BRAKING_DECELERATION_M_S2 = .6
MIN_ACTUATED_SPEED_M_S = .08


def validate_native_policy(policy, time_scale):
    expected = dict(require_drivable_surface=True,
                    native_control_timeout_s=NATIVE_CONTROL_TIMEOUT_S,
                    maximum_native_frame_s=MAX_NATIVE_FRAME_S,
                    reaction_budget_s=CONTROL_REACTION_S,
                    maximum_source_state_age_s=MAX_SOURCE_STATE_AGE_S)
    if time_scale != 1 or policy != expected:
        raise ValueError('Native safety policy must match the compiled guard and fixed reaction budget')


def braking_distance(speed, margin_m=.5):
    speed = abs(speed)
    return speed * CONTROL_REACTION_S + speed * speed / (2 * BRAKING_DECELERATION_M_S2) + margin_m


def braking_speed(available_m, leader_speed=0., reaction_s=CONTROL_REACTION_S):
    a = BRAKING_DECELERATION_M_S2
    return max(0., math.sqrt((a * reaction_s)**2 + max(0., leader_speed)**2 + 2 * a * max(0., available_m)) - a * reaction_s)
