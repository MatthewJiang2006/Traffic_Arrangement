import math
import pytest
from logistics_park.runtime.speed_limits import SpeedLimits,bend_stations


def test_approach_brakes_before_uturn():
    limits = SpeedLimits()
    assert limits.approach(limits.uturn_m_s, 20) == 3.0
    assert limits.approach(limits.uturn_m_s, 2) == pytest.approx(math.sqrt(3.4))
    assert limits.approach(limits.uturn_m_s, 0) == 1.0
    assert limits.approach(limits.uturn_m_s, -1) == 1.0


def test_bend_detection_preserves_straight_speed_and_detects_corner():
    assert bend_stations([(0,0),(10,0),(20,0)],[0,10,20]) == []
    assert bend_stations([(0,0),(10,0),(10,10)],[0,10,20]) == [10]


@pytest.mark.parametrize('config', [dict(straight_m_s=float('nan')),
                                  dict(approach_deceleration_m_s2=0),
                                  dict(uturn_m_s=2)])
def test_reject_invalid_speed_limits(config):
    with pytest.raises(ValueError):
        SpeedLimits(**config)
