import pytest

from logistics_park.scheduling.shared_resource import SharedResourceReservations


def test_shared_channel_is_exclusive_and_fifo():
    resources = SharedResourceReservations(("q1_to_w5",))

    assert resources.request("q1_to_w5", "car_1", 0.0)
    assert not resources.request("q1_to_w5", "car_2", 1.0)
    assert not resources.request("q1_to_w5", "car_3", 2.0)
    assert resources.queued("q1_to_w5") == ("car_2", "car_3")
    assert resources.allows("q1_to_w5", "car_1")
    assert not resources.allows("q1_to_w5", "car_2")

    assert resources.release("q1_to_w5", "car_1", 3.0, tail_clear=True)
    assert resources.allows("q1_to_w5", "car_2")
    assert resources.queued("q1_to_w5") == ("car_3",)

    assert resources.release("q1_to_w5", "car_2", 4.0, tail_clear=True)
    assert resources.allows("q1_to_w5", "car_3")


def test_shared_channel_waits_for_tail_clear_and_duplicate_requests_are_idempotent():
    resources = SharedResourceReservations(("loading_lane",))
    assert resources.request("loading_lane", "car_1", 0.0)
    assert resources.request("loading_lane", "car_2", 1.0) is False
    assert resources.request("loading_lane", "car_2", 2.0) is False
    assert resources.queued("loading_lane") == ("car_2",)

    assert resources.release("loading_lane", "car_1", 3.0, tail_clear=False) is False
    assert resources.allows("loading_lane", "car_1")
    assert resources.release("loading_lane", "car_1", 4.0, tail_clear=True)
    assert resources.allows("loading_lane", "car_2")
    assert [event["event"] for event in resources.events] == [
        "REQUESTED", "GRANTED", "REQUESTED", "TAIL_NOT_CLEAR", "RELEASED", "GRANTED"
    ]


def test_shared_channel_rejects_invalid_operations():
    with pytest.raises(ValueError):
        SharedResourceReservations(())
    resources = SharedResourceReservations(("lane",))
    with pytest.raises(ValueError):
        resources.request("missing", "car", 0.0)
    with pytest.raises(ValueError):
        resources.request("lane", "car", -1.0)
    with pytest.raises(ValueError):
        resources.release("lane", "car", 0.0, tail_clear=True)