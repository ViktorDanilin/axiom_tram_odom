from collections import deque
from threading import Lock

import pytest
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from tram_model.velocity_delta import VelocityDelta


def odom(t: float, speed: float) -> Odometry:
    msg = Odometry()
    msg.header = Header()
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = round((t - int(t)) * 1e9)
    msg.twist.twist.linear.x = speed
    return msg


def comparator(max_diff_s=0.1, offset_s=0.0):
    node = VelocityDelta.__new__(VelocityDelta)
    node._results = deque(maxlen=200)
    node._localizations = deque(maxlen=200)
    node._last_stamp_ns = None
    node._max_time_diff_ns = round(max_diff_s * 1e9)
    node._localization_offset_ns = round(offset_s * 1e9)
    node._samples = deque(maxlen=20000)
    node._samples_lock = Lock()
    node._first_stamp_ns = None
    node._sample_count = 0
    recorded = []
    node._record = lambda result, speed, gap: recorded.append(
        (result.twist.twist.linear.x, speed, gap))
    return node, recorded


def test_interpolates_at_result_stamp_and_reuses_localization():
    node, recorded = comparator()
    node._on_localization(odom(0.98, 0.0))
    node._on_result(odom(1.00, 0.5))
    node._on_result(odom(1.05, 1.75))
    assert recorded == []
    node._on_localization(odom(1.08, 2.5))
    assert [row[1] for row in recorded] == pytest.approx([0.5, 1.75])


def test_skips_large_gap_without_extrapolation():
    node, recorded = comparator()
    node._on_localization(odom(0.5, 0.0))
    node._on_result(odom(0.75, 1.0))
    node._on_localization(odom(1.0, 2.0))
    assert recorded == []


def test_uses_exact_timestamp_without_second_localization():
    node, recorded = comparator()
    node._on_localization(odom(1.0, 2.0))
    node._on_result(odom(1.0, 2.0))
    assert recorded == [(2.0, 2.0, 0)]


def test_optional_offset_samples_later_localization():
    node, recorded = comparator(offset_s=0.1)
    node._on_localization(odom(1.0, 0.0))
    node._on_result(odom(1.0, 1.0))
    node._on_localization(odom(1.2, 2.0))
    assert recorded[0][1] == pytest.approx(1.0)
