from types import SimpleNamespace

import pytest

from tram_model.simple_model import WheelIntegrator


def test_average_and_integral_without_double_counting_paired_messages():
    model = WheelIntegrator(max_age_s=2.0)
    model.update("front", 0.0, 2.0)
    model.update("rear", 0.0, 4.0)
    assert model.speed == 3.0
    model.update("front", 1.0, 4.0)
    model.update("rear", 1.0, 6.0)
    assert model.distance == 3.0
    assert model.speed == 5.0
    model.advance(1.5)
    assert model.distance == 5.5


def test_expiry_splits_interval_and_uses_remaining_sensor():
    model = WheelIntegrator(max_age_s=1.0)
    model.update("front", 0.0, 2.0)
    model.update("rear", 0.5, 4.0)
    model.advance(2.0)

    assert model.distance == pytest.approx(4.5)
    assert model.speed == 0.0


def test_invalid_and_old_samples_do_not_change_state():
    model = WheelIntegrator(max_age_s=2.0)
    model.update("front", 1.0, 2.0)
    model.update("rear", 0.5, 100.0)
    model.update("rear", 2.0, float("nan"))
    assert model.time == 1.0
    assert model.speed == 2.0
    model.advance(2.0)
    assert model.distance == 2.0


def test_large_gap_is_skipped_and_negative_speed_is_clamped():
    model = WheelIntegrator(max_age_s=10.0, max_gap_s=3.0)
    model.update("front", 0.0, 2.0)
    model.update("rear", 4.0, -2.0)
    assert model.distance == 0.0
    assert model.speed == 0.0


def test_ros_callbacks_use_wheels_only():
    from builtin_interfaces.msg import Time
    from tram_vehicle_msgs.msg import VelocitySensor
    from tram_model import tram_model
    from tram_model.ekf import PoseSpeedEkf
    from tram_model.simple_model_ros import SimpleModelNode

    node = SimpleModelNode.__new__(SimpleModelNode)
    node._integrator = WheelIntegrator(max_age_s=2.0)
    node._wheel_time_offsets = {}
    node._wheel_last_stamps = {}
    node._t_first = node._t_last = None
    node._state = tram_model.State(velocity=0.0, distance=0.0)
    node._ekf = PoseSpeedEkf()
    node._wheel_scale = 1 / 3.6
    node._follower = SimpleNamespace(locked=False, set_prelock_pose=lambda *a: None)
    node._try_route_match = lambda *a, **kw: None
    published = []
    node._publish = lambda *a: published.append(node._state.velocity)

    def fail(*args):
        pytest.fail("Простая модель не должна вызывать EKF")

    node._ekf.predict = node._ekf.update_speed = fail
    for which, speed in (("front", 7.2), ("rear", 14.4)):
        msg = VelocitySensor()
        msg.header.stamp = Time(sec=0)
        msg.velocity = speed
        node._on_wheel(which, msg)
    node._on_command(SimpleNamespace(
        header=SimpleNamespace(stamp=Time(sec=1)), position=15))
    assert node._integrator.time == 0.0
    msg.header.stamp = Time(sec=1)
    node._on_wheel("rear", msg)
    assert published[-1] == pytest.approx(3.0)
    assert node._state.distance == pytest.approx(3.0)
    assert node._ekf.position == pytest.approx((3.0, 0.0))


def test_ros_independent_sensor_clocks_and_commands_ahead():
    from builtin_interfaces.msg import Time
    from tram_vehicle_msgs.msg import VelocitySensor
    from tram_model import tram_model
    from tram_model.ekf import PoseSpeedEkf
    from tram_model.simple_model_ros import SimpleModelNode

    node = SimpleModelNode.__new__(SimpleModelNode)
    node._integrator = WheelIntegrator()
    node._wheel_time_offsets = {}
    node._wheel_last_stamps = {}
    node._t_first = node._t_last = None
    node._state = tram_model.State(velocity=0.0, distance=0.0)
    node._ekf = PoseSpeedEkf()
    node._wheel_scale = 1 / 3.6
    node._follower = SimpleNamespace(locked=False, set_prelock_pose=lambda *a: None)
    node._try_route_match = lambda *a, **kw: None
    node._publish = lambda *a: None

    for i in range(21):
        node._on_command(SimpleNamespace(
            header=SimpleNamespace(stamp=Time(sec=1000 + i)), position=8))

        samples = [("front", 100.0 + i * 0.1, 7.2),
                   ("rear", 98.8 + i * 0.1, 14.4)]
        if i % 2:
            samples.reverse()
        for which, t, speed in samples:
            msg = VelocitySensor()
            ns = round(t * 1e9)
            msg.header.stamp = Time(sec=ns // 10**9, nanosec=ns % 10**9)
            msg.velocity = speed
            node._on_wheel(which, msg)
    assert node._state.velocity == pytest.approx(3.0)
    assert node._state.distance == pytest.approx(6.0)
    assert node._ekf.position == pytest.approx((6.0, 0.0))
