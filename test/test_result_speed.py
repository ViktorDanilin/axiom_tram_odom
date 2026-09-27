from types import SimpleNamespace

from builtin_interfaces.msg import Time

from tram_model.train_model_ros import TramModelNode


def test_route_position_correction_does_not_change_published_speed():
    node = TramModelNode.__new__(TramModelNode)
    published = []
    node._ekf = SimpleNamespace(speed=3.0)
    node._state = SimpleNamespace(velocity=2.0)
    node._velocity_source = "ekf"
    node._base_frame = "base_link"
    node._result_follower = SimpleNamespace(locked=True)
    node._result_speed = 4.0
    node._pub_velocity = SimpleNamespace(publish=published.append)
    node._enu = None

    node._publish(Time(), 0.0)

    assert len(published) == 1
    assert published[0].velocity == 3.0


def test_route_model_publishes_physical_speed_in_odometry():
    node = TramModelNode.__new__(TramModelNode)
    velocities, positions = [], []
    node._ekf = SimpleNamespace(speed=3.0, accel=0.0, yaw=0.0, kappa=0.0)
    node._state = SimpleNamespace(velocity=2.0)
    node._velocity_source = "model"
    node._position_source = "route"
    node._base_frame, node._map_frame = "base_link", "map"
    node._follower = SimpleNamespace(
        locked=True, x=10.0, y=20.0, yaw=0.0, curv=0.0, s=10.0,
        route=SimpleNamespace(name="test"))
    node._result_follower = SimpleNamespace(locked=False)
    node._pub_velocity = SimpleNamespace(publish=velocities.append)
    node._pub_position = SimpleNamespace(publish=positions.append)
    node._pub_route_position = SimpleNamespace(publish=lambda msg: None)
    node._enu = object()
    node._to_route_map = lambda x, y, yaw: (x, y, yaw)
    node._append_pose = lambda *args: None
    node._path = node._route_path = None
    node._tf = SimpleNamespace(sendTransform=lambda msg: None)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    node.get_logger = lambda: SimpleNamespace(info=lambda *args, **kwargs: None)
    node._gnss_last_fused_t = None
    node._notch, node._log_period = 0.0, 2.0
    node._input_period = None
    node._slip_mps = 0.0
    node._slip_ratio = 0.0
    node._slip_kind = "ok"
    node._wheel_trust = 1.0
    node._model_slip = 0.0
    node._mu = 0.0
    node._adh_scale = 1.0
    node._slip = False
    node._fault = ""
    node._wheels = {"front": None, "rear": None}
    node._wheel_max_age = 0.25
    node._pub_diag = SimpleNamespace(publish=lambda msg: None)
    node._pub_slip = SimpleNamespace(publish=lambda msg: None)
    node._wheels_slip = SimpleNamespace(
        front=SimpleNamespace(kind="ok"), rear=SimpleNamespace(kind="ok"))

    node._publish(Time(), 0.0)

    assert velocities[0].velocity == 2.0
    assert positions[0].twist.twist.linear.x == 2.0
    assert positions[0].pose.pose.position.x == 10.0
