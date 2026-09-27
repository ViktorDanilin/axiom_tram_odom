import math
from dataclasses import replace

import rclpy

from tram_model import tram_model as model
from tram_model.ekf import IV, IA
from tram_model.simple_model import WheelIntegrator
from tram_model.train_model_ros import TramModelNode, _stamp_to_sec


class SimpleModelNode(TramModelNode):

    def __init__(self):
        super().__init__()
        self._integrator = WheelIntegrator(self._wheel_max_age, self._max_gap)
        self._wheel_time_offsets = {}
        self._wheel_last_stamps = {}
        self._gnss_fusion = False
        self._velocity_source = "model"
        self._position_source = "ekf"
        self.get_logger().info(
            "Простой режим: интеграл средней скорости тележек; "
            "динамика, EKF и GNSS-коррекция отключены, масштаб пути 1.0")

    def _sync_speed(self):
        v = self._integrator.speed
        self._state = replace(self._state, velocity=v, omega=v / model.R,
                              distance=self._integrator.distance, motor_torque=0.0)

        self._ekf.x[IV], self._ekf.x[IA] = v, 0.0
        self._ekf.P[:] = 0.0

    def _advance(self, t):
        ds = self._integrator.advance(t)
        if self._t_first is None:
            self._t_first = t
        self._t_last = self._integrator.time
        self._sync_speed()
        f = self._follower
        if f.locked:
            f.advance(ds, self._state.velocity)
            if f.route.name != self._follower_route_name:
                self._path.poses.clear()
                self._route_path.poses.clear()
            self._follower_route_name = f.route.name
            self._result_route_name = f.route.name
            self._copy_lock(self._result_follower, f)
            self._ekf.snap_pose(f.x, f.y, f.yaw, f.curv)
        else:
            x, y = self._ekf.position
            yaw = self._ekf.yaw
            x += ds * math.cos(yaw)
            y += ds * math.sin(yaw)
            self._ekf.snap_pose(x, y, yaw, 0.0)
            f.set_prelock_pose(x, y, yaw)
            self._try_route_match(x, y, from_gnss=False)

    # входы
    def _on_wheel(self, which, msg):
        stamp = _stamp_to_sec(msg.header.stamp)
        v = float(msg.velocity) * self._wheel_scale
        if not math.isfinite(v):
            return
        previous = self._wheel_last_stamps.get(which)
        if previous is not None and stamp <= previous:
            return
        self._wheel_last_stamps[which] = stamp

        if which not in self._wheel_time_offsets:
            self._wheel_time_offsets[which] = (
                0.0 if self._t_last is None else self._t_last - stamp)
        t = stamp + self._wheel_time_offsets[which]

        t = max(t, self._t_last) if self._t_last is not None else t
        self._advance(t)
        self._integrator.update(which, t, v)
        self._sync_speed()
        self._publish(msg.header.stamp, t)

    def _on_command(self, msg):
        self._notch = float(msg.position)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
