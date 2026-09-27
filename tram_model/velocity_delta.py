from collections import deque
import math
from threading import Lock, Thread

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


def stamp_ns(msg: Odometry) -> int:

    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


# сравнение
class VelocityDelta(Node):
    def __init__(self) -> None:
        super().__init__("velocity_delta")
        self.declare_parameter("max_time_diff_s", 0.1)
        self.declare_parameter("localization_time_offset_s", 0.0)
        self.declare_parameter("plot", True)
        self.declare_parameter("plot_window_s", 120.0)
        max_time_diff_s = float(self.get_parameter("max_time_diff_s").value)
        if not math.isfinite(max_time_diff_s) or max_time_diff_s < 0:
            raise ValueError("max_time_diff_s должен быть конечным и неотрицательным")
        self._max_time_diff_ns = round(max_time_diff_s * 1_000_000_000)
        offset_s = float(self.get_parameter("localization_time_offset_s").value)
        if not math.isfinite(offset_s):
            raise ValueError("localization_time_offset_s должен быть конечным")

        self._localization_offset_ns = round(offset_s * 1_000_000_000)
        self.plot_enabled = bool(self.get_parameter("plot").value)
        self.plot_window_s = float(self.get_parameter("plot_window_s").value)
        if not math.isfinite(self.plot_window_s) or self.plot_window_s <= 0:
            raise ValueError("plot_window_s должен быть конечным и положительным")

        self._results = deque(maxlen=200)
        self._localizations = deque(maxlen=200)
        self._last_stamp_ns = None
        self._samples = deque(maxlen=20000)
        self._samples_lock = Lock()
        self._first_stamp_ns = None
        self._sample_count = 0
        self._publisher = self.create_publisher(Float64, "/result/velocity_delta", 10)
        self.create_subscription(
            Odometry, "/result/position", self._on_result, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/localization/kinematic_state", self._on_localization,
            qos_profile_sensor_data,
        )

    def _on_result(self, msg: Odometry) -> None:
        self._reset_on_time_jump(stamp_ns(msg))
        self._results.append(msg)
        self._match()

    def _on_localization(self, msg: Odometry) -> None:
        t = stamp_ns(msg)
        self._reset_on_time_jump(t)
        if self._localizations and t < stamp_ns(self._localizations[-1]):
            return
        self._localizations.append(msg)
        self._match()

    def _reset_on_time_jump(self, t: int) -> None:
        if self._last_stamp_ns is not None and t < self._last_stamp_ns - 1_000_000_000:

            self._results.clear()
            self._localizations.clear()
            with self._samples_lock:
                self._samples.clear()
                self._first_stamp_ns = None
                self._sample_count += 1
            self._last_stamp_ns = t
        elif self._last_stamp_ns is None or t > self._last_stamp_ns:
            self._last_stamp_ns = t

    def _match(self) -> None:
        while self._results and self._localizations:
            result = self._results[0]
            t = stamp_ns(result) + self._localization_offset_ns
            while len(self._localizations) >= 2 and stamp_ns(self._localizations[1]) <= t:
                self._localizations.popleft()
            before = self._localizations[0]
            t0 = stamp_ns(before)
            if t < t0:
                self._results.popleft()
                continue
            if t == t0:
                localization_speed = float(before.twist.twist.linear.x)
                bracket_gap_ns = 0
            else:
                if len(self._localizations) < 2:
                    break
                after = self._localizations[1]
                t1 = stamp_ns(after)
                if t1 - t > self._max_time_diff_ns or t - t0 > self._max_time_diff_ns:
                    self._results.popleft()
                    continue
                fraction = (t - t0) / (t1 - t0)
                localization_speed = (float(before.twist.twist.linear.x) * (1 - fraction)
                                      + float(after.twist.twist.linear.x) * fraction)
                bracket_gap_ns = t1 - t0
            self._results.popleft()
            self._record(result, localization_speed, bracket_gap_ns)

    def _record(self, result: Odometry, localization_speed: float,
                bracket_gap_ns: int) -> None:
        result_speed = float(result.twist.twist.linear.x)
        if not math.isfinite(result_speed) or not math.isfinite(localization_speed):
            self.get_logger().warn("Пропущена пара с неконечной скоростью")
            return

        delta = result_speed - localization_speed
        self._publisher.publish(Float64(data=delta))
        sample_stamp_ns = stamp_ns(result)
        with self._samples_lock:
            if (self._samples and
                    sample_stamp_ns < self._samples[-1][0] - 1_000_000_000):

                self._samples.clear()
                self._first_stamp_ns = None
            if self._first_stamp_ns is None:
                self._first_stamp_ns = sample_stamp_ns
            self._samples.append((sample_stamp_ns, result_speed,
                                  localization_speed, delta))
            cutoff = sample_stamp_ns - round(self.plot_window_s * 1_000_000_000)
            while self._samples and self._samples[0][0] < cutoff:
                self._samples.popleft()
            self._sample_count += 1
        self.get_logger().info(
            f"v_result={result_speed:.3f} м/с, "
            f"v_localization(t_result+offset)={localization_speed:.3f} м/с, "
            f"Δv={delta:+.3f} м/с, интервал={bracket_gap_ns / 1e9:.3f} с")

    def plot_snapshot(self):

        with self._samples_lock:
            return self._sample_count, self._first_stamp_ns, list(self._samples)


# график
class VelocityPlot:

    def __init__(self, node: VelocityDelta) -> None:
        import matplotlib.pyplot as plt

        self._node = node
        self._plt = plt
        self._last_count = -1
        self.figure, (speed_ax, delta_ax) = plt.subplots(
            2, 1, sharex=True, figsize=(11, 7), layout="constrained"
        )
        self._axes = (speed_ax, delta_ax)
        self._result_line, = speed_ax.plot([], [], label="/result/position")
        self._localization_line, = speed_ax.plot(
            [], [], label="/localization/kinematic_state"
        )
        self._delta_line, = delta_ax.plot([], [], color="tab:red", label="Δv")
        speed_ax.set_title("Продольная скорость")
        speed_ax.set_ylabel("Скорость, м/с")
        speed_ax.legend()
        speed_ax.grid(True)
        delta_ax.axhline(0, color="gray", linewidth=1)
        delta_ax.set_title("Δv = результат − локализация")
        delta_ax.set_ylabel("Δv, м/с")
        delta_ax.set_xlabel("Время от первого сообщения, с")
        delta_ax.grid(True)
        self._timer = self.figure.canvas.new_timer(interval=100)
        self._timer.add_callback(self.refresh)
        self._timer.start()

    def refresh(self) -> None:
        count, first_stamp_ns, samples = self._node.plot_snapshot()
        if count == self._last_count or not samples:
            return
        self._last_count = count
        times = [(row[0] - first_stamp_ns) / 1e9 for row in samples]
        self._result_line.set_data(times, [row[1] for row in samples])
        self._localization_line.set_data(times, [row[2] for row in samples])
        self._delta_line.set_data(times, [row[3] for row in samples])
        for ax in self._axes:
            ax.relim()
            ax.autoscale_view(scalex=False)
        self._axes[1].set_xlim(
            max(0, times[-1] - self._node.plot_window_s),
            max(10, times[-1] + 1),
        )
        self.figure.canvas.draw_idle()

    def show(self) -> None:
        self._plt.show(block=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    thread = None
    try:
        node = VelocityDelta()
        if node.plot_enabled:
            plot = VelocityPlot(node)
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            thread = Thread(target=executor.spin, daemon=True)
            thread.start()
            plot.show()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if thread is not None:
            thread.join()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
