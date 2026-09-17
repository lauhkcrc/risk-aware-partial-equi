#!/usr/bin/env python3
"""ROS 2 bridge for the simulation-only R2 contract.

The node is intentionally namespaced at ``/sim/rpe`` and only uses standard
ROS messages.  It is a deterministic kinematic backend for contract tests and
for connecting a policy container before the Isaac articulation adapter is
enabled.  It has no vendor SDK imports and no hardware topic aliases.
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory

from .core import (
    R2ContractError,
    RangerCommand,
    SimulatorCore,
)


def _yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class R2SimulationBridge(Node):
    """Namespaced ROS contract node; never connects to physical drivers."""

    def __init__(self, *, rate_hz: float = 20.0) -> None:
        super().__init__("rpe_sim_bridge", namespace="/sim/rpe")
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive and finite")
        self.core = SimulatorCore()
        self.rate_hz = float(rate_hz)
        self._last_tick = time.monotonic()
        self._last_error = ""

        # All names are relative to /sim/rpe.  Do not add hardware aliases.
        self.joint_pub = self.create_publisher(JointState, "state/joint_states", 10)
        self.odom_pub = self.create_publisher(Odometry, "state/odometry", 10)
        self.tf_pub = self.create_publisher(TFMessage, "state/tf", 10)
        self.status_pub = self.create_publisher(String, "system/status", 10)

        self.create_subscription(Twist, "command/base", self._base_callback, 10)
        self.create_subscription(
            JointTrajectory, "command/arm_trajectory", self._arm_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "command/hand", self._hand_callback, 10
        )
        # A boolean stop gate is intentionally simulation-local.  It is not
        # /safety/motion_enable and cannot affect a physical controller.
        self.create_subscription(Bool, "command/stop", self._stop_callback, 10)
        self.create_service(Trigger, "reset", self._reset_callback)
        self.create_service(SetBool, "pause", self._pause_callback)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

    def _record_error(self, message: str) -> None:
        self._last_error = message
        self.get_logger().warning(message)

    def _base_callback(self, message: Twist) -> None:
        try:
            command = RangerCommand.validate(
                message.linear.x, message.linear.y, message.angular.z
            )
            self.core.set_base(command)
        except (R2ContractError, TypeError, ValueError) as exc:
            self._record_error(f"rejected base command: {exc}")

    def _arm_callback(self, message: JointTrajectory) -> None:
        if not message.points:
            self._record_error("rejected arm command: trajectory has no points")
            return
        point = message.points[-1]
        duration = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1.0e-9
        if duration <= 0.0:
            duration = 1.0 / self.rate_hz
        try:
            self.core.set_arm(message.joint_names, point.positions, duration)
        except (R2ContractError, TypeError, ValueError) as exc:
            self._record_error(f"rejected arm command: {exc}")

    def _hand_callback(self, message: Float32MultiArray) -> None:
        try:
            self.core.set_hand(message.data)
        except (R2ContractError, TypeError, ValueError) as exc:
            self._record_error(f"rejected hand command: {exc}")

    def _stop_callback(self, message: Bool) -> None:
        if message.data:
            self.core.stop()

    def _reset_callback(self, _request: Trigger.Request, response: Trigger.Response):
        self.core.reset()
        response.success = True
        response.message = "simulation state reset; lift remains fixed"
        return response

    def _pause_callback(self, request: SetBool.Request, response: SetBool.Response):
        self.core.paused = bool(request.data)
        response.success = True
        response.message = "paused" if self.core.paused else "running"
        return response

    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(1.0 / self.rate_hz, max(0.0, now - self._last_tick))
        self._last_tick = now
        self.core.step(dt, now=now)
        stamp = self.get_clock().now().to_msg()
        self._publish_joint_state(stamp)
        self._publish_odometry(stamp)
        self._publish_tf(stamp)
        self._publish_status()

    def _publish_joint_state(self, stamp) -> None:
        values = self.core.all_joint_state()
        message = JointState()
        message.header.stamp = stamp
        message.name = list(values)
        message.position = [values[name] for name in message.name]
        message.velocity = [0.0] * len(message.name)
        self.joint_pub.publish(message)

    def _publish_odometry(self, stamp) -> None:
        state = self.core.state
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "odom"
        message.child_frame_id = "ranger_base_link"
        message.pose.pose.position.x = state.base_x
        message.pose.pose.position.y = state.base_y
        qx, qy, qz, qw = _yaw_quaternion(state.base_yaw)
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = self.core.last_base.linear_x
        message.twist.twist.linear.y = self.core.last_base.linear_y
        message.twist.twist.angular.z = self.core.last_base.angular_z
        self.odom_pub.publish(message)

    def _publish_tf(self, stamp) -> None:
        state = self.core.state
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "ranger_base_link"
        transform.transform.translation.x = state.base_x
        transform.transform.translation.y = state.base_y
        qx, qy, qz, qw = _yaw_quaternion(state.base_yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_pub.publish(TFMessage(transforms=[transform]))

    def _publish_status(self) -> None:
        status = self.core.status()
        if self._last_error:
            status["last_rejected_command"] = self._last_error
        message = String()
        message.data = json.dumps(status, sort_keys=True)
        self.status_pub.publish(message)


def main() -> None:
    rclpy.init()
    node = R2SimulationBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
