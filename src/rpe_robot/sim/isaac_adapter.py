#!/usr/bin/env python3
"""ROS message/validation adapter for the Isaac-only R2 visual demo.

Isaac Sim 6.1 and ROS 2 Jazzy both run with Python 3.12 in ``thor``. This
deliberately small process owns the public R2 message types, validation, and
bounded command evolution, then republishes standard messages understood by
Isaac's native ROS 2 bridge:

``/sim/rpe/command/*`` -> ``/sim/rpe/isaac/{base_cmd,joint_command}``.

The adapter is simulation-only.  It has no vendor imports, CAN access, root
hardware aliases, or physical safety service.  The internal topics are an
implementation detail of the Isaac visual launcher and are not part of the
public contract.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, Float32MultiArray
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

from .core import (
    ARM_JOINTS,
    HAND_INDEPENDENT_JOINTS,
    RANGER_STEERING_JOINTS,
    RANGER_WHEEL_JOINTS,
    R2ContractError,
    RangerCommand,
    normalize_hand,
    ranger_wheel_targets,
    validate_arm_command,
)


COMMAND_TIMEOUT_S = 0.5
PUBLISH_PERIOD_S = 0.05
HAND_MAX_SPEED_RAD_S = 0.5
RANGER_STEERING_CHANGE_RAD = 0.05
RANGER_WHEEL_ACCEL_RAD_S2 = 2.0
RANGER_STEERING_POSITION_TOLERANCE_RAD = 0.04
RANGER_STEERING_VELOCITY_TOLERANCE_RAD_S = 0.10
RANGER_STEERING_STILL_WINDOW_S = 0.30
RANGER_STEERING_ALIGNMENT_TIMEOUT_S = 6.0


class IsaacVisualAdapter(Node):
    """Translate the public R2 contract to native Isaac bridge messages."""

    def __init__(self, *, reset_file: Path) -> None:
        super().__init__("rpe_isaac_visual_adapter", namespace="/sim/rpe")
        self._reset_file = reset_file
        self._last_error = ""
        self._paused = False
        self._ranger_wheel_physics = (
            os.environ.get("RPE_PHYSICS_MODE", "visual") == "ranger-wheel"
        )

        self._base = RangerCommand.validate(0.0, 0.0, 0.0)
        self._base_stamp = time.monotonic()
        self._ranger_steering_target = [0.0] * len(RANGER_STEERING_JOINTS)
        self._ranger_steering_position: dict[str, float] | None = None
        self._ranger_steering_velocity: dict[str, float] | None = None
        self._ranger_feedback_time: float | None = None
        self._ranger_steering_gate_started = self._base_stamp
        self._ranger_steering_aligned_since: float | None = self._base_stamp
        self._ranger_steering_ready = True
        self._ranger_steering_timeout_reported = False
        self._ranger_wheel_target = [0.0] * len(RANGER_WHEEL_JOINTS)
        self._ranger_wheel_update_stamp = self._base_stamp
        self._arm_current = [0.0] * len(ARM_JOINTS)
        self._arm_start = list(self._arm_current)
        self._arm_target = list(self._arm_current)
        self._arm_start_stamp = 0.0
        self._arm_duration = 0.0
        self._hand_current = [0.0] * len(HAND_INDEPENDENT_JOINTS)
        self._hand_target = list(self._hand_current)
        self._hand_update_stamp = time.monotonic()

        # Public, namespaced R2 contract.
        self.create_subscription(Twist, "command/base", self._base_callback, 10)
        self.create_subscription(
            JointTrajectory, "command/arm_trajectory", self._arm_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "command/hand", self._hand_callback, 10
        )
        self.create_subscription(Bool, "command/stop", self._stop_callback, 10)
        self.create_service(Trigger, "reset", self._reset_callback)
        self.create_service(SetBool, "pause", self._pause_callback)

        # Internal topics consumed by isaacsim.ros2.bridge OmniGraph nodes.
        self._base_pub = self.create_publisher(Twist, "isaac/base_cmd", 10)
        self._joint_pub = self.create_publisher(JointState, "isaac/joint_command", 10)
        self._ranger_joint_pub = None
        self._traction_mode_pub = None
        if self._ranger_wheel_physics:
            self.create_subscription(
                JointState, "state/joint_states", self._joint_state_callback, 10
            )
            self._ranger_joint_pub = self.create_publisher(
                JointState, "isaac/base_joint_command", 10
            )
            self._traction_mode_pub = self.create_publisher(
                JointState, "isaac/base_traction_mode", 10
            )
        self._status_pub = self.create_publisher(String, "system/visual_status", 10)
        self._timer = self.create_timer(PUBLISH_PERIOD_S, self._publish)

    def _record_error(self, message: str) -> None:
        self._last_error = message
        self.get_logger().warning(message)

    def _base_callback(self, message: Twist) -> None:
        try:
            command = RangerCommand.validate(
                message.linear.x, message.linear.y, message.angular.z
            )
            now = self._control_time()
            if self._ranger_wheel_physics and command.mode.value != "stop":
                steering = list(ranger_wheel_targets(command).steering_positions)
                if max(
                    abs(target - previous)
                    for target, previous in zip(
                        steering, self._ranger_steering_target
                    )
                ) > RANGER_STEERING_CHANGE_RAD:
                    # A swerve chassis must point and settle all wheels before
                    # applying rolling velocity. Repeated messages with the
                    # same target do not restart this feedback gate.
                    self._ranger_steering_gate_started = now
                    self._ranger_steering_aligned_since = None
                    self._ranger_steering_ready = False
                    self._ranger_steering_timeout_reported = False
                    self._publish_traction_mode(0.0)
                self._ranger_steering_target = steering
            self._base = command
            self._base_stamp = now
        except (R2ContractError, TypeError, ValueError) as exc:
            self._record_error(f"rejected base command: {exc}")

    def _joint_state_callback(self, message: JointState) -> None:
        self._ranger_feedback_time = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        positions = {
            name: float(value) for name, value in zip(message.name, message.position)
        }
        velocities = {
            name: float(value) for name, value in zip(message.name, message.velocity)
        }
        if all(name in positions for name in RANGER_STEERING_JOINTS):
            self._ranger_steering_position = positions
            self._ranger_steering_velocity = velocities

    def _control_time(self) -> float:
        """Use Isaac simulation time for Ranger control once feedback exists."""
        if self._ranger_wheel_physics and self._ranger_feedback_time is not None:
            return self._ranger_feedback_time
        return time.monotonic()

    def _steering_feedback_ready(self, now: float) -> bool:
        """Release rolling only after every steering DOF is aligned and still."""
        if self._ranger_steering_ready:
            return True
        positions = self._ranger_steering_position
        velocities = self._ranger_steering_velocity
        aligned = positions is not None and velocities is not None
        if aligned:
            aligned = all(
                math.isfinite(positions[name])
                and math.isfinite(velocities.get(name, math.inf))
                and abs(positions[name] - target)
                <= RANGER_STEERING_POSITION_TOLERANCE_RAD
                and abs(velocities.get(name, math.inf))
                <= RANGER_STEERING_VELOCITY_TOLERANCE_RAD_S
                for name, target in zip(
                    RANGER_STEERING_JOINTS, self._ranger_steering_target
                )
            )
        if aligned:
            if self._ranger_steering_aligned_since is None:
                self._ranger_steering_aligned_since = now
            elif (
                now - self._ranger_steering_aligned_since
                >= RANGER_STEERING_STILL_WINDOW_S
            ):
                self._ranger_steering_ready = True
        else:
            self._ranger_steering_aligned_since = None
        if (
            not self._ranger_steering_ready
            and not self._ranger_steering_timeout_reported
            and now - self._ranger_steering_gate_started
            > RANGER_STEERING_ALIGNMENT_TIMEOUT_S
        ):
            self._ranger_steering_timeout_reported = True
            self._record_error(
                "Ranger steering alignment timed out; wheel velocity remains gated"
            )
        return self._ranger_steering_ready

    def _publish_traction_mode(self, mode: float) -> None:
        """Publish 0=align, 1=drive, or 2=brake to the Isaac contact loop."""
        message = JointState()
        message.name = ["ranger_traction_mode"]
        message.position = [float(mode)]
        self._traction_mode_pub.publish(message)

    def _arm_callback(self, message: JointTrajectory) -> None:
        if not message.points:
            self._record_error("rejected arm command: trajectory has no points")
            return
        point = message.points[-1]
        duration = float(point.time_from_start.sec) + float(
            point.time_from_start.nanosec
        ) * 1.0e-9
        if duration <= 0.0:
            duration = PUBLISH_PERIOD_S
        try:
            command = validate_arm_command(message.joint_names, point.positions, duration)
        except (R2ContractError, TypeError, ValueError) as exc:
            self._record_error(f"rejected arm command: {exc}")
            return
        now = time.monotonic()
        self._update_arm(now)
        self._arm_start = list(self._arm_current)
        self._arm_target = list(command.positions)
        self._arm_start_stamp = now
        self._arm_duration = command.duration_s

    def _hand_callback(self, message: Float32MultiArray) -> None:
        try:
            self._hand_target = list(normalize_hand(message.data))
        except (R2ContractError, TypeError, ValueError) as exc:
            self._record_error(f"rejected hand command: {exc}")

    def _stop_callback(self, message: Bool) -> None:
        if message.data:
            now = self._control_time()
            self._base = RangerCommand.validate(0.0, 0.0, 0.0)
            self._base_stamp = now
            self._update_arm(now)
            self._update_hand(now)
            self._arm_start = list(self._arm_current)
            self._arm_target = list(self._arm_current)
            self._arm_duration = 0.0
            self._hand_target = list(self._hand_current)

    def _reset_callback(self, _request: Trigger.Request, response: Trigger.Response):
        now = self._control_time()
        self._base = RangerCommand.validate(0.0, 0.0, 0.0)
        self._base_stamp = now
        self._ranger_steering_target = [0.0] * len(RANGER_STEERING_JOINTS)
        self._ranger_steering_gate_started = now
        self._ranger_steering_aligned_since = None
        self._ranger_steering_ready = False
        self._ranger_steering_timeout_reported = False
        self._ranger_wheel_target = [0.0] * len(RANGER_WHEEL_JOINTS)
        self._ranger_wheel_update_stamp = now
        self._update_arm(now)
        self._update_hand(now)
        self._arm_start = list(self._arm_current)
        self._arm_target = [0.0] * len(ARM_JOINTS)
        self._arm_start_stamp = now
        self._arm_duration = 1.0
        self._hand_target = [0.0] * len(HAND_INDEPENDENT_JOINTS)
        self._reset_file.parent.mkdir(parents=True, exist_ok=True)
        # The Isaac process polls this private, local marker.  Writing a
        # timestamp avoids stale requests if a previous process was killed.
        self._reset_file.write_text(f"{time.time_ns()}\n")
        response.success = True
        response.message = "Isaac visual scene reset requested; lift remains fixed"
        return response

    def _pause_callback(self, request: SetBool.Request, response: SetBool.Response):
        requested = bool(request.data)
        if requested and not self._paused:
            now = self._control_time()
            self._update_arm(now)
            self._update_hand(now)
            self._arm_start = list(self._arm_current)
            self._arm_target = list(self._arm_current)
            self._arm_duration = 0.0
            self._hand_target = list(self._hand_current)
        self._paused = requested
        response.success = True
        response.message = "paused" if self._paused else "running"
        return response

    def _update_arm(self, now: float) -> None:
        if self._arm_duration <= 0.0:
            return
        ratio = min(
            1.0,
            max(0.0, (now - self._arm_start_stamp) / self._arm_duration),
        )
        self._arm_current = [
            start + ratio * (target - start)
            for start, target in zip(self._arm_start, self._arm_target)
        ]
        if ratio >= 1.0:
            self._arm_duration = 0.0

    def _update_hand(self, now: float) -> None:
        """Slew hand targets to avoid destabilizing tiny imported inertias."""
        dt = max(0.0, now - self._hand_update_stamp)
        self._hand_update_stamp = now
        max_delta = HAND_MAX_SPEED_RAD_S * dt
        if max_delta <= 0.0:
            return
        updated: list[float] = []
        for current, target in zip(self._hand_current, self._hand_target):
            delta = target - current
            if abs(delta) <= max_delta:
                updated.append(target)
            else:
                updated.append(current + math.copysign(max_delta, delta))
        self._hand_current = updated

    def _publish(self) -> None:
        now = self._control_time()
        if not self._paused:
            self._update_arm(now)
            self._update_hand(now)
        else:
            # Prevent a time jump when execution is resumed.
            self._hand_update_stamp = now

        active_base = RangerCommand.validate(0.0, 0.0, 0.0)
        if not self._paused and now - self._base_stamp <= COMMAND_TIMEOUT_S:
            active_base = self._base
        base = Twist()
        base.linear.x = active_base.linear_x
        base.linear.y = active_base.linear_y
        base.angular.z = active_base.angular_z
        self._base_pub.publish(base)

        if self._ranger_wheel_physics:
            targets = ranger_wheel_targets(active_base)
            # Stopping brakes wheel rotation without scrubbing the tyres back
            # to zero steering. Reset explicitly recentres the steering.
            desired_wheels = (
                list(targets.wheel_velocities)
                if self._steering_feedback_ready(now)
                else [0.0] * len(RANGER_WHEEL_JOINTS)
            )
            dt = max(0.0, now - self._ranger_wheel_update_stamp)
            self._ranger_wheel_update_stamp = now
            max_delta = RANGER_WHEEL_ACCEL_RAD_S2 * dt
            for index, desired in enumerate(desired_wheels):
                delta = desired - self._ranger_wheel_target[index]
                self._ranger_wheel_target[index] += max(
                    -max_delta, min(max_delta, delta)
                )
            ranger = JointState()
            ranger.name = list(RANGER_STEERING_JOINTS + RANGER_WHEEL_JOINTS)
            # One atomic action owns all eight Ranger DOFs. Wheel position
            # targets are inert because their USD drives have zero stiffness;
            # steering zero-velocity targets complement their position drive.
            ranger.position = list(self._ranger_steering_target) + [0.0] * len(
                RANGER_WHEEL_JOINTS
            )
            ranger.velocity = [0.0] * len(RANGER_STEERING_JOINTS) + list(
                self._ranger_wheel_target
            )
            self._ranger_joint_pub.publish(ranger)
            traction_mode = (
                0.0
                if not self._ranger_steering_ready
                else (2.0 if active_base.mode.value == "stop" else 1.0)
            )
            self._publish_traction_mode(traction_mode)

        command = JointState()
        command.name = list(ARM_JOINTS + HAND_INDEPENDENT_JOINTS)
        command.position = list(self._arm_current) + list(self._hand_current)
        self._joint_pub.publish(command)

        status = {
            "namespace": "/sim/rpe",
            "hardware_access": False,
            "isaac_native_bridge": True,
            "paused": self._paused,
            "base_command_timed_out": now - self._base_stamp > COMMAND_TIMEOUT_S,
            "lift_fixed": True,
            "hand_max_speed_rad_s": HAND_MAX_SPEED_RAD_S,
            "ranger_wheel_physics": self._ranger_wheel_physics,
            "ranger_steering_feedback_ready": self._ranger_steering_ready,
            "ranger_steering_position_tolerance_rad": (
                RANGER_STEERING_POSITION_TOLERANCE_RAD
            ),
            "ranger_steering_velocity_tolerance_rad_s": (
                RANGER_STEERING_VELOCITY_TOLERANCE_RAD_S
            ),
            "ranger_steering_still_window_s": RANGER_STEERING_STILL_WINDOW_S,
            "ranger_steering_alignment_timeout_s": (
                RANGER_STEERING_ALIGNMENT_TIMEOUT_S
            ),
            "ranger_wheel_accel_rad_s2": RANGER_WHEEL_ACCEL_RAD_S2,
        }
        if self._last_error:
            status["last_rejected_command"] = self._last_error
        message = String()
        message.data = json.dumps(status, sort_keys=True)
        self._status_pub.publish(message)


def main() -> None:
    rclpy.init()
    reset_file = Path(os.environ.get("RPE_R2_RESET_FILE", "/tmp/rpe_r2/reset.request"))
    node = IsaacVisualAdapter(reset_file=reset_file)
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
