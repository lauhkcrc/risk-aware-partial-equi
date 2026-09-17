#!/usr/bin/env python3
"""Bounded ROS-only command sequence for Ranger wheel-physics validation."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .core import (
    BASE_STATE_JOINTS,
    RANGER_STEERING_JOINTS,
    RangerCommand,
    ranger_wheel_targets,
)


STRAIGHT_SPEED_M_S = 0.10
SPIN_SPEED_RAD_S = 0.10
INITIAL_SETTLE_DURATION_S = 2.0
INITIAL_SETTLE_TIMEOUT_S = 12.0
INITIAL_STILL_WINDOW_S = 0.5
INITIAL_MAX_WHEEL_SPEED_RAD_S = 0.05
INITIAL_MAX_STEERING_SPEED_RAD_S = 0.10
STRAIGHT_DURATION_S = 2.0
SPIN_DURATION_S = 4.0
STOP_SETTLE_MIN_DURATION_S = 0.25
STOP_SETTLE_TIMEOUT_S = 12.0
STOP_STILL_WINDOW_S = 0.30
STOP_MAX_WHEEL_SPEED_RAD_S = 0.35
STEERING_ALIGNMENT_TIMEOUT_S = 6.0
STEERING_POSITION_TOLERANCE_RAD = 0.04
STEERING_VELOCITY_TOLERANCE_RAD_S = 0.10
STEERING_STILL_WINDOW_S = 0.15


class RangerWheelTask(Node):
    def __init__(self) -> None:
        super().__init__("rpe_ranger_wheel_task", namespace="/sim/rpe")
        self.base = self.create_publisher(Twist, "command/base", 10)
        self.stop = self.create_publisher(Bool, "command/stop", 10)
        self.latest_position: dict[str, float] | None = None
        self.latest_velocity: dict[str, float] | None = None
        self.sim_time_s: float | None = None
        self.create_subscription(JointState, "state/joint_states", self._state, 10)
        self.create_subscription(Clock, "/clock", self._clock_callback, 10)

    def _clock_callback(self, message: Clock) -> None:
        self.sim_time_s = (
            float(message.clock.sec) + float(message.clock.nanosec) * 1.0e-9
        )

    def now_s(self) -> float:
        if self.sim_time_s is None:
            return time.monotonic()
        return self.sim_time_s

    def _state(self, message: JointState) -> None:
        self.latest_position = {
            name: float(value) for name, value in zip(message.name, message.position)
        }
        self.latest_velocity = {
            name: float(value) for name, value in zip(message.name, message.velocity)
        }

    def spin_for(self, duration_s: float) -> None:
        deadline = self.now_s() + duration_s
        wall_deadline = time.monotonic() + max(5.0, duration_s * 3.0)
        while (
            rclpy.ok()
            and self.now_s() < deadline
            and time.monotonic() < wall_deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.now_s() < deadline:
            raise RuntimeError("Isaac simulation clock stalled")

    def command_for(self, *, linear_x: float = 0.0, angular_z: float = 0.0,
                    duration_s: float) -> None:
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        deadline = self.now_s() + duration_s
        wall_deadline = time.monotonic() + max(5.0, duration_s * 3.0)
        while rclpy.ok() and self.now_s() < deadline and time.monotonic() < wall_deadline:
            self.base.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.now_s() < deadline:
            raise RuntimeError("Isaac simulation clock stalled during base command")

    def stop_for(self, duration_s: float) -> None:
        deadline = self.now_s() + min(0.25, duration_s)
        wall_deadline = time.monotonic() + 5.0
        message = Bool(data=True)
        while rclpy.ok() and self.now_s() < deadline and time.monotonic() < wall_deadline:
            self.stop.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.spin_for(max(0.0, duration_s - 0.25))

    def stop_until_still(self) -> float:
        """Brake, then require measured stillness before changing steering."""
        started = self.now_s()
        self.stop_for(STOP_SETTLE_MIN_DURATION_S)
        deadline = started + STOP_SETTLE_TIMEOUT_S
        wall_deadline = time.monotonic() + STOP_SETTLE_TIMEOUT_S * 3.0
        still_since: float | None = None
        while rclpy.ok() and self.now_s() < deadline and time.monotonic() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            state = self.snapshot()
            velocities = state["velocity_rad_s"]
            wheels_still = max(
                abs(velocities[name])
                for name in ("fr_wheel", "fl_wheel", "rl_wheel", "rr_wheel")
            ) <= STOP_MAX_WHEEL_SPEED_RAD_S
            steering_still = max(
                abs(velocities[name]) for name in RANGER_STEERING_JOINTS
            ) <= INITIAL_MAX_STEERING_SPEED_RAD_S
            now = self.now_s()
            if wheels_still and steering_still:
                if still_since is None:
                    still_since = now
                elif now - still_since >= STOP_STILL_WINDOW_S:
                    return now - started
            else:
                still_since = None
        raise RuntimeError(
            "Ranger did not stop before the next phase; joint velocities="
            f"{self.snapshot()['velocity_rad_s']}"
        )

    def align_for_spin(self) -> tuple[float, dict[str, dict[str, float]]]:
        """Request spin steering while the adapter keeps rolling gated."""
        command = RangerCommand.validate(0.0, 0.0, SPIN_SPEED_RAD_S)
        target_values = ranger_wheel_targets(command).steering_positions
        targets = dict(zip(RANGER_STEERING_JOINTS, target_values))
        message = Twist()
        message.angular.z = SPIN_SPEED_RAD_S
        started = self.now_s()
        deadline = started + STEERING_ALIGNMENT_TIMEOUT_S
        wall_deadline = time.monotonic() + STEERING_ALIGNMENT_TIMEOUT_S * 3.0
        still_since: float | None = None
        while rclpy.ok() and self.now_s() < deadline and time.monotonic() < wall_deadline:
            self.base.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)
            state = self.snapshot()
            positions = state["position_rad"]
            velocities = state["velocity_rad_s"]
            aligned = all(
                abs(positions[name] - targets[name])
                <= STEERING_POSITION_TOLERANCE_RAD
                and abs(velocities[name])
                <= STEERING_VELOCITY_TOLERANCE_RAD_S
                for name in RANGER_STEERING_JOINTS
            )
            now = self.now_s()
            if aligned:
                if still_since is None:
                    still_since = now
                elif now - still_since >= STEERING_STILL_WINDOW_S:
                    return now - started, state
            else:
                still_since = None
        state = self.snapshot()
        raise RuntimeError(
            "Ranger steering did not align before spin; target="
            f"{targets}, position={state['position_rad']}, "
            f"velocity={state['velocity_rad_s']}"
        )

    def snapshot(self) -> dict[str, dict[str, float]]:
        if self.latest_position is None or self.latest_velocity is None:
            raise RuntimeError("Isaac joint-state feedback was not received")
        missing = set(BASE_STATE_JOINTS) - set(self.latest_position)
        if missing:
            raise RuntimeError(f"Ranger joint-state feedback is missing {sorted(missing)}")
        position = {name: self.latest_position[name] for name in BASE_STATE_JOINTS}
        velocity = {
            name: self.latest_velocity.get(name, 0.0) for name in BASE_STATE_JOINTS
        }
        if not all(math.isfinite(value) for value in (*position.values(), *velocity.values())):
            raise RuntimeError("Ranger joint-state feedback contains NaN or infinity")
        return {"position_rad": position, "velocity_rad_s": velocity}

    def settle_until_still(self) -> float:
        """Brake until all Ranger joints remain still for a bounded window."""
        started = self.now_s()
        self.stop_for(INITIAL_SETTLE_DURATION_S)
        deadline = started + INITIAL_SETTLE_TIMEOUT_S
        wall_deadline = time.monotonic() + INITIAL_SETTLE_TIMEOUT_S * 3.0
        still_since: float | None = None
        while rclpy.ok() and self.now_s() < deadline and time.monotonic() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            state = self.snapshot()
            velocities = state["velocity_rad_s"]
            wheels_still = max(
                abs(velocities[name])
                for name in ("fr_wheel", "fl_wheel", "rl_wheel", "rr_wheel")
            ) <= INITIAL_MAX_WHEEL_SPEED_RAD_S
            steering_still = max(
                abs(velocities[name])
                for name in (
                    "fr_steering_joint", "fl_steering_joint",
                    "rl_steering_joint", "rr_steering_joint",
                )
            ) <= INITIAL_MAX_STEERING_SPEED_RAD_S
            now = self.now_s()
            if wheels_still and steering_still:
                if still_since is None:
                    still_since = now
                elif now - still_since >= INITIAL_STILL_WINDOW_S:
                    return now - started
            else:
                still_since = None
        state = self.snapshot()
        raise RuntimeError(
            "Ranger did not settle before motion test; joint velocities="
            f"{state['velocity_rad_s']}"
        )


def _mark(directory: Path, phase: str) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / phase).write_text(f"{time.time_ns()}\n")


def main() -> None:
    phase_dir = Path(
        os.environ.get("RPE_RANGER_WHEEL_PHASE_DIR", "/tmp/rpe_ranger_wheel/phases")
    )
    report_path = Path(
        os.environ.get(
            "RPE_RANGER_WHEEL_TASK_REPORT", "/tmp/rpe_ranger_wheel/task_report.json"
        )
    )
    done_path = Path(
        os.environ.get("RPE_RANGER_WHEEL_DONE_FILE", "/tmp/rpe_ranger_wheel/done")
    )
    phase_dir.mkdir(parents=True, exist_ok=True)
    for phase in (
        "settled", "straight", "stopped", "spin_aligned", "spin", "final"
    ):
        (phase_dir / phase).unlink(missing_ok=True)
    report_path.unlink(missing_ok=True)
    done_path.unlink(missing_ok=True)

    report: dict[str, object] = {
        "schema": "rpe.r2.ranger_wheel_task_report/v1",
        "namespace": "/sim/rpe",
        "hardware_access": False,
        "status": "ERROR",
        "commands": {
            "straight_linear_x_m_s": STRAIGHT_SPEED_M_S,
            "spin_angular_z_rad_s": SPIN_SPEED_RAD_S,
            "straight_duration_s": STRAIGHT_DURATION_S,
            "spin_duration_s": SPIN_DURATION_S,
            "steering_alignment_timeout_s": STEERING_ALIGNMENT_TIMEOUT_S,
            "steering_position_tolerance_rad": STEERING_POSITION_TOLERANCE_RAD,
            "steering_velocity_tolerance_rad_s": (
                STEERING_VELOCITY_TOLERANCE_RAD_S
            ),
            "steering_still_window_s": STEERING_STILL_WINDOW_S,
            "stop_settle_min_duration_s": STOP_SETTLE_MIN_DURATION_S,
            "stop_settle_timeout_s": STOP_SETTLE_TIMEOUT_S,
            "stop_still_window_s": STOP_STILL_WINDOW_S,
            "stop_max_wheel_speed_rad_s": STOP_MAX_WHEEL_SPEED_RAD_S,
            "initial_settle_timeout_s": INITIAL_SETTLE_TIMEOUT_S,
            "initial_still_window_s": INITIAL_STILL_WINDOW_S,
            "initial_max_wheel_speed_rad_s": INITIAL_MAX_WHEEL_SPEED_RAD_S,
            "initial_max_steering_speed_rad_s": INITIAL_MAX_STEERING_SPEED_RAD_S,
        },
    }
    # Keep every completed phase in the report even when a later gate fails;
    # contact-calibration failures otherwise discard the evidence needed to
    # distinguish actuation, settling, and final-state problems.
    states: dict[str, dict[str, dict[str, float]]] = {}
    report["states"] = states
    rclpy.init()
    node = RangerWheelTask()
    try:
        deadline = time.monotonic() + 90.0
        while (
            node.base.get_subscription_count() < 1
            or node.latest_position is None
            or node.sim_time_s is None
        ) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.base.get_subscription_count() < 1:
            raise RuntimeError("Isaac base-command subscriber did not appear")
        if node.latest_position is None:
            raise RuntimeError("Isaac joint-state publisher did not appear")
        if node.sim_time_s is None:
            raise RuntimeError("Isaac simulation clock publisher did not appear")

        initial_settle_duration = node.settle_until_still()
        states["settled"] = node.snapshot()
        _mark(phase_dir, "settled")

        node.command_for(
            linear_x=STRAIGHT_SPEED_M_S, duration_s=STRAIGHT_DURATION_S
        )
        states["straight"] = node.snapshot()
        _mark(phase_dir, "straight")

        straight_stop_duration = node.stop_until_still()
        states["stopped"] = node.snapshot()
        _mark(phase_dir, "stopped")

        spin_alignment_duration, spin_aligned_state = node.align_for_spin()
        states["spin_aligned"] = spin_aligned_state
        _mark(phase_dir, "spin_aligned")

        node.command_for(
            angular_z=SPIN_SPEED_RAD_S, duration_s=SPIN_DURATION_S
        )
        states["spin"] = node.snapshot()
        _mark(phase_dir, "spin")

        spin_stop_duration = node.stop_until_still()
        states["final"] = node.snapshot()
        _mark(phase_dir, "final")

        report.update({
            "status": "PASS",
            "states": states,
            "initial_settle_duration_s": initial_settle_duration,
            "straight_stop_duration_s": straight_stop_duration,
            "spin_stop_duration_s": spin_stop_duration,
            "spin_alignment_duration_s": spin_alignment_duration,
            "spin_steering_target_rad": dict(zip(
                RANGER_STEERING_JOINTS,
                ranger_wheel_targets(
                    RangerCommand.validate(0.0, 0.0, SPIN_SPEED_RAD_S)
                ).steering_positions,
            )),
        })
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        done_path.write_text(f"{time.time_ns()}\n")
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
