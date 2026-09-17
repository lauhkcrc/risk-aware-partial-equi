#!/usr/bin/env python3
"""ROS-only manager for the bounded Revo2 grasp/contact calibration scene."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from .core import HAND_INDEPENDENT_JOINTS, HAND_MIMIC_JOINTS, normalize_hand


OPEN = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# Conservative normalized targets: thumb opposition/flexion and index flexion
# only. The remaining fingers stay open to avoid an unvalidated power-grasp
# posture or inter-finger collision.
PINCH = (0.275, 0.40, 0.50, 0.0, 0.0, 0.0)


class GraspContactTask(Node):
    def __init__(self) -> None:
        super().__init__("rpe_grasp_contact_task", namespace="/sim/rpe")
        self.publisher = self.create_publisher(Float32MultiArray, "command/hand", 10)
        self.latest_state: dict[str, float] | None = None
        self.create_subscription(
            JointState, "state/joint_states", self._state_callback, 10
        )

    def _state_callback(self, message: JointState) -> None:
        self.latest_state = dict(zip(message.name, message.position))

    def publish_for(
        self, values: tuple[float, ...], duration_s: float
    ) -> dict[str, float]:
        message = Float32MultiArray(data=list(values))
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            self.publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.latest_state is None:
            raise RuntimeError("Isaac joint-state feedback was not received")
        required = set(HAND_INDEPENDENT_JOINTS + HAND_MIMIC_JOINTS)
        missing = required - set(self.latest_state)
        if missing:
            raise RuntimeError(f"Revo2 joint-state feedback is missing {sorted(missing)}")
        state = {name: float(self.latest_state[name]) for name in sorted(required)}
        if not all(math.isfinite(value) for value in state.values()):
            raise RuntimeError("Revo2 joint-state feedback contains NaN or infinity")
        return state


def _write_marker(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"{time.time_ns()}\n")


def main() -> None:
    release_file = Path(
        os.environ.get(
            "RPE_GRASP_RELEASE_FILE", "/tmp/rpe_grasp_contact/release.request"
        )
    )
    open_file = Path(
        os.environ.get("RPE_GRASP_OPEN_FILE", "/tmp/rpe_grasp_contact/open.request")
    )
    done_file = Path(
        os.environ.get("RPE_GRASP_DONE_FILE", "/tmp/rpe_grasp_contact/done")
    )
    report_file = Path(
        os.environ.get(
            "RPE_GRASP_TASK_REPORT", "/tmp/rpe_grasp_contact/task_report.json"
        )
    )
    for marker in (release_file, open_file, done_file, report_file):
        marker.unlink(missing_ok=True)

    rclpy.init()
    node = GraspContactTask()
    report: dict[str, object] = {
        "schema": "rpe.r2.grasp_contact_task_report/v1",
        "namespace": "/sim/rpe",
        "hardware_access": False,
        "status": "ERROR",
    }
    try:
        deadline = time.monotonic() + 90.0
        while (
            node.publisher.get_subscription_count() < 1
            or node.latest_state is None
        ) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.publisher.get_subscription_count() < 1:
            raise RuntimeError("Isaac hand-command subscriber did not appear")
        if node.latest_state is None:
            raise RuntimeError("Isaac joint-state publisher did not appear")

        initial_state = node.publish_for(OPEN, 1.0)
        pinch_state = node.publish_for(PINCH, 3.0)
        _write_marker(release_file)
        hold_state = node.publish_for(PINCH, 3.0)
        _write_marker(open_file)
        open_state = node.publish_for(OPEN, 3.0)
        report.update(
            {
                "status": "PASS",
                "command_policy": "partial thumb-index pinch; other fingers open",
                "normalized_pinch_target": list(PINCH),
                "pinch_target_rad": dict(
                    zip(HAND_INDEPENDENT_JOINTS, normalize_hand(PINCH))
                ),
                "initial_state_rad": initial_state,
                "pinch_state_rad": pinch_state,
                "hold_state_rad": hold_state,
                "open_state_rad": open_state,
            }
        )
        report_file.parent.mkdir(parents=True, exist_ok=True)
        report_file.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        _write_marker(done_file)
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report_file.parent.mkdir(parents=True, exist_ok=True)
        report_file.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
