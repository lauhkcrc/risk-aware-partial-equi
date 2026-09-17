#!/usr/bin/python3
"""Compare an independent URDF FK implementation with robot_state_publisher.

The process starts only a local ``robot_state_publisher`` instance and feeds it
synthetic JointState messages on private, remapped topics.  It never connects
to a vendor driver and never publishes to any hardware command topic.  The
independent implementation below parses joint origins/axes directly with the
Python standard library and NumPy; RSP provides the second implementation.

The default 1,000 configurations are deliberately offline and bounded.  They
are a regression test for importer/name/order/sign errors, not a physical
robot test.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import random
import signal
import shutil
import subprocess
import sys
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


def xyz_rpy_transform(xyz: list[float], rpy: list[float]) -> np.ndarray:
    x, y, z = xyz
    rr, rp, ry = rpy
    cr, sr = math.cos(rr), math.sin(rr)
    cp, sp = math.cos(rp), math.sin(rp)
    cy, sy = math.cos(ry), math.sin(ry)
    rot = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)
    result = np.eye(4)
    result[:3, :3] = rot
    result[:3, 3] = [x, y, z]
    return result


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    one = 1.0 - c
    result = np.eye(4)
    result[:3, :3] = [
        [c + x * x * one, x * y * one - z * s, x * z * one + y * s],
        [y * x * one + z * s, c + y * y * one, y * z * one - x * s],
        [z * x * one - y * s, z * y * one + x * s, c + z * z * one],
    ]
    return result


def axis_translation(axis: np.ndarray, distance: float) -> np.ndarray:
    result = np.eye(4)
    result[:3, 3] = axis / np.linalg.norm(axis) * distance
    return result


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(4)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    return result


def transform_from_msg(transform: Any) -> np.ndarray:
    return np.block([
        [quaternion_matrix(transform.rotation.x, transform.rotation.y,
                           transform.rotation.z, transform.rotation.w)[:3, :3],
         np.array([[transform.translation.x], [transform.translation.y], [transform.translation.z]])],
        [np.zeros((1, 3)), np.ones((1, 1))],
    ])


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray | None
    lower: float | None
    upper: float | None
    mimic: str | None
    multiplier: float
    offset: float


class Model:
    def __init__(self, path: Path):
        root = ET.parse(path).getroot()
        self.links = [link.attrib["name"] for link in root.findall("link")]
        self.joints: list[Joint] = []
        self.by_parent: dict[str, list[Joint]] = {}
        self.by_name: dict[str, Joint] = {}
        for element in root.findall("joint"):
            origin = element.find("origin")
            origin_xyz = [float(x) for x in (origin.attrib.get("xyz", "0 0 0").split() if origin is not None else ["0", "0", "0"])]
            origin_rpy = [float(x) for x in (origin.attrib.get("rpy", "0 0 0").split() if origin is not None else ["0", "0", "0"])]
            axis_element = element.find("axis")
            axis = None
            if axis_element is not None:
                axis = np.asarray([float(x) for x in axis_element.attrib.get("xyz", "0 0 0").split()], dtype=float)
            limit = element.find("limit")
            lower = float(limit.attrib["lower"]) if limit is not None and "lower" in limit.attrib else None
            upper = float(limit.attrib["upper"]) if limit is not None and "upper" in limit.attrib else None
            mimic = element.find("mimic")
            joint = Joint(
                name=element.attrib["name"], kind=element.attrib["type"],
                parent=element.find("parent").attrib["link"],
                child=element.find("child").attrib["link"],
                origin=xyz_rpy_transform(origin_xyz, origin_rpy), axis=axis,
                lower=lower, upper=upper,
                mimic=mimic.attrib.get("joint") if mimic is not None else None,
                multiplier=float(mimic.attrib.get("multiplier", "1")) if mimic is not None else 1.0,
                offset=float(mimic.attrib.get("offset", "0")) if mimic is not None else 0.0,
            )
            self.joints.append(joint)
            self.by_name[joint.name] = joint
            self.by_parent.setdefault(joint.parent, []).append(joint)
        children = {joint.child for joint in self.joints}
        roots = [link for link in self.links if link not in children]
        if len(roots) != 1:
            raise ValueError(f"expected one root, got {roots}")
        self.root = roots[0]

    def joint_value(self, joint: Joint, values: dict[str, float], resolving: set[str] | None = None) -> float:
        if resolving is None:
            resolving = set()
        if joint.name in resolving:
            raise ValueError(f"mimic cycle at {joint.name}")
        if joint.mimic:
            resolving.add(joint.name)
            target = self.by_name[joint.mimic]
            return joint.multiplier * self.joint_value(target, values, resolving) + joint.offset
        return float(values.get(joint.name, 0.0))

    def fk(self, values: dict[str, float], target: str) -> np.ndarray:
        transforms: dict[str, np.ndarray] = {self.root: np.eye(4)}

        def visit(parent: str) -> None:
            for joint in self.by_parent.get(parent, []):
                transform = joint.origin.copy()
                q = self.joint_value(joint, values)
                if joint.kind in {"revolute", "continuous"}:
                    transform = transform @ axis_rotation(joint.axis, q)
                elif joint.kind == "prismatic":
                    transform = transform @ axis_translation(joint.axis, q)
                transforms[joint.child] = transforms[parent] @ transform
                visit(joint.child)

        visit(self.root)
        if target not in transforms:
            raise ValueError(f"target link {target!r} is not reachable")
        return transforms[target]

    def random_values(self, rng: random.Random) -> dict[str, float]:
        values: dict[str, float] = {}
        for joint in self.joints:
            if joint.mimic or joint.kind == "fixed":
                continue
            if joint.kind in {"revolute", "prismatic"} and joint.lower is not None and joint.upper is not None:
                margin = min(0.05 * (joint.upper - joint.lower), 0.02)
                lower, upper = joint.lower + margin, joint.upper - margin
                values[joint.name] = rng.uniform(lower, upper)
            elif joint.kind == "continuous":
                values[joint.name] = rng.uniform(-math.pi, math.pi)
        return values

    def zero_values(self) -> dict[str, float]:
        return {
            joint.name: 0.0
            for joint in self.joints
            if joint.mimic is None and joint.kind in {"revolute", "continuous", "prismatic"}
        }


def pose_error(expected: np.ndarray, actual: np.ndarray) -> tuple[float, float]:
    translation = float(np.linalg.norm(expected[:3, 3] - actual[:3, 3]))
    relative = expected[:3, :3].T @ actual[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    orientation = math.degrees(math.acos(cosine))
    return translation, orientation


class RspProbe:
    def __init__(self, urdf_text: str, *, root_frame: str, timeout: float = 3.0):
        # Importing ROS must happen under /usr/bin/python3 in the Jazzy image.
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from tf2_msgs.msg import TFMessage

        self.rclpy = rclpy
        self.Node = Node
        self.JointState = JointState
        self.root_frame = root_frame
        self.tf_messages: list[Any] = []
        self.static_messages: list[Any] = []
        self.last_joint_stamp_ns = 0
        self.topic_prefix = f"/rpe_r1_fk_{os.getpid()}_{uuid.uuid4().hex[:8]}"
        self.joint_topic = self.topic_prefix + "/joint_states"
        self.tf_topic = self.topic_prefix + "/tf"
        self.tf_static_topic = self.topic_prefix + "/tf_static"
        self.params_file = tempfile.NamedTemporaryFile(prefix="rpe_fk_", suffix=".yaml", delete=False)
        self.params_path = Path(self.params_file.name)
        self.params_file.write(b"/**:\n  ros__parameters:\n    robot_description: |\n")
        self.params_file.write(b"".join((f"      {line}\n").encode() for line in urdf_text.splitlines()))
        self.params_file.write(b"    publish_frequency: 1000.0\n    ignore_timestamp: true\n")
        self.params_file.close()
        ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
        executable = shutil.which("robot_state_publisher")
        if executable is None:
            candidate = Path(f"/opt/ros/{ros_distro}/lib/robot_state_publisher/robot_state_publisher")
            executable = str(candidate) if candidate.is_file() else None
        if executable is None:
            raise RuntimeError("robot_state_publisher executable was not found")
        self.process = subprocess.Popen([
            executable,
            "--ros-args", "--params-file", str(self.params_path),
            "-r", "/joint_states:=" + self.joint_topic,
            "-r", "/tf:=" + self.tf_topic,
            "-r", "/tf_static:=" + self.tf_static_topic,
            "-r", "__node:=rpe_r1_fk_robot_state_publisher",
        ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
           start_new_session=True)
        rclpy.init(args=[])
        self.node = Node("rpe_r1_fk_probe")
        self.publisher = self.node.create_publisher(JointState, self.joint_topic, 10)
        self.node.create_subscription(TFMessage, self.tf_topic, self.tf_messages.append, 50)
        static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(TFMessage, self.tf_static_topic, self.static_messages.append, static_qos)
        self.timeout = timeout

    def spin_until(self, predicate, timeout: float | None = None) -> bool:
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def publish(self, values: dict[str, float]) -> None:
        message = self.JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        self.last_joint_stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        message.name = list(values)
        message.position = [float(values[name]) for name in values]
        self.publisher.publish(message)

    def transform(self, target: str) -> np.ndarray | None:
        by_child: dict[str, Any] = {}
        for message in self.static_messages + self.tf_messages:
            for transform in message.transforms:
                by_child[transform.child_frame_id] = transform
        chain: list[Any] = []
        current = target
        while current in by_child:
            transform = by_child[current]
            chain.append(transform)
            current = transform.header.frame_id
        if not chain or current != self.root_frame:
            return None
        # RSP may throttle or reorder very fast JointState messages.  Never
        # compare a newly requested configuration against an older TF sample.
        chain_stamps = [
            transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
            for transform in chain
        ]
        if not chain_stamps or max(chain_stamps) < self.last_joint_stamp_ns:
            return None
        result = np.eye(4)
        for transform in reversed(chain):
            result = result @ transform_from_msg(transform.transform)
        return result

    def close(self) -> None:
        try:
            os.killpg(self.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.process.wait()
        stderr = self.process.stderr.read() if self.process.stderr else ""
        self.node.destroy_node()
        self.rclpy.shutdown()
        self.params_path.unlink(missing_ok=True)
        if self.process.returncode not in (0, -15):
            raise RuntimeError(f"robot_state_publisher failed: {stderr[-2000:]}")


def run(args: argparse.Namespace) -> dict[str, Any]:
    model = Model(args.urdf)
    urdf_text = args.urdf.read_text()
    probe = RspProbe(urdf_text, root_frame=model.root, timeout=args.timeout)
    rng = random.Random(args.seed)
    samples: list[dict[str, float]] = []
    samples.append(model.zero_values())
    for _ in range(max(0, args.samples - 1)):
        samples.append(model.random_values(rng))
    translation_errors: list[float] = []
    orientation_errors: list[float] = []
    missing = 0
    failures: list[str] = []
    try:
        if not probe.spin_until(lambda: bool(probe.static_messages), timeout=args.timeout):
            failures.append("robot_state_publisher did not publish static transforms")
        # Do not lose the first JointState while DDS endpoint discovery is
        # still completing.  This is a local test-only publisher/subscriber;
        # waiting here cannot affect a hardware graph.
        if not probe.spin_until(lambda: probe.publisher.get_subscription_count() > 0, timeout=args.timeout):
            failures.append("robot_state_publisher did not discover the synthetic JointState publisher")
        # RSP can legitimately omit a dynamic TF frame for an all-zero sample
        # while its first joint-state cache is being initialized. Prime the
        # private graph with one non-zero synthetic state, then begin the
        # measured samples.
        prime = model.random_values(random.Random(args.seed + 99))
        probe.publish(prime)
        probe.spin_until(lambda: probe.transform(args.target) is not None, timeout=args.timeout)
        for index, values in enumerate(samples):
            expected = model.fk(values, args.target)
            previous_tf_count = len(probe.tf_messages)
            probe.publish(values)
            if not probe.spin_until(
                lambda: len(probe.tf_messages) > previous_tf_count and probe.transform(args.target) is not None,
                timeout=args.timeout,
            ):
                missing += 1
                if len(failures) < 5:
                    failures.append(f"sample {index}: no TF for {args.target}")
                continue
            actual = probe.transform(args.target)
            translation, orientation = pose_error(expected, actual)
            translation_errors.append(translation)
            orientation_errors.append(orientation)
            if translation > args.translation_tolerance or orientation > args.orientation_tolerance:
                if len(failures) < 5:
                    failures.append(
                        f"sample {index}: translation={translation:.9g} m, orientation={orientation:.9g} deg"
                    )
    finally:
        probe.close()

    # A finite-difference Jacobian sanity check on the independent FK parser.
    # This is intentionally separate from the RSP comparison and is useful for
    # catching axis/origin mistakes before Isaac import.
    jacobian_max = 0.0
    arm_joints = [joint for joint in model.joints if joint.name.startswith("right_arm_") and joint.kind in {"revolute", "continuous"} and not joint.mimic]
    if arm_joints:
        values = model.random_values(random.Random(args.seed + 1))
        base = model.fk(values, args.target)
        epsilon = 1e-6
        for joint in arm_joints:
            plus = dict(values); plus[joint.name] = plus.get(joint.name, 0.0) + epsilon
            minus = dict(values); minus[joint.name] = minus.get(joint.name, 0.0) - epsilon
            numerical = (model.fk(plus, args.target)[:3, 3] - model.fk(minus, args.target)[:3, 3]) / (2 * epsilon)
            # Geometric Jacobian column from the joint axis in the world frame.
            # Build the parent transform by evaluating each chain prefix.
            parent_tf = model.fk(values, joint.parent)
            joint_tf = parent_tf @ joint.origin
            axis_world = joint_tf[:3, :3] @ (joint.axis / np.linalg.norm(joint.axis))
            if joint.kind in {"revolute", "continuous"}:
                analytic = np.cross(axis_world, base[:3, 3] - joint_tf[:3, 3])
            else:
                analytic = axis_world
            jacobian_max = max(jacobian_max, float(np.linalg.norm(numerical - analytic)))

    result = {
        "schema": "rpe.robot.fk_cross_check/v1",
        "urdf": str(args.urdf),
        "urdf_sha256": hashlib.sha256(urdf_text.encode()).hexdigest(),
        "target_frame": args.target,
        "samples_requested": len(samples),
        "samples_compared": len(translation_errors),
        "samples_missing_tf": missing,
        "translation_tolerance_m": args.translation_tolerance,
        "orientation_tolerance_deg": args.orientation_tolerance,
        "translation_max_m": max(translation_errors, default=None),
        "translation_mean_m": float(np.mean(translation_errors)) if translation_errors else None,
        "orientation_max_deg": max(orientation_errors, default=None),
        "orientation_mean_deg": float(np.mean(orientation_errors)) if orientation_errors else None,
        "jacobian_position_fd_max_m_per_rad": jacobian_max,
        "failures": failures,
        "status": "PASS" if not failures and len(translation_errors) == len(samples) else "FAIL",
    }
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--target", default="right_base_link")
    parser.add_argument("--samples", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=20260914)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--translation-tolerance", type=float, default=1e-5)
    parser.add_argument("--orientation-tolerance", type=float, default=1e-4)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args(argv)
    try:
        result = run(args)
    except Exception as exc:
        print(f"FK cross-check failed to run: {exc}", file=sys.stderr)
        return 2
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
