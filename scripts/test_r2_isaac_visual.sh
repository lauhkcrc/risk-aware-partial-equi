#!/usr/bin/env bash
set -euo pipefail

# End-to-end simulation-only smoke test for the Isaac native ROS bridge.
CODEBASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${RPE_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
ROS_PYTHON="${ROS_PYTHON:-/usr/bin/python3}"
ISAAC_PYTHON="${ISAAC_PYTHON:-/opt/isaaclab/bin/python}"
URDF="${RPE_DESCRIPTION:-/workspace/assets/combined_robot_ur5e_single_arm/urdf/total_robot.urdf}"
USD="${RPE_R2_USD:-/tmp/rpe_r2/combined_robot_ur5e_single_arm.usd}"
REPORT="${RPE_R2_VISUAL_REPORT:-/tmp/rpe_r2/isaac_visual_smoke_report.json}"
RESET_FILE="${RPE_R2_RESET_FILE:-/tmp/rpe_r2/reset.request}"
SCENE_REPORT="${RPE_R2_ISAAC_REPORT:-/tmp/rpe_r2/isaac_scene_report.json}"
ISAAC_LOG="${RPE_R2_VISUAL_LOG:-/tmp/rpe_r2/isaac_visual_smoke.log}"
ADAPTER_LOG="${RPE_R2_ADAPTER_LOG:-/tmp/rpe_r2/isaac_visual_adapter.log}"
PHYSICS_MODE="${RPE_PHYSICS_MODE:-visual}"
SPAWN_HEIGHT_M="${RPE_SPAWN_HEIGHT_M:-0.324}"

if [[ -f "$ROS_SETUP" ]]; then
  set +u
  source "$ROS_SETUP"
  set -u
fi
export PYTHONPATH="$CODEBASE_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"

if [[ ! -x "$ISAAC_PYTHON" ]]; then
  echo "SKIP: Isaac interpreter not present at $ISAAC_PYTHON"
  exit 0
fi
if [[ ! -x "$ROS_PYTHON" ]]; then
  echo "FAIL: ROS Python interpreter not present at $ROS_PYTHON" >&2
  exit 1
fi
if [[ ! -f "$URDF" ]]; then
  echo "FAIL: canonical URDF not found at $URDF" >&2
  exit 1
fi

mkdir -p "$(dirname "$USD")" "$(dirname "$REPORT")" "$(dirname "$RESET_FILE")"
rm -f "$RESET_FILE" "$REPORT" "$ISAAC_LOG" "$ADAPTER_LOG"
if [[ "${RPE_SKIP_IMPORT:-0}" == "1" ]]; then
  if [[ ! -f "$USD" ]]; then
    echo "FAIL: RPE_SKIP_IMPORT=1 but USD does not exist at $USD" >&2
    exit 1
  fi
else
  env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
    "$ISAAC_PYTHON" "$CODEBASE_ROOT/src/rpe_robot/sim/isaac_scene.py" \
    --urdf "$URDF" --output-usd "$USD" --report "$SCENE_REPORT" \
    --physics-mode "$PHYSICS_MODE" --spawn-height "$SPAWN_HEIGHT_M"
fi

cleanup() {
  if [[ -n "${ISAAC_PID:-}" ]] && kill -0 "$ISAAC_PID" 2>/dev/null; then
    kill "$ISAAC_PID" 2>/dev/null || true
    wait "$ISAAC_PID" 2>/dev/null || true
  fi
  if [[ -n "${ADAPTER_PID:-}" ]] && kill -0 "$ADAPTER_PID" 2>/dev/null; then
    kill "$ADAPTER_PID" 2>/dev/null || true
    wait "$ADAPTER_PID" 2>/dev/null || true
  fi
  rm -f "$RESET_FILE"
}
trap cleanup EXIT INT TERM

"$ROS_PYTHON" -m rpe_robot.sim.isaac_adapter >"$ADAPTER_LOG" 2>&1 &
ADAPTER_PID=$!
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  "$ISAAC_PYTHON" "$CODEBASE_ROOT/src/rpe_robot/sim/isaac_visual_demo.py" \
  --usd "$USD" --duration "${RPE_R2_VISUAL_DURATION:-30}" \
  --report "$REPORT" --reset-file "$RESET_FILE" \
  --physics-mode "$PHYSICS_MODE" \
  >"$ISAAC_LOG" 2>&1 &
ISAAC_PID=$!

# Isaac Sim 6.1 can take about one minute on a cold start while extensions
# initialize.  Wait generously for its native publisher before probing.
for _ in $(seq 1 "${RPE_R2_STARTUP_POLLS:-240}"); do
  if ros2 topic list 2>/dev/null | grep -qx "/sim/rpe/state/joint_states"; then
    break
  fi
  sleep 0.5
done
if ! ros2 topic list 2>/dev/null | grep -qx "/sim/rpe/state/joint_states"; then
  echo "FAIL: Isaac native joint-state publisher did not appear" >&2
  exit 1
fi

"$ROS_PYTHON" - "$REPORT" "$PHYSICS_MODE" "$SCENE_REPORT" "$SPAWN_HEIGHT_M" <<'PY'
import json
import math
import pathlib
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = [
    "right_arm_shoulder_pan_joint",
    "right_arm_shoulder_lift_joint",
    "right_arm_elbow_joint",
    "right_arm_wrist_1_joint",
    "right_arm_wrist_2_joint",
    "right_arm_wrist_3_joint",
]
HAND = [
    "right_thumb_metacarpal_joint",
    "right_thumb_proximal_joint",
    "right_index_proximal_joint",
    "right_middle_proximal_joint",
    "right_ring_proximal_joint",
    "right_pinky_proximal_joint",
]
HAND_MIMIC = [
    "right_thumb_distal_joint",
    "right_index_distal_joint",
    "right_middle_distal_joint",
    "right_ring_distal_joint",
    "right_pinky_distal_joint",
]
BASE = [
    "fr_steering_joint",
    "fr_wheel",
    "fl_steering_joint",
    "fl_wheel",
    "rl_steering_joint",
    "rl_wheel",
    "rr_steering_joint",
    "rr_wheel",
]


class Probe(Node):
    def __init__(self):
        super().__init__("rpe_isaac_visual_probe")
        self.latest = None
        self.latest_command = None
        self.create_subscription(JointState, "/sim/rpe/state/joint_states", self._state, 10)
        self.create_subscription(
            JointState, "/sim/rpe/isaac/joint_command", self._command, 10
        )
        self.base = self.create_publisher(Twist, "/sim/rpe/command/base", 10)
        self.arm = self.create_publisher(JointTrajectory, "/sim/rpe/command/arm_trajectory", 10)
        self.hand = self.create_publisher(Float32MultiArray, "/sim/rpe/command/hand", 10)
        self.stop = self.create_publisher(Bool, "/sim/rpe/command/stop", 10)
        self.reset = self.create_client(Trigger, "/sim/rpe/reset")
        self.pause = self.create_client(SetBool, "/sim/rpe/pause")

    def _state(self, msg):
        self.latest = dict(zip(msg.name, msg.position))

    def _command(self, msg):
        self.latest_command = dict(zip(msg.name, msg.position))


def make_arm(values, duration_s=1):
    msg = JointTrajectory()
    msg.joint_names = list(ARM)
    point = JointTrajectoryPoint()
    msg.points.append(point)
    point.positions = list(values)
    point.time_from_start.sec = int(duration_s)
    return msg


def spin_for(node, duration_s):
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.05, deadline - time.monotonic()))


def wait_for(node, predicate, message, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError(message)


def call_service(node, client, request, label, timeout_s=5.0):
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise AssertionError(f"{label} service unavailable")
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise AssertionError(f"{label} service timed out")
    response = future.result()
    if response is None or not response.success:
        raise AssertionError(f"{label} service failed: {response}")
    return response


def max_delta(before, after, names):
    return max(abs(after[name] - before[name]) for name in names)


rclpy.init()
node = Probe()
try:
    wait_for(
        node,
        lambda: node.latest is not None and node.latest_command is not None,
        "native Isaac state or adapter command stream was not received",
        10.0,
    )
    wait_for(
        node,
        lambda: node.arm.get_subscription_count() > 0
        and node.hand.get_subscription_count() > 0
        and node.stop.get_subscription_count() > 0,
        "public command subscribers were not discovered",
        5.0,
    )
    before = dict(node.latest)

    # Start a deliberately longer arm/hand evolution so public stop can be
    # tested while targets are still changing. The hand adapter slews these
    # normalized targets before they reach the Isaac articulation.
    arm_msg = make_arm([0.40, -0.20, 0.35, 0.0, 0.15, 0.0], duration_s=4)
    hand_msg = Float32MultiArray(data=[0.0, 0.6, 0.6, 0.6, 0.6, 0.6])
    node.arm.publish(arm_msg)
    node.hand.publish(hand_msg)
    end = time.monotonic() + 1.1
    while time.monotonic() < end:
        base_msg = Twist()
        base_msg.linear.x = 0.20
        node.base.publish(base_msg)
        rclpy.spin_once(node, timeout_sec=0.05)
    after = dict(node.latest)
    arm_delta = max(abs(after[name] - before.get(name, after[name])) for name in ARM)
    hand_delta = abs(
        after["right_index_proximal_joint"]
        - before.get("right_index_proximal_joint", after["right_index_proximal_joint"])
    )
    if arm_delta < 0.03:
        raise AssertionError(f"arm command did not change native state (delta={arm_delta:g})")
    if hand_delta < 0.03:
        raise AssertionError(f"hand command did not change native state (delta={hand_delta:g})")
    if sys.argv[2] == "arm-hand":
        missing_mimic = set(HAND_MIMIC) - set(after)
        if missing_mimic:
            raise AssertionError(f"Revo2 mimic state is missing: {sorted(missing_mimic)}")
        mimic_delta = max(
            abs(after[name] - before.get(name, after[name])) for name in HAND_MIMIC
        )
        if mimic_delta < 0.02:
            raise AssertionError(
                f"Revo2 mimic joints did not follow the independent command "
                f"(delta={mimic_delta:g})"
            )
    if sys.argv[2] == "arm-hand":
        missing_fixed = set(BASE + ["lift_joint"]) - set(after)
        if missing_fixed:
            raise AssertionError(f"fixed joint state is missing: {sorted(missing_fixed)}")
        fixed_delta = max_delta(before, after, BASE + ["lift_joint"])
        if fixed_delta > 0.002:
            raise AssertionError(
                f"Ranger/lift moved during arm-hand physics (delta={fixed_delta:g})"
            )

    # Stop must freeze the adapter's evolving arm and hand command targets.
    node.stop.publish(Bool(data=True))
    spin_for(node, 0.25)
    stopped_before = dict(node.latest_command)
    spin_for(node, 0.60)
    stopped_after = dict(node.latest_command)
    stopped_delta = max_delta(stopped_before, stopped_after, ARM + HAND)
    if stopped_delta > 0.002:
        raise AssertionError(f"stop did not freeze command evolution (delta={stopped_delta:g})")

    # Pause must freeze target evolution even when a new command arrives;
    # resume must let that pending command evolve again.
    pause_request = SetBool.Request()
    pause_request.data = True
    call_service(node, node.pause, pause_request, "pause")
    paused_before = dict(node.latest_command)
    node.arm.publish(make_arm([-0.20, 0.10, 0.0, 0.0, -0.10, 0.0], duration_s=2))
    node.hand.publish(Float32MultiArray(data=[0.0] * 6))
    spin_for(node, 0.60)
    paused_after = dict(node.latest_command)
    paused_delta = max_delta(paused_before, paused_after, ARM + HAND)
    if paused_delta > 0.002:
        raise AssertionError(f"pause did not freeze command evolution (delta={paused_delta:g})")

    resume_request = SetBool.Request()
    resume_request.data = False
    call_service(node, node.pause, resume_request, "resume")
    resumed_before = dict(node.latest_command)
    spin_for(node, 0.80)
    resumed_after = dict(node.latest_command)
    resumed_delta = max_delta(resumed_before, resumed_after, ARM + HAND)
    if resumed_delta < 0.03:
        raise AssertionError(f"resume did not restart command evolution (delta={resumed_delta:g})")

    call_service(node, node.reset, Trigger.Request(), "reset")
    wait_for(
        node,
        lambda: max(abs(node.latest_command[name]) for name in ARM + HAND) < 0.02,
        "reset did not return adapter arm/hand commands near zero",
        5.0,
    )
    if sys.argv[2] == "arm-hand":
        # Let gravity and the drives settle after reset. This catches finite but
        # rapidly diverging articulations that a single response sample misses.
        stable_start = dict(node.latest)
        spin_for(node, 5.0)
        stable_end = dict(node.latest)
        if not all(math.isfinite(value) for value in stable_end.values()):
            raise AssertionError("non-finite joint state after gravity settling")
        fixed_delta = max_delta(stable_start, stable_end, BASE + ["lift_joint"])
        if fixed_delta > 0.002:
            raise AssertionError(
                f"fixed Ranger/lift drifted after reset (delta={fixed_delta:g})"
            )
    wait_for(
        node,
        lambda: max(abs(node.latest[name]) for name in ARM) < 0.08
        and abs(node.latest["right_index_proximal_joint"]) < 0.08,
        "reset did not return native arm/hand state near zero",
        5.0,
    )

    topics = [name for name, _types in node.get_topic_names_and_types()]
    forbidden = {"/cmd_vel", "/joint_states", "/revo2/command_positions", "/safety/motion_enable"}
    leaked = forbidden.intersection(topics)
    if leaked:
        raise AssertionError(f"physical/root topic leaked into visual graph: {leaked}")
finally:
    node.destroy_node()
    rclpy.shutdown()

report_path = pathlib.Path(sys.argv[1])
physics_mode = sys.argv[2]
scene_report_path = pathlib.Path(sys.argv[3])
spawn_height_m = float(sys.argv[4])
deadline = time.monotonic() + 30.0
while not report_path.is_file() and time.monotonic() < deadline:
    time.sleep(0.1)
report = json.loads(report_path.read_text())
assert report.get("status") == "PASS", report
assert report.get("native_ros2_bridge") is True, report
assert report.get("namespace") == "/sim/rpe", report
assert report.get("hardware_access") is False, report
if physics_mode == "visual":
    assert report.get("max_base_displacement_m", 0.0) > 0.03, report
else:
    assert report.get("max_base_displacement_m", 0.0) < 1.0e-9, report
assert report.get("reset_count", 0) >= 1, report
assert report.get("physics_profile") == physics_mode, report
expected_mode = (
    "fixed-base arm/hand gravity and contact"
    if physics_mode == "arm-hand"
    else "zero-gravity articulation with kinematic base visualization"
)
assert report.get("physics_mode") == expected_mode, report
expected_physics = physics_mode == "arm-hand"
assert abs(report.get("gravity_magnitude_m_s2", -1.0) - (9.81 if expected_physics else 0.0)) < 1.0e-6, report
assert report.get("ground_collision_enabled") is expected_physics, report
assert report.get("contact_dynamics") is expected_physics, report
scene_report = json.loads(scene_report_path.read_text())
assert scene_report.get("status") == "PASS", scene_report
assert scene_report.get("physics_mode") == physics_mode, scene_report
translation = scene_report.get("spawn_translation", [])
assert len(translation) == 3 and abs(translation[2] - spawn_height_m) < 1.0e-6, scene_report
drives = scene_report.get("drive_configuration", {})
if expected_physics:
    assert drives.get("base_joints_locked") == 8, drives
    assert drives.get("base_root_fixed") == 1, drives
    material = scene_report.get("scene_configuration", {}).get("ground_material")
    assert material == {
        "path": "/World/PhysicsMaterials/Ground",
        "static_friction": 0.9,
        "dynamic_friction": 0.8,
        "restitution": 0.0,
    }, material
print("R2 Isaac visual ROS probe: PASS")
PY

wait "$ISAAC_PID"

if grep -Eq 'Invalid PhysX transform|Nested articulation roots|Traceback|\[Error\]' "$ISAAC_LOG"; then
  echo "FAIL: Isaac visual log contains a forbidden runtime error" >&2
  grep -En 'Invalid PhysX transform|Nested articulation roots|Traceback|\[Error\]' "$ISAAC_LOG" >&2 || true
  exit 1
fi
if grep -Eq 'Traceback|\[ERROR\]' "$ADAPTER_LOG"; then
  echo "FAIL: Isaac ROS adapter log contains a runtime error" >&2
  grep -En 'Traceback|\[ERROR\]' "$ADAPTER_LOG" >&2 || true
  exit 1
fi

echo "R2 Isaac visual command/state/stop/pause/reset smoke: PASS"
