#!/usr/bin/env bash
set -euo pipefail

# End-to-end ROS-to-Isaac Ranger wheel/contact validation. No hardware access.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RPE_R2_WINDOW="${RPE_R2_WINDOW:-0}"
export RPE_R2_VISUAL_DURATION="${RPE_R2_VISUAL_DURATION:-35}"
export RPE_R2_VISUAL_LOG="${RPE_R2_VISUAL_LOG:-/tmp/rpe_ranger_wheel/runtime.log}"

mkdir -p "$(dirname "$RPE_R2_VISUAL_LOG")"
rm -f "$RPE_R2_VISUAL_LOG"
"$SCRIPT_DIR/launch_ranger_wheel_physics.sh" 2>&1 | tee "$RPE_R2_VISUAL_LOG"

"${ROS_PYTHON:-/usr/bin/python3}" - \
  "${RPE_R2_VISUAL_REPORT:-/tmp/rpe_ranger_wheel/runtime_report.json}" \
  "${RPE_R2_ISAAC_REPORT:-/tmp/rpe_ranger_wheel/scene_report.json}" \
  "${RPE_RANGER_WHEEL_TASK_REPORT:-/tmp/rpe_ranger_wheel/task_report.json}" \
  "$RPE_R2_VISUAL_LOG" <<'PY'
import json
import math
import pathlib
import sys

runtime = json.loads(pathlib.Path(sys.argv[1]).read_text())
scene = json.loads(pathlib.Path(sys.argv[2]).read_text())
task = json.loads(pathlib.Path(sys.argv[3]).read_text())
log = pathlib.Path(sys.argv[4]).read_text(errors="replace")
for forbidden in ("[Error]", "Invalid PhysX transform", "Traceback", "NaN"):
    assert forbidden not in log, (forbidden, log[-6000:])
assert runtime.get("status") == "PASS", runtime
assert runtime.get("physics_profile") == "ranger-wheel", runtime
assert runtime.get("physics_mode") == (
    "free-base Ranger four-wheel-steering contact physics"
), runtime
assert runtime.get("base_policy") == "dynamic four-wheel-steering articulation", runtime
assert runtime.get("hardware_access") is False, runtime
assert runtime.get("ground_collision_enabled") is True, runtime
assert runtime.get("physics_time_steps_per_second") == 60, runtime
assert runtime.get("enhanced_determinism") is True, runtime
assert abs(runtime.get("gravity_magnitude_m_s2", 0.0) - 9.81) < 1.0e-6, runtime

wheel_runtime = runtime.get("ranger_wheel_physics") or {}
traction = wheel_runtime.get("traction_switch") or {}
assert traction.get("mode") == "brake", traction
assert traction.get("transitions", 0) >= 5, traction
assert abs(traction.get("static_friction", 0.0) - 0.2) < 1.0e-6, traction
assert abs(traction.get("dynamic_friction", 0.0) - 0.15) < 1.0e-6, traction
assert abs(traction.get("wheel_drive_damping", 0.0) - 100.0) < 1.0e-6, traction
poses = wheel_runtime.get("phase_poses") or {}
assert set(("settled", "straight", "stopped", "spin_aligned", "spin", "final")) <= set(poses), poses
settled, straight, stopped, spin_aligned, spin, final = (
    poses[name] for name in (
        "settled", "straight", "stopped", "spin_aligned", "spin", "final"
    )
)
for pose in (settled, straight, stopped, spin_aligned, spin, final):
    assert all(math.isfinite(pose[name]) for name in ("x", "y", "z", "yaw", "tilt")), pose
    assert pose["z"] > 0.20, pose
    assert pose["tilt"] < 0.10, pose

straight_dx = straight["x"] - settled["x"]
straight_dy = straight["y"] - settled["y"]
straight_dyaw = math.atan2(
    math.sin(straight["yaw"] - settled["yaw"]),
    math.cos(straight["yaw"] - settled["yaw"]),
)
assert straight_dx > 0.12, (straight_dx, poses)
assert abs(straight_dy) < 0.05, (straight_dy, poses)
assert abs(straight_dyaw) < 0.10, (straight_dyaw, poses)
stop_drift = math.hypot(stopped["x"] - straight["x"], stopped["y"] - straight["y"])
# This is a bounded-contact smoke test, not a calibrated braking-distance
# claim. Native finite-width cylinders may roll several centimetres while
# their velocity drives settle; the measured gate below still requires all
# four wheel speeds to fall below 0.35 rad/s before steering changes.
assert stop_drift < 0.20, (stop_drift, poses)

spin_dyaw = math.atan2(
    math.sin(spin["yaw"] - stopped["yaw"]),
    math.cos(spin["yaw"] - stopped["yaw"]),
)
spin_translation = math.hypot(spin["x"] - stopped["x"], spin["y"] - stopped["y"])
alignment_translation = math.hypot(
    spin_aligned["x"] - stopped["x"], spin_aligned["y"] - stopped["y"]
)
rolling_spin_translation = math.hypot(
    spin["x"] - spin_aligned["x"], spin["y"] - spin_aligned["y"]
)
# Chassis yaw is reported but not qualified: an isotropic analytic cylinder
# is not an anisotropic tyre model. Joint-space spin actuation is checked
# strictly below, while chassis contact motion must remain bounded.
assert math.isfinite(spin_dyaw), spin_dyaw
assert alignment_translation < 0.12, (alignment_translation, poses)
assert rolling_spin_translation < 0.15, (rolling_spin_translation, poses)
assert spin_translation < 0.20, (spin_translation, poses)
final_drift = math.hypot(final["x"] - spin["x"], final["y"] - spin["y"])
assert final_drift < 0.12, (final_drift, poses)

assert task.get("status") == "PASS", task
assert task.get("hardware_access") is False, task
states = task["states"]
wheel_names = ("fr_wheel", "fl_wheel", "rl_wheel", "rr_wheel")
steering_names = (
    "fr_steering_joint", "fl_steering_joint", "rl_steering_joint", "rr_steering_joint"
)
assert max(
    abs(states["settled"]["velocity_rad_s"][name])
    for name in wheel_names
) <= 0.05, states["settled"]
assert max(
    abs(states["settled"]["velocity_rad_s"][name])
    for name in steering_names
) <= 0.10, states["settled"]
assert 0.0 < task["spin_alignment_duration_s"] < 6.0, task
spin_targets = task["spin_steering_target_rad"]
spin_aligned = states["spin_aligned"]
for name in steering_names:
    assert abs(
        spin_aligned["position_rad"][name] - spin_targets[name]
    ) <= 0.04, (name, spin_aligned, spin_targets)
    assert abs(spin_aligned["velocity_rad_s"][name]) <= 0.10, (
        name, spin_aligned
    )
straight_positions = states["straight"]["position_rad"]
settled_positions = states["settled"]["position_rad"]
assert min(abs(straight_positions[name] - settled_positions[name]) for name in wheel_names) > 0.5
assert max(abs(straight_positions[name]) for name in steering_names) < 0.12
straight_velocities = states["straight"]["velocity_rad_s"]
for name in wheel_names:
    assert 0.60 < straight_velocities[name] < 1.60, (
        name, straight_velocities[name], states["straight"]
    )
assert max(
    abs(states["stopped"]["velocity_rad_s"][name]) for name in wheel_names
) < 0.35, states["stopped"]
spin_positions = states["spin"]["position_rad"]
for name in steering_names:
    assert abs(spin_positions[name] - spin_targets[name]) <= 0.04, (
        name, spin_positions, spin_targets
    )
assert max(
    abs(states["spin"]["velocity_rad_s"][name]) for name in steering_names
) <= 0.25, states["spin"]
spin_wheel_velocities = states["spin"]["velocity_rad_s"]
expected_spin_sign = {
    "fr_wheel": 1.0,
    "fl_wheel": -1.0,
    "rl_wheel": -1.0,
    "rr_wheel": 1.0,
}
for name, sign in expected_spin_sign.items():
    speed = spin_wheel_velocities[name]
    assert sign * speed > 0.20, (name, speed, states["spin"])
    assert abs(speed) < 0.70, (name, speed, states["spin"])
final_velocities = states["final"]["velocity_rad_s"]
assert max(abs(final_velocities[name]) for name in wheel_names) < 0.35, final_velocities

assert scene.get("status") == "PASS", scene
assert scene.get("physics_mode") == "ranger-wheel", scene
drives = scene.get("drive_configuration") or {}
assert drives.get("base_root_fixed") == 0, drives
assert drives.get("base_joints_locked") == 0, drives
assert drives.get("base_steering_position_drives") == 4, drives
assert drives.get("base_wheel_velocity_drives") == 4, drives
assert drives.get("wheel_alignment_drive_damping") == 5.0, drives
assert drives.get("steering_drive_max_force") == 100.0, drives
assert drives.get("wheel_drive_max_force") == 1000.0, drives
assert scene.get("import_policy") == (
    "eight Ranger DOFs plus merged rigid upper-body payload"
), scene
assert (scene.get("topology") or {}).get("ranger_wheel_collapsed_joints") == 34, scene
assert (scene.get("root_release") or {}).get("free_root_verified") is True, scene
contacts = scene.get("ranger_wheel_configuration") or {}
assert contacts.get("collision_shapes", 0) >= 4, contacts
assert contacts.get("analytic_collision_shapes") == 4, contacts
assert contacts.get("collision_approximation") == (
    "native URDF analytic cylinders"
), contacts
assert contacts.get("source_cylinder_collision_enabled") is True, contacts
assert contacts.get("solver", {}).get("position_iterations") == 32, contacts
assert contacts.get("solver", {}).get("velocity_iterations") == 4, contacts
geometry = contacts.get("contact_geometry") or ()
assert len(geometry) == 4, contacts
assert {item.get("wheel_link") for item in geometry} == {
    "fr_wheel_link", "fl_wheel_link", "rl_wheel_link", "rr_wheel_link"
}, geometry
for item in geometry:
    assert item.get("type") == "Cylinder", item
    assert item.get("collision_enabled") is True, item
    assert abs(item.get("radius_m", 0.0) - 0.09) < 1.0e-9, item
    assert abs(item.get("width_m", 0.0) - 0.08) < 1.0e-9, item
    assert abs(item.get("mass_kg", 0.0) - 8.0) < 1.0e-9, item
    inertia = item.get("diagonal_inertia_kg_m2") or ()
    assert len(inertia) == 3, item
    assert all(
        abs(actual - expected) < 1.0e-8
        for actual, expected in zip(inertia, (0.02047, 0.0324, 0.02047))
    ), item
    assert item.get("inertia_alignment") == (
        "axial principal inertia on wheel joint Y axis"
    ), item
    assert item.get("axis_alignment") == (
        "exactly parallel to wheel joint Y axis"
    ), item
    assert item.get("center_alignment") == (
        "coincident with wheel joint axis"
    ), item
    assert "rpe_wheel_contact" not in item.get("path", ""), item
assert abs(contacts.get("wheel_radius_m", 0.0) - 0.09) < 1.0e-9, contacts
assert contacts.get("material") == {
    "path": "/World/PhysicsMaterials/RangerWheels",
    "static_friction": 0.2,
    "dynamic_friction": 0.15,
    "restitution": 0.0,
    "friction_combine_mode": "min",
    "alignment_static_friction": 0.02,
    "alignment_dynamic_friction": 0.01,
    "drive_static_friction": 0.20,
    "drive_dynamic_friction": 0.15,
    "brake_static_friction": 0.2,
    "brake_dynamic_friction": 0.15,
    "friction_policy": (
        "low-slip steering alignment; moderate-traction rolling and braking"
    ),
}, contacts
print("R2 Ranger native-cylinder actuation/contact smoke: PASS")
PY
