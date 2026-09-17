#!/usr/bin/env bash
set -euo pipefail

# End-to-end ROS-to-Isaac contact, retention, and release regression.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RPE_PHYSICS_MODE=grasp-contact
export RPE_SPAWN_HEIGHT_M="${RPE_SPAWN_HEIGHT_M:-0.324}"
export RPE_R2_VISUAL_DURATION="${RPE_R2_VISUAL_DURATION:-14}"
export RPE_R2_USD="${RPE_R2_USD:-/tmp/rpe_grasp_contact/combined_robot.usd}"
export RPE_R2_ISAAC_REPORT="${RPE_R2_ISAAC_REPORT:-/tmp/rpe_grasp_contact/scene_smoke_report.json}"
export RPE_R2_VISUAL_REPORT="${RPE_R2_VISUAL_REPORT:-/tmp/rpe_grasp_contact/runtime_smoke_report.json}"
export RPE_R2_RESET_FILE="${RPE_R2_RESET_FILE:-/tmp/rpe_grasp_contact/reset.request}"
export RPE_GRASP_RELEASE_FILE="${RPE_GRASP_RELEASE_FILE:-/tmp/rpe_grasp_contact/release.request}"
export RPE_GRASP_OPEN_FILE="${RPE_GRASP_OPEN_FILE:-/tmp/rpe_grasp_contact/open.request}"
export RPE_GRASP_DONE_FILE="${RPE_GRASP_DONE_FILE:-/tmp/rpe_grasp_contact/done}"
export RPE_GRASP_TASK_REPORT="${RPE_GRASP_TASK_REPORT:-/tmp/rpe_grasp_contact/task_smoke_report.json}"
export RPE_GRASP_RUNTIME_LOG="${RPE_GRASP_RUNTIME_LOG:-/tmp/rpe_grasp_contact/runtime_smoke.log}"
export RPE_GRASP_TASK=1

rm -f "$RPE_GRASP_RUNTIME_LOG"
"$SCRIPT_DIR/launch_r2_visual.sh" 2>&1 | tee "$RPE_GRASP_RUNTIME_LOG"

"${ROS_PYTHON:-/usr/bin/python3}" - \
  "$RPE_R2_VISUAL_REPORT" "$RPE_R2_ISAAC_REPORT" "$RPE_GRASP_TASK_REPORT" \
  "$RPE_GRASP_RUNTIME_LOG" <<'PY'
import json
import math
import pathlib
import sys

runtime = json.loads(pathlib.Path(sys.argv[1]).read_text())
scene = json.loads(pathlib.Path(sys.argv[2]).read_text())
task = json.loads(pathlib.Path(sys.argv[3]).read_text())
runtime_log = pathlib.Path(sys.argv[4]).read_text(errors="replace")
for forbidden in ("[Error]", "Invalid PhysX transform", "Traceback", "NaN"):
    assert forbidden not in runtime_log, (forbidden, runtime_log[-4000:])
assert runtime.get("status") == "PASS", runtime
assert runtime.get("physics_profile") == "grasp-contact", runtime
assert runtime.get("hardware_access") is False, runtime
grasp = runtime.get("grasp_contact") or {}
assert grasp.get("shape") == "cylinder", grasp
assert abs(grasp.get("radius_m", 0.0) - 0.012) < 1.0e-9, grasp
assert abs(grasp.get("height_m", 0.0) - 0.042) < 1.0e-9, grasp
assert grasp.get("released") is True, grasp
assert grasp.get("hand_opened") is True, grasp
assert grasp.get("release_captured") is True, grasp
assert grasp.get("contact_report_enabled") is False, grasp
assert "dynamic retention" in grasp.get("contact_measurement_policy", ""), grasp
assert grasp.get("self_collision_enabled") is False, grasp
assert math.isfinite(grasp.get("max_hold_displacement_m", math.nan)), grasp
assert grasp.get("max_hold_displacement_m", math.inf) < 0.010, grasp
assert 0.10 <= grasp.get("post_open_displacement_m", 0.0) < 0.30, grasp
assert 0.10 <= grasp.get("post_open_vertical_drop_m", 0.0) < 0.30, grasp
surfaces = grasp.get("release_collision_surfaces") or {}
assert 0.035 < surfaces.get("surface_gap_m", 0.0) < 0.050, surfaces
for pad in ("thumb", "index"):
    sample = surfaces.get(pad) or {}
    assert sample.get("mesh_triangles", 0) > 0, sample
    assert math.isfinite(sample.get("distance_to_object_center_m", math.nan)), sample
assert task.get("status") == "PASS", task
assert task.get("hardware_access") is False, task
initial = task["initial_state_rad"]
pinch = task["pinch_state_rad"]
hold = task["hold_state_rad"]
opened = task["open_state_rad"]
active = [
    "right_thumb_metacarpal_joint",
    "right_thumb_proximal_joint",
    "right_index_proximal_joint",
]
inactive = [
    "right_middle_proximal_joint",
    "right_ring_proximal_joint",
    "right_pinky_proximal_joint",
]
assert min(abs(pinch[name] - initial[name]) for name in active) > 0.10, task
assert max(abs(pinch[name]) for name in inactive) < 0.05, task
assert abs(pinch["right_thumb_distal_joint"] - initial["right_thumb_distal_joint"]) > 0.10, task
assert abs(pinch["right_index_distal_joint"] - initial["right_index_distal_joint"]) > 0.10, task
assert max(abs(hold[name] - pinch[name]) for name in active) < 0.10, task
assert max(abs(opened[name]) for name in active) < 0.08, task
configuration = scene.get("grasp_contact_configuration") or {}
assert configuration.get("collision_shapes", 0) >= 17, configuration
assert configuration.get("convex_hull_collision_shapes", 0) >= 17, configuration
assert configuration.get("collision_approximation") == (
    "convexHull for Revo2 mesh colliders"
), configuration
assert configuration.get("rigid_contact_bodies", 0) >= 12, configuration
assert configuration.get("contact_report_policy") == (
    "disabled; validate contact through dynamic retention and release"
), configuration
assert configuration.get("self_collision_enabled") is False, configuration
assert configuration.get("material") == {
    "path": "/World/PhysicsMaterials/Revo2Pads",
    "static_friction": 1.2,
    "dynamic_friction": 1.0,
    "restitution": 0.0,
}, configuration
assert scene.get("hardware_access") is False, scene
scene_configuration = scene.get("scene_configuration") or {}
assert scene_configuration.get("ground_collision_enabled") is True, scene_configuration
assert abs(scene_configuration.get("ground_thickness_m", 0.0) - 0.05) < 1.0e-9, scene_configuration
print("R2 grasp/contact ROS pinch, retention, and release: PASS")
PY
