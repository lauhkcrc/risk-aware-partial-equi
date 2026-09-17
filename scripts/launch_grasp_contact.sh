#!/usr/bin/env bash
set -euo pipefail

# Simulation-only thumb/index pinch calibration with a passive cylinder.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RPE_PHYSICS_MODE=grasp-contact
export RPE_SPAWN_HEIGHT_M="${RPE_SPAWN_HEIGHT_M:-0.324}"
export RPE_R2_USD="${RPE_R2_USD:-/tmp/rpe_grasp_contact/combined_robot.usd}"
export RPE_R2_ISAAC_REPORT="${RPE_R2_ISAAC_REPORT:-/tmp/rpe_grasp_contact/scene_report.json}"
export RPE_R2_VISUAL_REPORT="${RPE_R2_VISUAL_REPORT:-/tmp/rpe_grasp_contact/runtime_report.json}"
export RPE_R2_RESET_FILE="${RPE_R2_RESET_FILE:-/tmp/rpe_grasp_contact/reset.request}"
export RPE_GRASP_RELEASE_FILE="${RPE_GRASP_RELEASE_FILE:-/tmp/rpe_grasp_contact/release.request}"
export RPE_GRASP_OPEN_FILE="${RPE_GRASP_OPEN_FILE:-/tmp/rpe_grasp_contact/open.request}"
export RPE_GRASP_DONE_FILE="${RPE_GRASP_DONE_FILE:-/tmp/rpe_grasp_contact/done}"
export RPE_GRASP_TASK_REPORT="${RPE_GRASP_TASK_REPORT:-/tmp/rpe_grasp_contact/task_report.json}"
export RPE_GRASP_TASK=1
exec "$SCRIPT_DIR/launch_r2_visual.sh" "$@"
