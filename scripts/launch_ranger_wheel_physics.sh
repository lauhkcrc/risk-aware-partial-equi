#!/usr/bin/env bash
set -euo pipefail

# Interactive/headless simulation-only Ranger wheel-contact profile.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RPE_PHYSICS_MODE=ranger-wheel
export RPE_SPAWN_HEIGHT_M="${RPE_SPAWN_HEIGHT_M:-0.324}"
export RPE_R2_USD="${RPE_R2_USD:-/tmp/rpe_ranger_wheel/combined_robot.usd}"
export RPE_R2_ISAAC_REPORT="${RPE_R2_ISAAC_REPORT:-/tmp/rpe_ranger_wheel/scene_report.json}"
export RPE_R2_VISUAL_REPORT="${RPE_R2_VISUAL_REPORT:-/tmp/rpe_ranger_wheel/runtime_report.json}"
export RPE_R2_RESET_FILE="${RPE_R2_RESET_FILE:-/tmp/rpe_ranger_wheel/reset.request}"
export RPE_RANGER_WHEEL_PHASE_DIR="${RPE_RANGER_WHEEL_PHASE_DIR:-/tmp/rpe_ranger_wheel/phases}"
export RPE_RANGER_WHEEL_TASK_REPORT="${RPE_RANGER_WHEEL_TASK_REPORT:-/tmp/rpe_ranger_wheel/task_report.json}"
export RPE_RANGER_WHEEL_DONE_FILE="${RPE_RANGER_WHEEL_DONE_FILE:-/tmp/rpe_ranger_wheel/done}"
export RPE_RANGER_WHEEL_TASK=1
export RPE_R2_VISUAL_DURATION="${RPE_R2_VISUAL_DURATION:-20}"
exec "$SCRIPT_DIR/launch_r2_visual.sh" "$@"
