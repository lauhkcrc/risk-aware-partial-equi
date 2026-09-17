#!/usr/bin/env bash
set -euo pipefail

# Simulation-only ROS-to-Isaac validation for fixed-base arm/hand physics.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RPE_PHYSICS_MODE=arm-hand
export RPE_SPAWN_HEIGHT_M="${RPE_SPAWN_HEIGHT_M:-0.324}"
export RPE_R2_USD="${RPE_R2_USD:-/tmp/rpe_arm_hand_physics/combined_robot.usd}"
export RPE_R2_ISAAC_REPORT="${RPE_R2_ISAAC_REPORT:-/tmp/rpe_arm_hand_physics/scene_smoke_report.json}"
export RPE_R2_VISUAL_REPORT="${RPE_R2_VISUAL_REPORT:-/tmp/rpe_arm_hand_physics/runtime_smoke_report.json}"
export RPE_R2_RESET_FILE="${RPE_R2_RESET_FILE:-/tmp/rpe_arm_hand_physics/reset.request}"
export RPE_R2_VISUAL_LOG="${RPE_R2_VISUAL_LOG:-/tmp/rpe_arm_hand_physics/runtime_smoke.log}"
export RPE_R2_ADAPTER_LOG="${RPE_R2_ADAPTER_LOG:-/tmp/rpe_arm_hand_physics/adapter_smoke.log}"
exec "$SCRIPT_DIR/test_r2_isaac_visual.sh" "$@"
