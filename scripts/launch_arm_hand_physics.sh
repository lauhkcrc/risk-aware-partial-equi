#!/usr/bin/env bash
set -euo pipefail

# Stage-one physics: fixed Ranger/lift, gravity/contact, dynamic UR5e/Revo2.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RPE_PHYSICS_MODE=arm-hand
export RPE_SPAWN_HEIGHT_M="${RPE_SPAWN_HEIGHT_M:-0.324}"
export RPE_R2_USD="${RPE_R2_USD:-/tmp/rpe_arm_hand_physics/combined_robot.usd}"
export RPE_R2_ISAAC_REPORT="${RPE_R2_ISAAC_REPORT:-/tmp/rpe_arm_hand_physics/scene_report.json}"
export RPE_R2_VISUAL_REPORT="${RPE_R2_VISUAL_REPORT:-/tmp/rpe_arm_hand_physics/runtime_report.json}"
export RPE_R2_RESET_FILE="${RPE_R2_RESET_FILE:-/tmp/rpe_arm_hand_physics/reset.request}"
exec "$SCRIPT_DIR/launch_r2_visual.sh" "$@"
