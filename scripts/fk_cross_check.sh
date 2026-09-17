#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP="${RPE_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [[ -f "$ROS_SETUP" ]]; then
  set +u
  source "$ROS_SETUP"
  set -u
else
  set -u
fi

# Synthetic ROS-vs-independent FK test.  This launches only a private local
# robot_state_publisher and never contacts hardware.
CODEBASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSET_ROOT="${RPE_ASSET_ROOT:-/workspace/assets}"
DESCRIPTION="${RPE_DESCRIPTION:-$ASSET_ROOT/combined_robot_ur5e_single_arm}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
SAMPLES="${RPE_FK_SAMPLES:-1000}"
exec "$PYTHON_BIN" "$CODEBASE_ROOT/src/rpe_robot/audit/fk_cross_check.py" \
  --urdf "$DESCRIPTION/urdf/total_robot.urdf" \
  --samples "$SAMPLES" \
  --json-out "$CODEBASE_ROOT/reports/fk_jacobian_check.json"
