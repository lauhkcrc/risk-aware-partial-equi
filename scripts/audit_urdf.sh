#!/usr/bin/env bash
set -eo pipefail

# A ROS setup file may reference unset variables.  Source it before enabling
# nounset so this wrapper also works when launched with `docker exec` rather
# than through the image entrypoint.
ROS_SETUP="${RPE_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [[ -f "$ROS_SETUP" ]]; then
  set +u
  source "$ROS_SETUP"
  set -u
else
  set -u
fi

# Read-only R1 description audit.  The default paths are the repository mount
# used by the thor container; override RPE_ASSET_ROOT for another checkout.
CODEBASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSET_ROOT="${RPE_ASSET_ROOT:-/workspace/assets}"
DESCRIPTION="${RPE_DESCRIPTION:-$ASSET_ROOT/combined_robot_ur5e_single_arm}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"

URDF="$DESCRIPTION/urdf/total_robot.urdf"
XACRO="$DESCRIPTION/urdf/total_robot.xacro"
[[ -f "$URDF" ]] || { echo "URDF not found: $URDF" >&2; exit 2; }
[[ -f "$XACRO" ]] || { echo "Xacro not found: $XACRO" >&2; exit 2; }

CHECK_URDF_BIN="${CHECK_URDF_BIN:-$(command -v check_urdf || true)}"
[[ -n "$CHECK_URDF_BIN" ]] || {
  echo "check_urdf was not found; source ROS or set CHECK_URDF_BIN" >&2
  exit 2
}

# Keep the upstream parser as an explicit R1 gate.  The Python audit below
# adds the structural, mesh, inertial, and Xacro checks and writes the report.
"$CHECK_URDF_BIN" "$URDF"

exec "$PYTHON_BIN" "$CODEBASE_ROOT/src/rpe_robot/audit/urdf_audit.py" \
  --urdf "$URDF" \
  --xacro "$XACRO" \
  --json-out "$CODEBASE_ROOT/reports/urdf_audit.json" \
  --markdown-out "$CODEBASE_ROOT/reports/urdf_audit.md"
