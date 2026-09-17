#!/usr/bin/env bash
set -euo pipefail

# R2 is simulation-only.  This launcher starts the namespaced ROS contract
# bridge and (optionally) the Isaac Sim 6.1 scene importer.  It deliberately
# does not source or invoke any vendor bring-up file.
CODEBASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${RPE_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
ISAAC_PYTHON="${ISAAC_PYTHON:-/opt/isaaclab/bin/python}"

if [[ -f "$ROS_SETUP" ]]; then
  # ROS setup scripts use variables that may be unset.  Do not evaluate them
  # while nounset is active (this is the source of the AMENT_TRACE error).
  set +u
  source "$ROS_SETUP"
  set -u
fi

export PYTHONPATH="$CODEBASE_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
BRIDGE_PID=""
cleanup() {
  if [[ -n "$BRIDGE_PID" ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
    kill "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "R2 simulation namespace: /sim/rpe"
echo "Hardware drivers are not started; hardware command topics remain untouched."
"$PYTHON_BIN" -m rpe_robot.sim.bridge &
BRIDGE_PID=$!

if [[ "${RPE_SKIP_ISAAC:-0}" == "1" ]]; then
  echo "Isaac scene skipped (RPE_SKIP_ISAAC=1); ROS contract bridge is running."
  wait "$BRIDGE_PID"
  exit $?
fi

if [[ ! -x "$ISAAC_PYTHON" ]]; then
  echo "Isaac interpreter not found at $ISAAC_PYTHON" >&2
  echo "Set RPE_SKIP_ISAAC=1 to run only the ROS contract bridge." >&2
  exit 2
fi

ISAAC_ARGS=(
  "$CODEBASE_ROOT/src/rpe_robot/sim/isaac_scene.py"
  --urdf "${RPE_DESCRIPTION:-/workspace/assets/combined_robot_ur5e_single_arm/urdf/total_robot.urdf}"
  --output-usd "${RPE_R2_USD:-/tmp/rpe_r2/combined_robot_ur5e_single_arm.usd}"
  --report "${RPE_R2_ISAAC_REPORT:-/tmp/rpe_r2/isaac_scene_report.json}"
)
if [[ "${RPE_R2_WINDOW:-0}" != "1" ]]; then
  ISAAC_ARGS+=(--steps "${RPE_R2_STEPS:-10}")
else
  ISAAC_ARGS+=(--window --steps "${RPE_R2_STEPS:-10}")
fi

"$ISAAC_PYTHON" "${ISAAC_ARGS[@]}"
echo "Isaac scene completed; ROS bridge remains available (Ctrl-C to stop)."
wait "$BRIDGE_PID"
