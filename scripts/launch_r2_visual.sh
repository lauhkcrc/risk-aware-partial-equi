#!/usr/bin/env bash
set -euo pipefail

# R2-visual: Isaac Sim only.  This wrapper never starts the deterministic
# contract bridge or any physical/vendor process.  The ROS adapter is a
# namespaced Python 3.12 process; Isaac itself uses the native ROS bridge.
CODEBASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${RPE_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
ROS_PYTHON="${ROS_PYTHON:-/usr/bin/python3}"
ISAAC_PYTHON="${ISAAC_PYTHON:-/opt/isaaclab/bin/python}"
URDF="${RPE_DESCRIPTION:-/workspace/assets/combined_robot_ur5e_single_arm/urdf/total_robot.urdf}"
USD="${RPE_R2_USD:-/tmp/rpe_r2/combined_robot_ur5e_single_arm.usd}"
IMPORT_REPORT="${RPE_R2_ISAAC_REPORT:-/tmp/rpe_r2/isaac_scene_report.json}"
VISUAL_REPORT="${RPE_R2_VISUAL_REPORT:-/tmp/rpe_r2/isaac_visual_report.json}"
RESET_FILE="${RPE_R2_RESET_FILE:-/tmp/rpe_r2/reset.request}"
GRASP_RELEASE_FILE="${RPE_GRASP_RELEASE_FILE:-/tmp/rpe_grasp_contact/release.request}"
GRASP_OPEN_FILE="${RPE_GRASP_OPEN_FILE:-/tmp/rpe_grasp_contact/open.request}"
RANGER_PHASE_DIR="${RPE_RANGER_WHEEL_PHASE_DIR:-/tmp/rpe_ranger_wheel/phases}"
RANGER_TASK_REPORT="${RPE_RANGER_WHEEL_TASK_REPORT:-/tmp/rpe_ranger_wheel/task_report.json}"
RANGER_DONE_FILE="${RPE_RANGER_WHEEL_DONE_FILE:-/tmp/rpe_ranger_wheel/done}"
PHYSICS_MODE="${RPE_PHYSICS_MODE:-visual}"
SPAWN_HEIGHT_M="${RPE_SPAWN_HEIGHT_M:-0.324}"

if [[ -f "$ROS_SETUP" ]]; then
  # Jazzy setup uses variables that may be unset.  Evaluate it with nounset
  # disabled, then restore the launcher's strict shell mode.
  set +u
  source "$ROS_SETUP"
  set -u
fi
export PYTHONPATH="$CODEBASE_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"

if [[ ! -x "$ISAAC_PYTHON" ]]; then
  echo "Isaac interpreter not found at $ISAAC_PYTHON" >&2
  exit 2
fi
if [[ ! -x "$ROS_PYTHON" ]]; then
  echo "ROS Python interpreter not found at $ROS_PYTHON" >&2
  exit 2
fi
if [[ ! -f "$URDF" ]]; then
  echo "canonical URDF not found at $URDF" >&2
  exit 2
fi

mkdir -p "$(dirname "$USD")" "$(dirname "$RESET_FILE")"
rm -f "$RESET_FILE" "$GRASP_RELEASE_FILE" "$GRASP_OPEN_FILE" "$VISUAL_REPORT"
mkdir -p "$RANGER_PHASE_DIR"
rm -f "$RANGER_PHASE_DIR/settled" "$RANGER_PHASE_DIR/straight" \
  "$RANGER_PHASE_DIR/stopped" "$RANGER_PHASE_DIR/spin_aligned" \
  "$RANGER_PHASE_DIR/spin" \
  "$RANGER_PHASE_DIR/final" "$RANGER_TASK_REPORT" "$RANGER_DONE_FILE"

if [[ "${RPE_SKIP_IMPORT:-0}" != "1" ]]; then
  rm -f "$IMPORT_REPORT" "$USD"
  echo "Preparing flattened Isaac USD from canonical URDF..."
  "$ISAAC_PYTHON" "$CODEBASE_ROOT/src/rpe_robot/sim/isaac_scene.py" \
    --urdf "$URDF" --output-usd "$USD" --report "$IMPORT_REPORT" \
    --steps "${RPE_R2_STEPS:-10}" --physics-mode "$PHYSICS_MODE" \
    --spawn-height "$SPAWN_HEIGHT_M"
fi
if [[ ! -f "$USD" ]]; then
  echo "Isaac USD scene not found at $USD" >&2
  exit 1
fi

cleanup() {
  if [[ -n "${ADAPTER_PID:-}" ]] && kill -0 "$ADAPTER_PID" 2>/dev/null; then
    kill "$ADAPTER_PID" 2>/dev/null || true
    wait "$ADAPTER_PID" 2>/dev/null || true
  fi
  if [[ -n "${GRASP_TASK_PID:-}" ]] && kill -0 "$GRASP_TASK_PID" 2>/dev/null; then
    kill "$GRASP_TASK_PID" 2>/dev/null || true
    wait "$GRASP_TASK_PID" 2>/dev/null || true
  fi
  if [[ -n "${RANGER_TASK_PID:-}" ]] && kill -0 "$RANGER_TASK_PID" 2>/dev/null; then
    kill "$RANGER_TASK_PID" 2>/dev/null || true
    wait "$RANGER_TASK_PID" 2>/dev/null || true
  fi
  rm -f "$RESET_FILE" "$GRASP_RELEASE_FILE" "$GRASP_OPEN_FILE"
}
trap cleanup EXIT INT TERM

echo "R2-visual ROS namespace: /sim/rpe"
echo "Isaac native bridge: isaacsim.ros2.bridge"
echo "Hardware drivers/CAN/root topics are not started or touched."
"$ROS_PYTHON" -m rpe_robot.sim.isaac_adapter &
ADAPTER_PID=$!
sleep "${RPE_ADAPTER_STARTUP_S:-1}"
if [[ "${RPE_GRASP_TASK:-0}" == "1" ]]; then
  "$ROS_PYTHON" -m rpe_robot.sim.grasp_contact_task &
  GRASP_TASK_PID=$!
fi
if [[ "${RPE_RANGER_WHEEL_TASK:-0}" == "1" ]]; then
  "$ROS_PYTHON" -m rpe_robot.sim.ranger_wheel_task &
  RANGER_TASK_PID=$!
fi

ISAAC_ARGS=(
  "$CODEBASE_ROOT/src/rpe_robot/sim/isaac_visual_demo.py"
  --usd "$USD"
  --report "$VISUAL_REPORT"
  --reset-file "$RESET_FILE"
  --physics-mode "$PHYSICS_MODE"
  --grasp-release-file "$GRASP_RELEASE_FILE"
  --grasp-open-file "$GRASP_OPEN_FILE"
)
if [[ "${RPE_R2_WINDOW:-1}" == "1" ]]; then
  ISAAC_ARGS+=(--window)
fi
if [[ -n "${RPE_R2_VISUAL_DURATION:-}" ]]; then
  ISAAC_ARGS+=(--duration "$RPE_R2_VISUAL_DURATION")
fi
# Keep Isaac on its own packaged Python search path. The native bridge has its
# own Jazzy support and uses the same DDS domain as the adapter.
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  "$ISAAC_PYTHON" "${ISAAC_ARGS[@]}"

if [[ -n "${GRASP_TASK_PID:-}" ]]; then
  wait "$GRASP_TASK_PID"
fi
if [[ -n "${RANGER_TASK_PID:-}" ]]; then
  wait "$RANGER_TASK_PID"
fi
