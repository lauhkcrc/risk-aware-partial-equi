#!/usr/bin/env bash
set -euo pipefail

CODEBASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_PYTHON="${ISAAC_PYTHON:-/opt/isaaclab/bin/python}"
URDF="${RPE_DESCRIPTION:-/workspace/assets/combined_robot_ur5e_single_arm/urdf/total_robot.urdf}"
REPORT="${RPE_R2_ISAAC_REPORT:-/tmp/rpe_r2/isaac_scene_report.json}"
USD="${RPE_R2_USD:-/tmp/rpe_r2/combined_robot_ur5e_single_arm.usd}"

if [[ ! -x "$ISAAC_PYTHON" ]]; then
  echo "SKIP: Isaac interpreter not present at $ISAAC_PYTHON"
  exit 0
fi
if [[ ! -f "$URDF" ]]; then
  echo "FAIL: canonical URDF not found at $URDF" >&2
  exit 1
fi

"$ISAAC_PYTHON" "$CODEBASE_ROOT/src/rpe_robot/sim/isaac_scene.py" \
  --urdf "$URDF" --output-usd "$USD" --report "$REPORT" --steps "${RPE_R2_STEPS:-10}"

"${PYTHON_BIN:-/usr/bin/python3}" - "$REPORT" "$USD" <<'PY'
import json
import pathlib
import sys
report = json.loads(pathlib.Path(sys.argv[1]).read_text())
assert report.get("status") == "PASS", report
assert report.get("namespace") == "/sim/rpe", report
assert report.get("hardware_access") is False, report
assert pathlib.Path(sys.argv[2]).is_file(), sys.argv[2]
assert report.get("mesh_prims", 0) > 1, report
# Isaac Sim 6.x does not emit Physics* prims for the fixed joints in the
# URDF (the canonical topology still contains all 42 joints).  The imported
# scene therefore has 36 dynamic joint prims plus the fixed relationships.
assert report.get("topology", {}).get("joints", 0) >= 42, report
assert report.get("joint_prims", 0) >= 30, report
drives = report.get("drive_configuration", {})
assert drives.get("arm_position_drives") == 6, drives
assert drives.get("hand_position_drives") == 6, drives
assert drives.get("position_drives") == 12, drives
assert drives.get("base_joint_drives") == 0, drives
assert drives.get("mimic_joint_drives") == 0, drives
assert drives.get("lift_locked") == 1, drives
scene = report.get("scene_configuration", {})
assert scene.get("gravity_magnitude_m_s2") == 0.0, scene
assert scene.get("ground_collision_enabled") is False, scene
assert scene.get("contact_dynamics") is False, scene
configuration_dir = pathlib.Path(report["configuration_dir"])
assert configuration_dir.is_dir(), configuration_dir
assert any(configuration_dir.iterdir()), configuration_dir
print("R2 Isaac headless scene: PASS")
PY
