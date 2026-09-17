#!/usr/bin/env bash
set -eo pipefail

# Complete read-only R0/R1 local validation.  It intentionally does not invoke
# any vendor launch file, controller, service, action, or command publisher.
CODEBASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
"$CODEBASE_ROOT/scripts/audit_urdf.sh"
"$CODEBASE_ROOT/scripts/fk_cross_check.sh"
"$CODEBASE_ROOT/scripts/discover_robot.sh"

# Keep the checked-in contract artifacts machine-parseable as part of the
# same gate. This only reads files; the three wrappers above regenerate their
# own reports.
for json_file in \
  "$CODEBASE_ROOT/manifests/robot_manifest.json" \
  "$CODEBASE_ROOT/manifests/robot_model.lock.json" \
  "$CODEBASE_ROOT/manifests/environment.lock.json" \
  "$CODEBASE_ROOT/reports/urdf_audit.json" \
  "$CODEBASE_ROOT/reports/fk_jacobian_check.json" \
  "$CODEBASE_ROOT/reports/ros_graph_snapshot.json"; do
  "$PYTHON_BIN" -m json.tool "$json_file" >/dev/null
done
[[ -f "$CODEBASE_ROOT/configs/robot/baseline.yaml" ]]
[[ -f "$CODEBASE_ROOT/reports/blockers.md" ]]
echo "R0/R1 validation artifacts: PASS"
