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

# Read-only R0 ROS graph discovery.  It does not source or start a driver and
# never publishes a motion command.  Source ROS before running when using a
# non-standard installation; the thor image already provides /opt/ros/jazzy.
CODEBASE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
exec "$PYTHON_BIN" "$CODEBASE_ROOT/src/rpe_robot/audit/discover_ros.py" \
  --json-out "$CODEBASE_ROOT/reports/ros_graph_snapshot.json" \
  --markdown-out "$CODEBASE_ROOT/reports/discovery_report.md"
