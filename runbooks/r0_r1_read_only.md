# R0/R1 read-only runbook

Run these commands from inside `thor` after the repository is mounted at
`/workspace`:

```bash
/workspace/codebase/scripts/validate_r0_r1.sh
```

The command parses the canonical URDF, expands and compares the Xacro, runs
`check_urdf`, cross-checks 1,000 synthetic FK configurations against a local
`robot_state_publisher`, and performs ROS graph list queries. It does not
launch Ranger/UR/Revo2/lift drivers and does not publish or call any command
interface.

Expected bridge-network behavior is `NO_GRAPH` in
`reports/discovery_report.md`; this means the container cannot see the host's
DDS graph, not that the robot drivers are absent. The supplemental host
observation in `reports/robot_host_discovery.md` was collected separately
with list/topic introspection only.

Do not add `ros2 topic pub`, `ros2 action send_goal`, controller-manager
switches, safety-gate writes, reset/clear-fault calls, or CAN tools to this
runbook. Physical motion requires a later phase gate and a separately
reviewed, bounded procedure.
