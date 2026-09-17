# Robot description boundary

R0/R1 intentionally does not copy or rewrite the robot description. The
source of truth is the mounted asset tree:

```text
/workspace/assets/combined_robot_ur5e_single_arm
```

The audit wrappers consume `urdf/total_robot.xacro` and
`urdf/total_robot.urdf` from that tree and write only reports under
`codebase/reports/`. The current source is the right-side CAD-derived UR5e
mount with the right Revo2 hand. Older README/checklist text elsewhere in the
workspace still describes an earlier left-side model; that stale prose is not
used as the model input.

The lift's source `lift_joint` remains a prismatic URDF joint for description
compatibility. The baseline configuration locks it at the accepted CAD/model
height and does not expose it as a policy action. The initial tool convention
is `right_base_link` as a palm-center proxy; a measured fixed offset can be
added later without renaming the six arm joints.
