# `rpe-robot` R0/R1 baseline and R2 simulation contract

This nested repository contains the read-only robot-description and ROS 2
discovery baseline for the Risk-Aware Partial Equivariance robot bridge. It is
deliberately scoped to proving that the current CAD-derived model can be
parsed, imported, and checked against a local ROS transform publisher. It is
not a hardware bring-up, simulator task, policy trainer, or safety controller.

The canonical source description is mounted from:

```text
/workspace/assets/combined_robot_ur5e_single_arm
```

The current model is the project-owner-approved right-side UR5e with the
right Revo2 hand. The lift remains in the source URDF for geometry/state
compatibility, but the baseline treats it as fixed and excludes it from the
policy/control contract. `right_base_link` is the initial palm-center proxy.

R2 adds a simulation-only contract under `/sim/rpe`. It does not start a
vendor driver or publish a hardware-like root topic. The deterministic ROS
backend and the Isaac Sim 6.1 ROS-controlled visual endpoint are documented in
[`runbooks/r2_simulation.md`](runbooks/r2_simulation.md).

## Running in `thor`

The intended execution environment is the already-running `thor` container
with the project mounted at `/workspace`. The wrappers source Jazzy safely
(without enabling `nounset` while ROS setup is evaluated), use
`/usr/bin/python3` for ROS 2/rclpy, and default to the mounted asset path.

Run the complete static validation with:

```bash
/workspace/codebase/scripts/validate_r0_r1.sh
```

The individual read-only checks are:

```bash
/workspace/codebase/scripts/audit_urdf.sh
/workspace/codebase/scripts/fk_cross_check.sh
/workspace/codebase/scripts/discover_robot.sh
```

`RPE_ASSET_ROOT`, `RPE_DESCRIPTION`, `RPE_ROS_SETUP`, `PYTHON_BIN`,
`CHECK_URDF_BIN`, and `RPE_FK_SAMPLES` may be overridden for another mounted
checkout. The wrappers never launch a vendor driver, publish `/cmd_vel`, send
a UR trajectory, send a Revo2 command, command the lift, call a controller or
safety service, or send CAN frames. The FK check launches only a private local
`robot_state_publisher` with synthetic joint states and cleans up its process
group on exit.

## What R0/R1 establishes

The checked-in artifacts establish the import contract, active joint names and
ordering, collision/mesh inventory, Xacro-to-URDF consistency, fixed-lift and
palm-center conventions, and a 1,000-sample FK comparison against
`robot_state_publisher`. The Jacobian number in
`reports/fk_jacobian_check.json` is a finite-difference sanity check of the
independent FK evaluator; it is not a second Pinocchio/Isaac implementation.

The live robot host was queried separately using ROS list/topic introspection
only. That evidence is recorded in `reports/robot_host_discovery.md`. The
container's default Docker bridge cannot see that DDS graph, so
`reports/discovery_report.md` may correctly report `NO_GRAPH` until the
container is run with host networking or an explicit DDS bridge.

## Scope boundaries

This baseline does not claim serial-specific UR calibration, measured
palm-center offset, controller-level limits, Isaac physics/contact tuning,
MoveIt SRDF approval, or physical motion readiness. The historical Ranger
`vehicle_state=2`/`error_code=512` observation is retained as an accepted
hardware limitation in `reports/blockers.md`; R2 never connects to that robot,
clears faults, or issues motion.

R2 consumes these versioned contracts through the deterministic simulator
bridge, the minimal baseline task manager, and the Isaac scene importer. All
future policy and hardware phases must preserve the independent safety gate
and vendor-driver boundary described in the execution plan.

## R2 quick checks

```bash
/workspace/codebase/scripts/test_r2_contract.sh
/workspace/codebase/scripts/test_r2_isaac_headless.sh
/workspace/codebase/scripts/test_r2_isaac_visual.sh
```

Launch the Isaac-only visualization with:

```bash
/workspace/codebase/scripts/launch_r2_visual.sh
```

Run the isolated fixed-base UR5e/Revo2 physics stage with:

```bash
/workspace/codebase/scripts/test_arm_hand_physics.sh
RPE_R2_WINDOW=1 /workspace/codebase/scripts/launch_arm_hand_physics.sh
```

This separate profile enables gravity and ground collision, fixes the Ranger
root, steering, wheels, and lift, and drives only the six UR5e and six
independent Revo2 joints. The robot model container is placed `0.324 m` above
the ground. It does not enable Ranger vehicle dynamics or access hardware.

Run the simulation-only thumb/index grasp/contact baseline with:

```bash
/workspace/codebase/scripts/test_grasp_contact.sh
RPE_R2_WINDOW=1 /workspace/codebase/scripts/launch_grasp_contact.sh
```

This profile uses an aligned analytic cylinder, explicit hand/object friction,
convex-hull collision for the Revo2 mesh colliders, and a finite-thickness
ground box. The ROS task closes only the thumb and index finger, verifies live
joint and mimic-joint feedback, holds the dynamic object, then opens the hand
and verifies gravity-driven release. It remains entirely under `/sim/rpe` and
does not connect to or command the physical Revo2.

Run the isolated Ranger wheel/contact physics baseline with:

```bash
/workspace/codebase/scripts/test_ranger_wheel_physics.sh
RPE_R2_WINDOW=1 /workspace/codebase/scripts/launch_ranger_wheel_physics.sh
```

This profile gives the free Ranger chassis four steering position drives and
four wheel velocity drives under gravity. Public commands remain on
`/sim/rpe/command/base`; the adapter performs four-wheel steering kinematics,
a 500 ms watchdog, feedback-gated steer-before-roll, bounded acceleration,
and stop braking. One private JointState command owns all eight Ranger DOFs
atomically, avoiding competing articulation-controller writes. To keep the
vehicle test isolated, all non-Ranger joints are merged into a rigid
upper-body payload. Each wheel uses its canonical URDF analytic cylinder
(`0.09 m` radius, `0.08 m` width) for physical contact; the isolated runtime
centres it on the axle, applies the exact axle orientation, and aligns the
wheel inertia tensor with that axis. It therefore proves ROS-to-wheel
actuation and bounded ground contact, not calibrated tyre deformation,
odometry, stopping distance, or simultaneous arm/hand dynamics. The canonical
URDF and other profiles are not modified.

The implemented runtime target is Isaac Sim 6.1.0.0, Isaac Lab's
develop/pre-release 3.0 line, Python 3.12, ROS 2 Jazzy, and an NVIDIA driver
with CUDA 13.0 capability. Isaac's packaged Warp runtime currently reports
its internal CUDA toolkit as 12.9; that is not a request to downgrade the
host driver.

The Isaac checks skip with an explicit message when
`/opt/isaaclab/bin/python` is absent. `RPE_SKIP_ISAAC=1` runs only the
deterministic namespaced ROS bridge. `RPE_SKIP_IMPORT=1` may reuse a known-good
USD for an interactive launch; the visual smoke test rebuilds it by default
to prevent stale imported physics from being silently accepted.
