# R2 simulation-only runbook

R2 is an isolated simulation contract.  It is safe to run in `thor` because it
does not launch Ranger, UR5e, Revo2, lift, CAN, WebGUI, or any vendor driver.
The accepted HL-COMM hardware issue is outside this path.

## Preconditions

The container should have the project mounted at `/workspace`, ROS 2 Jazzy
under `/opt/ros/jazzy`, and (for the Isaac endpoint) Isaac Sim 6.1.0.0,
Isaac Lab's develop/pre-release 3.0 line, and Python 3.12 under
`/opt/isaaclab`. The NVIDIA driver advertises CUDA 13.0 capability. Isaac's
packaged Warp runtime reports an internal CUDA 12.9 toolkit, which is expected
and does not replace the host driver requirement. The canonical URDF is:

```text
/workspace/assets/combined_robot_ur5e_single_arm/urdf/total_robot.urdf
```

## Contract tests

```bash
/workspace/codebase/scripts/test_r2_contract.sh
```

These tests exercise the Ranger mode switch, ambiguous-command rejection,
bounded arm trajectories, six-value hand normalization and URDF mimic joints,
the fixed lift, deterministic reset, and the 500 ms command watchdog.  They do
not require Isaac or a ROS graph.

They also run `BaselineTaskManager`, a deliberately small non-learning task
that resets, accepts one policy-shaped base/arm/hand action, emits an
observation, and terminates at a fixed horizon. It is an integration smoke
test, not a training environment or a claim that a policy is ready for
hardware.

## ROS contract bridge

Start the deterministic ROS backend without Isaac:

```bash
RPE_SKIP_ISAAC=1 /workspace/codebase/scripts/launch_r2_isaac.sh
```

All names are relative to `/sim/rpe`:

```text
/sim/rpe/state/joint_states       sensor_msgs/msg/JointState
/sim/rpe/state/odometry           nav_msgs/msg/Odometry
/sim/rpe/state/tf                 tf2_msgs/msg/TFMessage
/sim/rpe/system/status            std_msgs/msg/String (JSON)

/sim/rpe/command/base             geometry_msgs/msg/Twist
/sim/rpe/command/arm_trajectory   trajectory_msgs/msg/JointTrajectory
/sim/rpe/command/hand             std_msgs/msg/Float32MultiArray (6 values)
/sim/rpe/command/stop             std_msgs/msg/Bool
/sim/rpe/reset                     std_srvs/srv/Trigger
/sim/rpe/pause                     std_srvs/srv/SetBool
```

The base contract preserves the Ranger mode-switched semantics:

- `linear.y != 0` is a parallel/lateral command and must be the only nonzero
  component;
- with `linear.y == 0`, nonzero `linear.x` is Ackermann and pure `angular.z`
  is spin;
- non-finite, out-of-range, or ambiguous combinations are rejected;
- a base command older than 500 ms is treated as stop.

The lift joint is published at its URDF initial value and is not commandable.
The six hand values are normalized `[0, 1]`; distal joints are generated from
the canonical URDF mimic multipliers.

Example simulation-only commands:

```bash
ros2 topic pub --once /sim/rpe/command/base geometry_msgs/msg/Twist \
  '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
ros2 topic pub --once /sim/rpe/command/hand std_msgs/msg/Float32MultiArray \
  '{data: [0.0, 0.2, 0.2, 0.2, 0.2, 0.2]}'
ros2 service call /sim/rpe/reset std_srvs/srv/Trigger '{}'
```

These commands only affect the `/sim/rpe` node.  Do not substitute root-level
hardware names such as `/cmd_vel`, `/joint_states`, or
`/revo2/command_positions`.

## Isaac scene import

Run the headless importer and short startup smoke test:

```bash
/workspace/codebase/scripts/test_r2_isaac_headless.sh
```

Or launch the bridge and importer together:

```bash
/workspace/codebase/scripts/launch_r2_isaac.sh
```

Set `RPE_R2_WINDOW=1` only when a display is available. The importer starts
`SimulationApp` before any Omni/pxr import, expands relative mesh references,
imports the locked URDF, and flattens the result to a standalone USD under
`/tmp/rpe_r2`. It leaves a `configuration/` provenance directory next to the
USD; the flattened USD does not depend on that directory when reopened.

The headless check verifies the robot root, all 37 URDF links with visual
geometry, mesh/joint counts, six UR5e position drives, six independent Revo2
position drives, no Ranger wheel/steering drives, no Revo2 mimic drives, and a
locked lift. It also verifies the deliberately limited R2-visual scene:

- gravity is zero so the ungrounded visual articulation does not fall;
- `/World/GroundPlane` is a non-colliding visual reference;
- wheel/contact dynamics are disabled and are not claimed by this milestone.

This is a stable visualization/control proof, not calibrated vehicle or
contact physics.

## Isaac-only ROS visualization

Start the interactive endpoint (no RViz):

```bash
/workspace/codebase/scripts/launch_r2_visual.sh
```

For an interactive window, `thor` must inherit the host display and an X11
credential. For example, create/run the container with the host's current
`$DISPLAY` and `$XAUTHORITY` (the host in the validation session used
`:1`, not `:0`):

```bash
docker run ... \
  -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=/tmp/.Xauthority \
  -v "${XAUTHORITY}:/tmp/.Xauthority:ro" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ...
```

If the container was created with a stale display (for example `DISPLAY=:0`
while the host is `:1`) or without Xauthority, Isaac can still be fully
validated headlessly, but no GUI window can be displayed until the container
is recreated with the matching display/socket/credential. This is a Docker
display setup issue, not a ROS or robot-model issue.

The launcher rebuilds the USD from the canonical URDF, starts only the
namespaced Python 3.12 message adapter, enables Isaac's native
`isaacsim.ros2.bridge`, and opens the Isaac viewport. It never starts the
deterministic backend at the same time, so there is one owner for simulated
joint state.

Isaac consumes the adapter's internal topics:

```text
/sim/rpe/isaac/base_cmd
/sim/rpe/isaac/joint_command
```

and natively publishes:

```text
/sim/rpe/state/joint_states
```

Public users still command only the `/sim/rpe/command/*` contract listed
above. UR5e and independent Revo2 joints are articulation-controlled. Revo2
targets are limited to a 0.5 rad/s slew because an abrupt large target is
numerically unstable with the tiny inertias in the imported hand model. The
Ranger base is a bounded kinematic visualization: validated Twist commands
move the common visual root, while wheel/contact dynamics remain deferred.
The lift is fixed and has no ROS command interface.

## Fixed-base arm and hand physics

The first incremental physics profile is intentionally isolated from the
known-good zero-gravity R2 visual artifact. Validate it in `thor` with:

```bash
cd /workspace/codebase
scripts/test_arm_hand_physics.sh
```

Open the interactive Isaac viewport with:

```bash
cd /workspace/codebase
RPE_R2_WINDOW=1 scripts/launch_arm_hand_physics.sh
```

This mode uses normal gravity (`9.81 m/s^2`) and a colliding ground plane. The
model container starts at `z=0.324 m`, preventing the reference ground plane
from bisecting the robot. The Ranger root is fixed, all eight Ranger
steering/wheel DOFs and the lift are locked, and base commands are deliberately
ignored. Only the six UR5e joints and six independent Revo2 joints receive
position drives. Revo2 mimic joints remain passive relations and
self-collision remains disabled for this first stability baseline.

The smoke test proves native joint-state publication, bounded arm/hand command
response under gravity, fixed Ranger/lift state, stop, pause/resume, reset,
five-second post-reset settling, finite state, exact spawn height, and clean
Isaac runtime logs. It does not claim calibrated arm/hand contact parameters,
grasp quality, Ranger wheel dynamics, or policy-training readiness.

## Revo2 grasp/contact baseline

The next isolated profile proves one bounded grasp without using physical
hardware. Run the headless acceptance test in `thor` with:

```bash
cd /workspace/codebase
scripts/test_grasp_contact.sh
```

Open the same scene interactively with:

```bash
cd /workspace/codebase
RPE_R2_WINDOW=1 scripts/launch_grasp_contact.sh
```

The fixture is a `24 mm` diameter, `42 mm` long analytic cylinder aligned with
the measured thumb-to-index collision-surface axis. Only thumb opposition,
thumb flexion, and index flexion are commanded; middle, ring, and pinky remain
open. The task stages the cylinder kinematically, closes the hand through the
public `/sim/rpe/command/hand` topic, makes the cylinder dynamic, holds it for
three seconds, and then reopens the hand.

The acceptance test requires finite independent and mimic-joint feedback,
less than `10 mm` cylinder motion during the dynamic hold, and at least
`100 mm` downward movement after opening. The harness then freezes the object
at that bounded release threshold while the hand finishes opening; ground
impact and bounce calibration are deliberately outside this grasp gate. The
test also verifies all 17 Revo2 mesh
colliders use `convexHull`, the hand and object have explicit friction, and the
floor is a colliding `50 mm` thick box. The detailed render meshes are
unchanged. Global hand self-collision and PhysX contact-report callbacks remain
disabled because both crash Isaac Sim 6.1 with this imported model; retention
and release provide the contact evidence instead.

This is a simulation stability/contact baseline, not a physical Revo2
calibration. Real-hand validation of normalized command mapping, joint zeros,
directions, compliance, saturation, and pad friction is deferred to a
separate guarded procedure.

## Ranger wheel/contact physics baseline

Run the complete headless acceptance test in `thor` with:

```bash
cd /workspace/codebase
scripts/test_ranger_wheel_physics.sh
```

Open the same profile interactively with:

```bash
cd /workspace/codebase
RPE_R2_WINDOW=1 scripts/launch_ranger_wheel_physics.sh
```

This is a fourth isolated profile. It imports the known-stable fixed-root
articulation, removes only the generated world root joint, and leaves the
Ranger chassis physically free. The four steering joints use position drives;
the four rolling joints use velocity drives. Gravity, the finite-thickness
ground, explicit wheel/ground friction, and contact dynamics are enabled. The
public interface remains `/sim/rpe/command/base`; private JointState topics
carry one atomic eight-joint steering-position/wheel-velocity command to
Isaac's native bridge. A separate private mode message selects alignment,
drive, or brake contact parameters.

The profile deliberately owns only the Ranger's eight DOFs. During its private
import, the lift, UR5e, and Revo2 joints are converted to fixed joints and
merged into one rigid upper-body payload. All visual meshes are preserved and
the canonical URDF is never changed. Arm/hand articulation remains covered by
the separate `arm-hand` and `grasp-contact` profiles; simultaneous mobile
manipulation physics is not claimed here.

The Ranger profile uses the four canonical imported URDF cylinders directly:
each is `0.09 m` in radius and `0.08 m` wide. The profile verifies that exactly
one cylinder under each wheel rigid body has collision enabled, then binds the
explicit wheel material and contact offsets. At runtime it replaces the
source's rounded `1.57 rad` orientation with an exact quarter turn, centres
each cylinder on its wheel axle, and places the axial inertia on the wheel
joint's Y axis. No sphere contact proxies are created. This is a bounded
ground-motion baseline, not a claim about tyre deformation, slip calibration,
odometry, or real stopping distance.

The test issues only simulation commands, in this order:

1. settle/stop, then require `0.5 s` of measured initial stillness;
2. drive at `linear.x=0.10 m/s` for `2.0 s`;
3. brake until measured wheel and steering feedback remains bounded for
   `0.30 s` (with a `12 s` simulation-time deadline);
4. request `angular.z=0.10 rad/s`, keep rolling gated until every steering
   joint reaches its target and remains still, then spin for `4.0 s`;
5. brake and verify the final measured state with the same bounded gate.

It accepts only finite feedback and a clean Isaac log, all four steering and
wheel drives, a free chassis root, and exactly four enabled native cylinders.
The chassis must remain above `0.20 m` with tilt below `0.10 rad`. Straight
motion must advance at least `0.12 m`, with lateral drift below `0.05 m` and
yaw drift below `0.10 rad`. Stop drift is bounded to `0.20 m`; steering
alignment translation and final drift are each bounded to `0.12 m`; rolling
spin translation is bounded to `0.15 m`; and the complete spin transition is
bounded to `0.20 m`. Straight wheel feedback must be `0.60-1.60 rad/s`. Spin
feedback must have the required `+,-,-,+` signs, signed magnitude above
`0.20 rad/s`, absolute magnitude below `0.70 rad/s`, steering error below
`0.04 rad`, and steering speed below `0.25 rad/s`. Final wheel speed must be
below `0.35 rad/s`.

Chassis yaw during spin is recorded but not used as a qualification gate.
Rigid analytic cylinders use isotropic friction and are not an anisotropic,
compliant tyre model; joint-space actuation and bounded contact motion are the
claims made by this smoke test.

Run the complete headless ROS-to-Isaac smoke test with:

```bash
/workspace/codebase/scripts/test_r2_isaac_visual.sh
```

The smoke test rebuilds the USD by default. Set `RPE_SKIP_IMPORT=1` only to
reuse an explicitly trusted artifact. It proves native state publication,
base/arm/hand response, stop, pause/resume, reset, namespace isolation, and a
clean Isaac log (no invalid PhysX transform, nested articulation root,
traceback, or Isaac `[Error]`).

## R2 acceptance evidence

An R2 run is accepted when:

1. `test_r2_contract.sh` passes;
2. `test_r2_isaac_headless.sh` passes its topology, drive, lift, gravity, and
   non-colliding-floor assertions;
3. `test_r2_isaac_visual.sh` proves ROS base/arm/hand response plus stop,
   pause/resume, and reset through the native Isaac bridge;
4. `test_arm_hand_physics.sh`, `test_grasp_contact.sh`, and
   `test_ranger_wheel_physics.sh` pass their isolated physics/contact gates;
5. only namespaced simulation endpoints exist for the run and no root
   hardware topic leaks into the graph;
6. the Isaac visual log contains no invalid transform, nested root, traceback,
   or Isaac error;
7. no vendor process, CAN interface, or physical command is started.
