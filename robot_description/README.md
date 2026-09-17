# Combined Robot UR5e Single-Arm Description

This directory is the canonical single-arm description candidate for the
Ranger Mini V3, lift platform, left-mounted UR5e, and left Revo2 hand.

The source inputs are intentionally preserved in their original directories:

- The Ranger Mini V3 body is synchronized with the official xacro in
  `assets/ranger_mini.xacro`, downloaded from
  `agilexrobotics/ugv_gazebo_sim/ranger_mini/ranger_mini_v3/urdf/ranger_mini.xacro`.
- Lift and left-hand files come from `assets/combined_robot`.
- UR5e macro and meshes come from `assets/combined_robot_ur5e`.

The right arm, right arm mount, right hand, and right-hand meshes are not part
of this description. Sensor frames are intentionally not included yet. Thor
has identified the following USB devices:

- Odin1: Rockchip `2207:0019`, mounted on the robot's +x surface and facing +x.
- Hand-mounted camera: Intel RealSense D435, `8086:0b07`.
- Top-mounted camera: Intel RealSense D455, `8086:0b5c`, mounted on the +z
  surface and facing +x.

An additional planned/missing sensor has not been installed. USB identity alone
does not establish a URDF frame: before adding any sensor frame, record its
exact parent link, mount pose, optical-frame convention, stable device path,
stream configuration, and calibration. Do not add sensor frames from the USB
IDs alone.

## Entry Points

```text
urdf/total_robot.xacro
```

The generated flat description is also checked in here:

```text
urdf/total_robot.urdf
```

`total_robot.urdf` was generated from `total_robot.xacro` with the ROS 2 Jazzy
xacro tool on 2026-09-02. It is a generated artifact and should be regenerated
when the xacro or any included description changes; it should not be edited by
hand.

The current offline checks pass:

- XML parsing succeeds.
- The model has one root, `ranger_base_link`, and all 42 links are connected
  through 41 joints.
- No right-arm or right-hand names remain.
- All 65 mesh references (39 unique paths) resolve within this directory.
- Joint limits and mimic-joint targets are present, and the supplied inertias
  are finite and positive definite.

The permanent ROS/xacro toolchain is not installed on the development machine,
but Thor now has ROS 2 Jazzy installed. On Thor, xacro, `check_urdf`,
robot-state-publisher, joint-state-publisher, TF smoke checks, and the
display-dependent RViz inspection with fixed and sample joint states passed
for this description. The independent FK/Jacobian comparison remains
pending. Isaac Sim, Isaac Lab, and their dependent libraries were
intentionally not installed.

The no-hardware RViz inspection was completed by the operator on 2026-09-03;
it verified the chain and sample states only. It did not connect to or command
the Ranger, lift, UR5e, Revo2 hand, or sensors.

## Model assumptions requiring validation

- The left-arm yaw mount is represented as `0.7853981633974483` radians,
  corresponding to the source bundle's stated 45 degree mount.
- The left hand mount is `0 0 0.02` metres, as specified by the UR5e bundle.
- The lift is present but its joint is not intended for the first task.
- No additional payload mass is included beyond the modeled Revo2 hand.

## Inertial-data provenance

The local Revo2 inertial data was compared with the current official BrainCo
`revo2_description` left-hand URDF and matched for all 17 links that contain
inertial data (mass, center-of-mass origin, and all six tensor terms). The
local file is still retained as the source used by this combined description,
because the official repository may change joint calibration and limits over
time.

The UR5e inertial values in the canonical description are now synchronized with
the current official Universal Robots ROS 2
`config/ur5e/physical_parameters.yaml`. This updates the six UR5e link masses,
COM offsets, inertia-frame rotations, and tensor terms. The vendor file is a
generic UR5e physical-parameter source; it is not a substitute for extracting
the serial-specific kinematic calibration from the robot controller. Do not
change these values solely because a mesh centroid differs; the mesh can omit
motors, bearings, electronics, wiring, and fasteners.

The Ranger body values are synchronized with the downloaded official AgileX
Gazebo-simulation xacro. The lift parameters are retained unchanged because
they have been confirmed correct. The official Ranger file contains legacy
commented transmission/Gazebo content; that content is intentionally excluded
from this description and must not be interpreted as permission to control the
physical robot.

These assumptions are for the description candidate only. They are not vendor
limits or permission to move the robot.
