#!/usr/bin/env python3
"""Interactive Isaac Sim visual endpoint for the R2 simulation contract.

This process owns no ROS Python nodes.  It enables Isaac Sim's native
``isaacsim.ros2.bridge`` extension and builds an OmniGraph containing native
Twist/joint-state subscribers, an articulation controller, and a live joint
state publisher.  ``isaac_adapter.py`` translates the two public R2 message
types that do not have typed Isaac nodes (JointTrajectory and
Float32MultiArray) into JointState messages.

The Ranger base is intentionally a bounded *kinematic visual* controller:
the validated Twist is integrated into the imported robot root pose.  Wheel
contact/dynamics calibration is outside R2-visual and is not implied by this
demo.  The lift remains fixed and is never exposed as a commandable DOF.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time


ROBOT_DEFAULT = "/combined_robot_ur5e_single_arm"
NAMESPACE = "/sim/rpe"
BASE_MAX_LINEAR_M_S = 2.0
BASE_MAX_ANGULAR_RAD_S = 3.259
COMMAND_TIMEOUT_S = 0.5
VISUAL_MODE = "visual"
ARM_HAND_PHYSICS_MODE = "arm-hand"
GRASP_CONTACT_MODE = "grasp-contact"
RANGER_WHEEL_PHYSICS_MODE = "ranger-wheel"
PHYSICS_MODES = (
    VISUAL_MODE,
    ARM_HAND_PHYSICS_MODE,
    GRASP_CONTACT_MODE,
    RANGER_WHEEL_PHYSICS_MODE,
)
# Measured from the zero-arm-pose Revo2 touch-link transforms at the bounded
# PINCH target. This keeps the calibration object centered where the two pads
# meet without starting the open hand in penetration.
PINCH_OBJECT_OFFSET_M = (0.0104718, -0.0012504, -0.0396731)
GRASP_OBJECT_RADIUS_M = float(os.environ.get("RPE_GRASP_OBJECT_RADIUS_M", "0.012"))
GRASP_OBJECT_HEIGHT_M = float(os.environ.get("RPE_GRASP_OBJECT_HEIGHT_M", "0.042"))
GRASP_OBJECT_MASS_KG = float(os.environ.get("RPE_GRASP_OBJECT_MASS_KG", "0.03"))
# Measured direction from the thumb collision surface toward the index
# collision surface at the bounded pinch target.
PINCH_OBJECT_AXIS = (-0.4965, 0.5602, -0.6632)


def _has_physics(physics_mode: str) -> bool:
    return physics_mode in (
        ARM_HAND_PHYSICS_MODE,
        GRASP_CONTACT_MODE,
        RANGER_WHEEL_PHYSICS_MODE,
    )


def _set_extension_enabled(name: str) -> None:
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    if not manager.is_extension_enabled(name):
        manager.set_extension_enabled_immediate(name, True)


def _find_robot_path(stage, requested: str | None) -> str:
    from pxr import UsdPhysics

    if requested:
        prim = stage.GetPrimAtPath(requested)
        if prim and prim.IsValid():
            return requested
    # The public Isaac 6.x importer places ArticulationRootAPI on the physical
    # root link under the model's Geometry hierarchy.  Resolve that prim before
    # falling back to the stage default (which is only a model container).
    articulation_roots = [
        prim for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if articulation_roots:
        # Prefer the shallowest root; more than one nested root is invalid.
        articulation_roots.sort(key=lambda prim: str(prim.GetPath()).count("/"))
        return str(articulation_roots[0].GetPath())
    default = stage.GetDefaultPrim()
    if default and default.IsValid():
        return str(default.GetPath())
    for candidate in (ROBOT_DEFAULT, "/World/rpe_robot", "/rpe_robot"):
        prim = stage.GetPrimAtPath(candidate)
        if prim and prim.IsValid():
            return candidate
    raise RuntimeError("could not find the imported robot root prim")


def _make_ros_graph(robot_path: str) -> dict[str, object]:
    """Create native Isaac ROS 2 bridge nodes and return node handles."""
    import omni.graph.core as og
    import usdrt.Sdf

    keys = og.Controller.Keys
    graph_path = "/RPEVisualROSGraph"
    _, nodes, _, _ = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("SubscribeTwist.inputs:nodeNamespace", NAMESPACE),
                ("SubscribeTwist.inputs:topicName", "isaac/base_cmd"),
                ("SubscribeTwist.inputs:queueSize", 10),
                ("SubscribeJointState.inputs:nodeNamespace", NAMESPACE),
                ("SubscribeJointState.inputs:topicName", "isaac/joint_command"),
                ("SubscribeJointState.inputs:queueSize", 10),
                ("ArticulationController.inputs:robotPath", robot_path),
                ("PublishJointState.inputs:nodeNamespace", NAMESPACE),
                ("PublishJointState.inputs:topicName", "state/joint_states"),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_path)]),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
        },
    )
    # ``nodes`` is ordered exactly as CREATE_NODES above in Isaac Sim 6.1.
    return {
        "graph_path": graph_path,
        "twist": nodes[3],
        "joint_subscriber": nodes[4],
        "controller": nodes[5],
        "joint_publisher": nodes[6],
    }


def _make_ranger_wheel_graph(robot_path: str) -> dict[str, object]:
    """Create one atomic controller path for all eight Ranger DOFs."""
    import omni.graph.core as og
    import usdrt.Sdf

    keys = og.Controller.Keys
    graph_path = "/RPERangerWheelROSGraph"
    _, nodes, _, _ = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SubscribeBaseJoints", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("BaseController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("SubscribeTractionMode", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("SubscribeBaseJoints.inputs:nodeNamespace", NAMESPACE),
                ("SubscribeBaseJoints.inputs:topicName", "isaac/base_joint_command"),
                ("SubscribeBaseJoints.inputs:queueSize", 10),
                ("BaseController.inputs:robotPath", robot_path),
                ("SubscribeTractionMode.inputs:nodeNamespace", NAMESPACE),
                ("SubscribeTractionMode.inputs:topicName", "isaac/base_traction_mode"),
                ("SubscribeTractionMode.inputs:queueSize", 10),
                ("PublishJointState.inputs:nodeNamespace", NAMESPACE),
                ("PublishJointState.inputs:topicName", "state/joint_states"),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_path)]),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeBaseJoints.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "BaseController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeTractionMode.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "SubscribeBaseJoints.inputs:context"),
                ("Context.outputs:context", "SubscribeTractionMode.inputs:context"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeBaseJoints.outputs:jointNames", "BaseController.inputs:jointNames"),
                ("SubscribeBaseJoints.outputs:positionCommand", "BaseController.inputs:positionCommand"),
                ("SubscribeBaseJoints.outputs:velocityCommand", "BaseController.inputs:velocityCommand"),
            ],
        },
    )
    return {
        "graph_path": graph_path,
        "joint_subscriber": nodes[2],
        "controller": nodes[3],
        "traction_mode_subscriber": nodes[4],
    }


def _get_twist(twist_node) -> tuple[float, float, float, float, float, float]:
    import omni.graph.core as og

    def read(name: str) -> list[float]:
        value = og.Controller.get(twist_node.get_attribute(name))
        if value is None:
            return [0.0, 0.0, 0.0]
        try:
            values = list(value)
        except TypeError:
            return [0.0, 0.0, 0.0]
        if len(values) != 3 or not all(math.isfinite(float(x)) for x in values):
            return [0.0, 0.0, 0.0]
        return [float(x) for x in values]

    linear = read("outputs:linearVelocity")
    angular = read("outputs:angularVelocity")
    return (*linear, *angular)


def _get_scalar(subscriber_node) -> float:
    import omni.graph.core as og

    value = og.Controller.get(
        subscriber_node.get_attribute("outputs:positionCommand")
    )
    if value is None:
        return 0.0
    try:
        values = list(value)
    except TypeError:
        return 0.0
    if not values or not math.isfinite(float(values[0])):
        return 0.0
    return float(values[0])


def _ranger_traction_switch(stage):
    """Return a stateful high-traction/low-scrub runtime switch."""
    from pxr import UsdPhysics

    material_prim = stage.GetPrimAtPath("/World/PhysicsMaterials/RangerWheels")
    if not material_prim or not material_prim.IsValid():
        raise RuntimeError("Ranger wheel material is missing from the USD stage")
    material = UsdPhysics.MaterialAPI(material_prim)
    static_attr = material.GetStaticFrictionAttr()
    dynamic_attr = material.GetDynamicFrictionAttr()
    wheel_drives = []
    wheel_names = {"fr_wheel", "fl_wheel", "rl_wheel", "rr_wheel"}
    for prim in stage.Traverse():
        if prim.GetName() in wheel_names and "RevoluteJoint" in str(prim.GetTypeName()):
            wheel_drives.append(UsdPhysics.DriveAPI.Apply(prim, "angular"))
    if len(wheel_drives) != 4:
        raise RuntimeError(
            f"Ranger traction switch expected four wheel drives, found {len(wheel_drives)}"
        )
    profiles = {
        "align": (0.02, 0.01, 5.0),
        "drive": (0.20, 0.15, 100.0),
        "brake": (0.20, 0.15, 100.0),
    }
    current: str | None = None
    transitions = 0

    def set_mode(mode_value: float) -> None:
        nonlocal current, transitions
        if mode_value >= 1.5:
            mode = "brake"
        elif mode_value >= 0.5:
            mode = "drive"
        else:
            mode = "align"
        if current == mode:
            return
        static_friction, dynamic_friction, damping = profiles[mode]
        static_attr.Set(static_friction)
        dynamic_attr.Set(dynamic_friction)
        for drive in wheel_drives:
            drive.GetDampingAttr().Set(damping)
        current = mode
        transitions += 1

    def report() -> dict[str, object]:
        return {
            "mode": current,
            "transitions": transitions,
            "static_friction": float(static_attr.Get()),
            "dynamic_friction": float(dynamic_attr.Get()),
            "wheel_drive_damping": (
                float(wheel_drives[0].GetDampingAttr().Get())
            ),
        }

    return set_mode, report


def _configure_root_pose(stage, robot_path: str):
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(robot_path)
    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()
    translate = next((op for op in ops if "translate" in str(op.GetOpName())), None)
    orient = next((op for op in ops if "orient" in str(op.GetOpName())), None)
    if translate is None:
        translate = xform.AddTranslateOp()
    if orient is None:
        orient = xform.AddOrientOp()
    raw_translation = translate.Get()
    raw_orientation = orient.Get()
    initial_translation = Gf.Vec3d(raw_translation or Gf.Vec3d(0.0, 0.0, 0.0))
    initial_orientation = Gf.Quatd(raw_orientation or Gf.Quatd(1.0))
    # Preserve the precision authored by the importer. Isaac 6.x uses float
    # xform ops; USD rejects GfVec3d/GfQuatd values for those attributes.
    translate_is_float = translate.GetPrecision() == UsdGeom.XformOp.PrecisionFloat
    orient_is_float = orient.GetPrecision() == UsdGeom.XformOp.PrecisionFloat

    def apply(x: float, y: float, yaw: float) -> None:
        translation_value = Gf.Vec3d(
            initial_translation[0] + x,
            initial_translation[1] + y,
            initial_translation[2],
        )
        if translate_is_float:
            translation_value = Gf.Vec3f(translation_value)
        translate.Set(translation_value)
        yaw_quat = Gf.Quatd(math.cos(yaw * 0.5), Gf.Vec3d(0.0, 0.0, math.sin(yaw * 0.5)))
        orientation_value = yaw_quat * initial_orientation
        if orient_is_float:
            imaginary = orientation_value.GetImaginary()
            orientation_value = Gf.Quatf(
                float(orientation_value.GetReal()),
                Gf.Vec3f(float(imaginary[0]), float(imaginary[1]), float(imaginary[2])),
            )
        orient.Set(orientation_value)

    return apply, initial_translation, initial_orientation


def _validate_scene(stage, physics_mode: str) -> dict[str, object]:
    """Verify the selected visual or arm/hand physics configuration."""
    from pxr import PhysxSchema, UsdPhysics

    physics_path = "/World/PhysicsScene"
    physics_prim = stage.GetPrimAtPath(physics_path)
    if not physics_prim or not physics_prim.IsValid():
        raise RuntimeError(f"R2 visual physics scene is missing: {physics_path}")
    gravity_attr = physics_prim.GetAttribute("physics:gravityMagnitude")
    gravity = gravity_attr.Get() if gravity_attr and gravity_attr.IsValid() else None
    if gravity is None or not math.isfinite(float(gravity)):
        raise RuntimeError("R2 visual physics scene has no finite gravity magnitude")
    expected_gravity = 9.81 if _has_physics(physics_mode) else 0.0
    if abs(float(gravity) - expected_gravity) > 1.0e-6:
        raise RuntimeError(
            f"{physics_mode} scene expected gravity {expected_gravity:g}, "
            f"got {float(gravity):g} m/s^2"
        )
    physx_scene = PhysxSchema.PhysxSceneAPI(physics_prim)
    time_steps_per_second = int(
        physx_scene.GetTimeStepsPerSecondAttr().Get() or 0
    )
    enhanced_determinism = bool(
        physx_scene.GetEnableEnhancedDeterminismAttr().Get()
    )
    if time_steps_per_second != 60 or not enhanced_determinism:
        raise RuntimeError(
            "R2 physics scene must use 60 Hz enhanced-determinism stepping; "
            f"got {time_steps_per_second} Hz, enhanced={enhanced_determinism}"
        )

    ground_path = "/World/GroundPlane"
    ground_prim = stage.GetPrimAtPath(ground_path)
    if not ground_prim or not ground_prim.IsValid():
        raise RuntimeError(f"R2 visual reference floor is missing: {ground_path}")
    ground_has_collision = ground_prim.HasAPI(UsdPhysics.CollisionAPI)
    expected_collision = _has_physics(physics_mode)
    if ground_has_collision != expected_collision:
        expectation = "have" if expected_collision else "not have"
        raise RuntimeError(
            f"{physics_mode} reference floor must {expectation} CollisionAPI"
        )

    return {
        "physics_profile": physics_mode,
        "physics_mode": (
            "free-base Ranger four-wheel-steering contact physics"
            if physics_mode == RANGER_WHEEL_PHYSICS_MODE
            else (
                "fixed-base Revo2 grasp/contact calibration"
                if physics_mode == GRASP_CONTACT_MODE
                else (
                    "fixed-base arm/hand gravity and contact"
                    if physics_mode == ARM_HAND_PHYSICS_MODE
                    else "zero-gravity articulation with kinematic base visualization"
                )
            )
        ),
        "physics_scene": physics_path,
        "gravity_magnitude_m_s2": float(gravity),
        "physics_time_steps_per_second": time_steps_per_second,
        "enhanced_determinism": enhanced_determinism,
        "ground_plane": ground_path,
        "ground_collision_enabled": ground_has_collision,
        "contact_dynamics": ground_has_collision,
        "base_policy": (
            "dynamic four-wheel-steering articulation"
            if physics_mode == RANGER_WHEEL_PHYSICS_MODE
            else (
                "fixed during arm/hand physics validation"
                if _has_physics(physics_mode)
                else "kinematic visual motion"
            )
        ),
    }


def _find_link_prim(stage, name: str):
    """Return the physical link prim, excluding same-name render children."""
    from pxr import UsdPhysics

    candidates = [prim for prim in stage.Traverse() if prim.GetName() == name]
    rigid = [prim for prim in candidates if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
    if rigid:
        return min(rigid, key=lambda prim: str(prim.GetPath()).count("/"))
    if candidates:
        return min(candidates, key=lambda prim: str(prim.GetPath()).count("/"))
    raise RuntimeError(f"grasp-contact scene is missing link prim {name}")


def _world_position(prim):
    from pxr import UsdGeom

    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()


def _world_planar_pose(prim) -> dict[str, float]:
    """Read finite world position, yaw, and chassis tilt from a link prim."""
    from pxr import Gf, UsdGeom

    transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
    position = transform.ExtractTranslation()
    forward = transform.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0)).GetNormalized()
    up = transform.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)).GetNormalized()
    values = {
        "x": float(position[0]),
        "y": float(position[1]),
        "z": float(position[2]),
        "yaw": float(math.atan2(forward[1], forward[0])),
        "tilt": float(math.acos(max(-1.0, min(1.0, float(up[2]))))),
    }
    if not all(math.isfinite(value) for value in values.values()):
        raise RuntimeError(f"non-finite Ranger chassis pose: {values}")
    return values


def _create_grasp_object(stage) -> dict[str, object]:
    """Create a held calibration cylinder between thumb and index pads."""
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

    thumb = _world_position(_find_link_prim(stage, "right_thumb_touch_link"))
    index = _world_position(_find_link_prim(stage, "right_index_touch_link"))
    center = Gf.Vec3d(
        0.5 * (thumb[0] + index[0]) + PINCH_OBJECT_OFFSET_M[0],
        0.5 * (thumb[1] + index[1]) + PINCH_OBJECT_OFFSET_M[1],
        0.5 * (thumb[2] + index[2]) + PINCH_OBJECT_OFFSET_M[2],
    )
    path = "/World/GraspTask/Object"
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateRadiusAttr(GRASP_OBJECT_RADIUS_M)
    cylinder.CreateHeightAttr(GRASP_OBJECT_HEIGHT_M)
    cylinder.CreateAxisAttr("X")
    cylinder.CreateDisplayColorAttr([Gf.Vec3f(0.10, 0.55, 0.85)])
    cylinder.AddTranslateOp().Set(center)
    orientation = Gf.Rotation(
        Gf.Vec3d(1.0, 0.0, 0.0),
        Gf.Vec3d(PINCH_OBJECT_AXIS).GetNormalized(),
    ).GetQuat()
    imaginary = orientation.GetImaginary()
    cylinder.AddOrientOp().Set(
        Gf.Quatf(
            float(orientation.GetReal()),
            Gf.Vec3f(
                float(imaginary[0]),
                float(imaginary[1]),
                float(imaginary[2]),
            ),
        )
    )
    prim = cylinder.GetPrim()

    UsdPhysics.CollisionAPI.Apply(prim)
    rigid = UsdPhysics.RigidBodyAPI.Apply(prim)
    rigid.CreateRigidBodyEnabledAttr(True)
    rigid.CreateKinematicEnabledAttr().Set(True)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(GRASP_OBJECT_MASS_KG)
    collision_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    collision_api.CreateContactOffsetAttr().Set(0.0015)
    collision_api.CreateRestOffsetAttr().Set(0.0)
    material_path = "/World/PhysicsMaterials/GraspObject"
    material = UsdShade.Material.Define(stage, material_path)
    material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    material_api.CreateStaticFrictionAttr().Set(0.9)
    material_api.CreateDynamicFrictionAttr().Set(0.8)
    material_api.CreateRestitutionAttr().Set(0.0)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        materialPurpose="physics",
    )
    return {
        "path": path,
        "prim": prim,
        "rigid_api": rigid,
        "initial_position": center,
        "radius_m": GRASP_OBJECT_RADIUS_M,
        "height_m": GRASP_OBJECT_HEIGHT_M,
        "axis": list(PINCH_OBJECT_AXIS),
        "mass_kg": GRASP_OBJECT_MASS_KG,
        "placement_policy": "zero-arm-pose calibrated pinch midpoint",
        "pinch_midpoint_offset_m": list(PINCH_OBJECT_OFFSET_M),
        "contact_report_enabled": False,
        "material": {
            "path": material_path,
            "static_friction": 0.9,
            "dynamic_friction": 0.8,
            "restitution": 0.0,
        },
    }


def _position_list(prim) -> list[float]:
    position = _world_position(prim)
    return [float(position[0]), float(position[1]), float(position[2])]


def _find_collision_mesh(stage, link_name: str):
    """Return the collision mesh below a named physical link."""
    from pxr import UsdPhysics

    link = _find_link_prim(stage, link_name)
    prefix = str(link.GetPath()).rstrip("/") + "/"
    candidates = [
        prim
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith(prefix)
        and prim.GetTypeName() == "Mesh"
        and prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    if len(candidates) != 1:
        raise RuntimeError(
            f"expected one collision mesh below {link_name}, found "
            f"{[str(prim.GetPath()) for prim in candidates]}"
        )
    return candidates[0]


def _closest_point_on_triangle(point, a, b, c):
    """Return the closest point on triangle ABC to a world-space point."""
    from pxr import Gf

    ab = b - a
    ac = c - a
    ap = point - a
    d1 = Gf.Dot(ab, ap)
    d2 = Gf.Dot(ac, ap)
    if d1 <= 0.0 and d2 <= 0.0:
        return a

    bp = point - b
    d3 = Gf.Dot(ab, bp)
    d4 = Gf.Dot(ac, bp)
    if d3 >= 0.0 and d4 <= d3:
        return b

    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        v = d1 / (d1 - d3)
        return a + v * ab

    cp = point - c
    d5 = Gf.Dot(ab, cp)
    d6 = Gf.Dot(ac, cp)
    if d6 >= 0.0 and d5 <= d6:
        return c

    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        w = d2 / (d2 - d6)
        return a + w * ac

    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return b + w * (c - b)

    denominator_sum = va + vb + vc
    if abs(denominator_sum) < 1.0e-15:
        return min((a, b, c), key=lambda vertex: (vertex - point).GetLength())
    denominator = 1.0 / denominator_sum
    v = vb * denominator
    w = vc * denominator
    return a + ab * v + ac * w


def _collision_surface_sample(stage, link_name: str, point_world) -> dict[str, object]:
    """Measure a point against the actual imported collision triangles."""
    from pxr import Gf, Usd, UsdGeom

    prim = _find_collision_mesh(stage, link_name)
    mesh = UsdGeom.Mesh(prim)
    points = mesh.GetPointsAttr().Get(Usd.TimeCode.Default()) or []
    counts = mesh.GetFaceVertexCountsAttr().Get(Usd.TimeCode.Default()) or []
    indices = mesh.GetFaceVertexIndicesAttr().Get(Usd.TimeCode.Default()) or []
    if not points or not counts or not indices:
        raise RuntimeError(f"collision mesh has no triangles: {prim.GetPath()}")

    transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    vertices = [transform.Transform(Gf.Vec3d(vertex)) for vertex in points]
    target = Gf.Vec3d(point_world)
    closest = None
    closest_distance = math.inf
    cursor = 0
    triangles = 0
    for count in counts:
        face = indices[cursor : cursor + count]
        cursor += count
        if count < 3:
            continue
        # Imported STL faces are triangles. Fan triangulation also handles a
        # polygonal face without changing the diagnostic.
        for offset in range(1, count - 1):
            candidate = _closest_point_on_triangle(
                target,
                vertices[face[0]],
                vertices[face[offset]],
                vertices[face[offset + 1]],
            )
            distance = (candidate - target).GetLength()
            triangles += 1
            if distance < closest_distance:
                closest = candidate
                closest_distance = distance
    if closest is None:
        raise RuntimeError(f"collision mesh has no valid faces: {prim.GetPath()}")
    return {
        "mesh_path": str(prim.GetPath()),
        "mesh_points": len(points),
        "mesh_triangles": triangles,
        "closest_point_m": [float(closest[0]), float(closest[1]), float(closest[2])],
        "distance_to_object_center_m": float(closest_distance),
    }


def _distance(left: list[float], right: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def run_demo(
    usd_path: Path,
    *,
    headless: bool,
    duration_s: float,
    robot_path: str | None,
    reset_file: Path,
    report_path: Path | None,
    physics_mode: str = VISUAL_MODE,
    grasp_release_file: Path | None = None,
    grasp_open_file: Path | None = None,
) -> dict[str, object]:
    from isaacsim import SimulationApp

    if not usd_path.is_file():
        raise FileNotFoundError(f"USD scene does not exist: {usd_path}")
    app = SimulationApp(
        {
            "headless": headless,
            "renderer": "None" if headless else "RayTracedLighting",
            "hide_ui": headless,
        }
    )
    result: dict[str, object] = {
        "schema": "rpe.r2.isaac_visual_report/v1",
        "isaac_sim": os.environ.get("ISAAC_SIM_VERSION", "6.1.0.0"),
        "namespace": NAMESPACE,
        "hardware_access": False,
        "native_ros2_bridge": True,
        "status": "ERROR",
        "usd": str(usd_path),
        "requested_physics_mode": physics_mode,
    }
    try:
        import omni.timeline
        import omni.usd

        context = omni.usd.get_context()
        if not context.open_stage(str(usd_path)):
            raise RuntimeError(f"failed to open USD scene: {usd_path}")
        stage = None
        for _ in range(120):
            app.update()
            stage = context.get_stage()
            if stage is not None:
                break
        if stage is None:
            raise RuntimeError("Isaac stage did not load")
        scene_configuration = _validate_scene(stage, physics_mode)
        grasp_object = (
            _create_grasp_object(stage)
            if physics_mode == GRASP_CONTACT_MODE
            else None
        )
        # Enabling the ROS bridge while a large USD is still loading can race
        # Isaac's asynchronous stage loader. Open and settle the scene first,
        # then enable the bridge.
        _set_extension_enabled("isaacsim.ros2.bridge")
        for _ in range(30):
            app.update()
        selected_robot_path = _find_robot_path(stage, robot_path)
        default_prim = stage.GetDefaultPrim()
        visual_root_path = (
            str(default_prim.GetPath())
            if default_prim and default_prim.IsValid()
            else selected_robot_path
        )
        # Move the model container for the R2 kinematic base demonstration,
        # not the PhysX articulation root link itself. Authoring transforms on
        # a simulated rigid body produces invalid PhysX transforms in 6.x;
        # moving the common parent keeps Geometry and Physics aligned.
        apply_root, initial_translation, _ = _configure_root_pose(stage, visual_root_path)
        if physics_mode == RANGER_WHEEL_PHYSICS_MODE:
            # The vehicle-isolation USD intentionally has only the eight
            # Ranger DOFs. Do not create the generic arm/hand controller: its
            # JointState commands would reference collapsed upper-body DOFs.
            ranger_wheel_graph = _make_ranger_wheel_graph(selected_robot_path)
            graph = ranger_wheel_graph
            ranger_set_traction, ranger_traction_report = (
                _ranger_traction_switch(stage)
            )
            ranger_set_traction(False)
        else:
            graph = _make_ros_graph(selected_robot_path)
            ranger_wheel_graph = None
            ranger_set_traction = None
            ranger_traction_report = None
        ranger_base_prim = (
            _find_link_prim(stage, "ranger_base_link")
            if physics_mode == RANGER_WHEEL_PHYSICS_MODE
            else None
        )
        ranger_phase_dir = Path(
            os.environ.get("RPE_RANGER_WHEEL_PHASE_DIR", "/tmp/rpe_ranger_wheel/phases")
        )

        if not headless:
            try:
                from isaacsim.core.utils.viewports import set_camera_view

                set_camera_view(
                    eye=(
                        [0.35, -0.35, 1.75]
                        if physics_mode == GRASP_CONTACT_MODE
                        else [4.0, -4.0, 2.8]
                    ),
                    target=(
                        list(grasp_object["initial_position"])
                        if grasp_object is not None
                        else [0.0, 0.0, 0.8]
                    ),
                    camera_prim_path="/OmniverseKit_Persp",
                )
            except Exception:
                # Camera convenience is not part of the ROS/visual contract.
                pass

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        x = y = yaw = 0.0
        max_base_displacement = 0.0
        reset_count = 0
        frames = 0
        last_tick = time.monotonic()
        started = time.monotonic()
        object_release_position = None
        object_open_position = None
        release_thumb_position = None
        release_index_position = None
        release_collision_surfaces = None
        object_max_hold_displacement = 0.0
        release_capture_position = None
        object_released = False
        hand_opened = False
        release_captured = False
        ranger_initial_pose = (
            _world_planar_pose(ranger_base_prim)
            if ranger_base_prim is not None
            else None
        )
        ranger_phase_poses: dict[str, dict[str, float]] = {}
        ranger_phase_names = (
            "settled", "straight", "stopped", "spin_aligned", "spin", "final"
        )
        while app.is_running() and (duration_s <= 0.0 or time.monotonic() - started < duration_s):
            app.update()
            now = time.monotonic()
            dt = min(1.0 / 30.0, max(0.0, now - last_tick))
            last_tick = now

            if reset_file.exists():
                try:
                    reset_file.unlink()
                except FileNotFoundError:
                    pass
                x = y = yaw = 0.0
                # The fixed-base physics scene cannot move its model container
                # during playback. Its reset is entirely joint-target based.
                if physics_mode == VISUAL_MODE:
                    apply_root(x, y, yaw)
                reset_count += 1

            if grasp_object is not None:
                object_position = _position_list(grasp_object["prim"])
                if (
                    not object_released
                    and grasp_release_file is not None
                    and grasp_release_file.exists()
                ):
                    grasp_release_file.unlink(missing_ok=True)
                    release_thumb_position = _position_list(
                        _find_link_prim(stage, "right_thumb_touch_link")
                    )
                    release_index_position = _position_list(
                        _find_link_prim(stage, "right_index_touch_link")
                    )
                    release_collision_surfaces = {
                        "thumb": _collision_surface_sample(
                            stage,
                            "right_thumb_touch_link",
                            object_position,
                        ),
                        "index": _collision_surface_sample(
                            stage,
                            "right_index_touch_link",
                            object_position,
                        ),
                    }
                    release_collision_surfaces["surface_gap_m"] = _distance(
                        release_collision_surfaces["thumb"]["closest_point_m"],
                        release_collision_surfaces["index"]["closest_point_m"],
                    )
                    grasp_object["rigid_api"].GetKinematicEnabledAttr().Set(False)
                    object_release_position = object_position
                    object_released = True
                if object_released and not hand_opened and object_release_position is not None:
                    object_max_hold_displacement = max(
                        object_max_hold_displacement,
                        _distance(object_position, object_release_position),
                    )
                if (
                    not hand_opened
                    and grasp_open_file is not None
                    and grasp_open_file.exists()
                ):
                    grasp_open_file.unlink(missing_ok=True)
                    object_open_position = object_position
                    hand_opened = True
                if (
                    hand_opened
                    and not release_captured
                    and object_open_position is not None
                    and object_open_position[2] - object_position[2] >= 0.10
                ):
                    # The grasp gate needs bounded release evidence, not a
                    # high-speed ground-impact calibration. Freeze the object
                    # after a visible 100 mm gravity drop while the ROS task
                    # continues opening and validating the hand feedback.
                    grasp_object["rigid_api"].GetKinematicEnabledAttr().Set(True)
                    release_capture_position = object_position
                    release_captured = True

            if physics_mode == VISUAL_MODE:
                lx, ly, _lz, _ax, _ay, wz = _get_twist(graph["twist"])
                # Defensive bounds are retained in the Isaac-side visual loop
                # in case an internal publisher is misconfigured.
                lx = max(-BASE_MAX_LINEAR_M_S, min(BASE_MAX_LINEAR_M_S, lx))
                ly = max(-BASE_MAX_LINEAR_M_S, min(BASE_MAX_LINEAR_M_S, ly))
                wz = max(-BASE_MAX_ANGULAR_RAD_S, min(BASE_MAX_ANGULAR_RAD_S, wz))
                c, s = math.cos(yaw), math.sin(yaw)
                x += (lx * c - ly * s) * dt
                y += (lx * s + ly * c) * dt
                yaw += wz * dt
                apply_root(x, y, yaw)
                max_base_displacement = max(max_base_displacement, math.hypot(x, y))
            elif physics_mode == RANGER_WHEEL_PHYSICS_MODE:
                ranger_set_traction(
                    _get_scalar(ranger_wheel_graph["traction_mode_subscriber"])
                )
                pose = _world_planar_pose(ranger_base_prim)
                x = pose["x"] - ranger_initial_pose["x"]
                y = pose["y"] - ranger_initial_pose["y"]
                yaw = pose["yaw"] - ranger_initial_pose["yaw"]
                max_base_displacement = max(max_base_displacement, math.hypot(x, y))
                for phase in ranger_phase_names:
                    marker = ranger_phase_dir / phase
                    if phase not in ranger_phase_poses and marker.exists():
                        ranger_phase_poses[phase] = pose
            frames += 1
            # Leave enough wall time for the Python 3.12 adapter to publish
            # its periodic command/state messages in a headless smoke test.
            time.sleep(1.0 / 120.0)

        timeline.stop()
        grasp_report = None
        if grasp_object is not None:
            final_object_position = _position_list(grasp_object["prim"])
            grasp_report = {
                "object_path": grasp_object["path"],
                "shape": "cylinder",
                "radius_m": grasp_object["radius_m"],
                "height_m": grasp_object["height_m"],
                "axis": grasp_object["axis"],
                "mass_kg": grasp_object["mass_kg"],
                "material": grasp_object["material"],
                "initial_position_m": list(grasp_object["initial_position"]),
                "release_position_m": object_release_position,
                "release_thumb_anchor_m": release_thumb_position,
                "release_index_anchor_m": release_index_position,
                "release_anchor_distance_m": (
                    _distance(release_thumb_position, release_index_position)
                    if release_thumb_position is not None
                    and release_index_position is not None
                    else None
                ),
                "release_collision_surfaces": release_collision_surfaces,
                "open_position_m": object_open_position,
                "final_position_m": final_object_position,
                "released": object_released,
                "hand_opened": hand_opened,
                "release_captured": release_captured,
                "release_capture_threshold_m": 0.10,
                "release_capture_position_m": release_capture_position,
                "max_hold_displacement_m": object_max_hold_displacement,
                "post_open_displacement_m": (
                    _distance(final_object_position, object_open_position)
                    if object_open_position is not None
                    else None
                ),
                "contact_report_enabled": grasp_object["contact_report_enabled"],
                "contact_measurement_policy": (
                    "dynamic retention followed by gravity-driven release; "
                    "PhysX event reporting disabled because Isaac Sim 6.1 aborts "
                    "natively when PhysxContactReportAPI is added to this scene"
                ),
                "post_open_vertical_drop_m": (
                    float(object_open_position[2] - final_object_position[2])
                    if object_open_position is not None
                    else None
                ),
                "self_collision_enabled": False,
                "self_collision_policy": (
                    "deferred after Isaac 6.1 rejected global self-collision on "
                    "the imported full-resolution hand meshes"
                ),
                "command_policy": "partial thumb-index pinch; other fingers open",
            }
        result.update(
            {
                "status": "PASS",
                "robot_prim": selected_robot_path,
                "visual_root_prim": visual_root_path,
                "frames": frames,
                "duration_s": time.monotonic() - started,
                "base_pose": {"x": x, "y": y, "yaw": yaw},
                "base_displacement_m": math.hypot(x, y),
                "max_base_displacement_m": max_base_displacement,
                "reset_count": reset_count,
                **scene_configuration,
                "lift_policy": "fixed at URDF initial position; no ROS lift interface",
                "command_topics": {
                    "base": f"{NAMESPACE}/command/base",
                    "arm": f"{NAMESPACE}/command/arm_trajectory",
                    "hand": f"{NAMESPACE}/command/hand",
                    "stop": f"{NAMESPACE}/command/stop",
                },
                "state_topic": f"{NAMESPACE}/state/joint_states",
                "internal_topics": {
                    "base": f"{NAMESPACE}/isaac/base_cmd",
                    "joint": f"{NAMESPACE}/isaac/joint_command",
                    "base_joints": (
                        f"{NAMESPACE}/isaac/base_joint_command"
                        if ranger_wheel_graph is not None
                        else None
                    ),
                    "base_traction_mode": (
                        f"{NAMESPACE}/isaac/base_traction_mode"
                        if ranger_wheel_graph is not None
                        else None
                    ),
                },
                "initial_root_translation": list(initial_translation),
                "grasp_contact": grasp_report,
                "ranger_wheel_physics": (
                    {
                        "controller_graph": ranger_wheel_graph["graph_path"],
                        "initial_pose": ranger_initial_pose,
                        "final_pose": _world_planar_pose(ranger_base_prim),
                        "phase_poses": ranger_phase_poses,
                        "traction_switch": ranger_traction_report(),
                    }
                    if ranger_wheel_graph is not None
                    else None
                ),
            }
        )
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if report_path is not None:
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        app.close()
    if result["status"] != "PASS":
        raise RuntimeError(str(result.get("error", "Isaac visual demo failed")))
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--usd",
        type=Path,
        default=Path(os.environ.get("RPE_R2_USD", "/tmp/rpe_r2/combined_robot_ur5e_single_arm.usd")),
    )
    parser.add_argument("--window", action="store_true", help="show the Isaac Sim window")
    parser.add_argument(
        "--duration",
        type=float,
        default=float(os.environ.get("RPE_R2_VISUAL_DURATION", "0")),
        help="headless/runtime duration in seconds; 0 means run until Ctrl-C",
    )
    parser.add_argument("--robot-path", default=os.environ.get("RPE_R2_ROBOT_PRIM"))
    parser.add_argument(
        "--physics-mode",
        choices=PHYSICS_MODES,
        default=os.environ.get("RPE_PHYSICS_MODE", VISUAL_MODE),
    )
    parser.add_argument(
        "--reset-file",
        type=Path,
        default=Path(os.environ.get("RPE_R2_RESET_FILE", "/tmp/rpe_r2/reset.request")),
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=(Path(os.environ["RPE_R2_VISUAL_REPORT"]) if os.environ.get("RPE_R2_VISUAL_REPORT") else None),
    )
    parser.add_argument(
        "--grasp-release-file",
        type=Path,
        default=(
            Path(os.environ["RPE_GRASP_RELEASE_FILE"])
            if os.environ.get("RPE_GRASP_RELEASE_FILE")
            else None
        ),
    )
    parser.add_argument(
        "--grasp-open-file",
        type=Path,
        default=(
            Path(os.environ["RPE_GRASP_OPEN_FILE"])
            if os.environ.get("RPE_GRASP_OPEN_FILE")
            else None
        ),
    )
    args = parser.parse_args(argv)
    try:
        result = run_demo(
            args.usd,
            headless=not args.window,
            duration_s=args.duration,
            robot_path=args.robot_path,
            reset_file=args.reset_file,
            report_path=args.report,
            physics_mode=args.physics_mode,
            grasp_release_file=args.grasp_release_file,
            grasp_open_file=args.grasp_open_file,
        )
    except Exception as exc:
        print(f"R2 Isaac visual: FAIL: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
