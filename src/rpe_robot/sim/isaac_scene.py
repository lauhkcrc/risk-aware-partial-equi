#!/usr/bin/env python3
"""Import the locked R2 robot description into Isaac Sim 6.1.

The process is simulation-only.  It never imports ``rclpy`` or a vendor
driver, and it does not connect to a ROS graph.  ROS communication is kept in
``bridge.py`` so the simulator and policy can be isolated under ``/sim/rpe``.

Isaac modules are imported *after* ``SimulationApp`` is instantiated; this is
required by Isaac Sim's Carbonite startup sequence.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET


EXPECTED_ARM_JOINTS = (
    "right_arm_shoulder_pan_joint",
    "right_arm_shoulder_lift_joint",
    "right_arm_elbow_joint",
    "right_arm_wrist_1_joint",
    "right_arm_wrist_2_joint",
    "right_arm_wrist_3_joint",
)
EXPECTED_HAND_JOINTS = (
    "right_thumb_metacarpal_joint",
    "right_thumb_proximal_joint",
    "right_index_proximal_joint",
    "right_middle_proximal_joint",
    "right_ring_proximal_joint",
    "right_pinky_proximal_joint",
)
EXPECTED_HAND_MIMIC_JOINTS = (
    "right_thumb_distal_joint",
    "right_index_distal_joint",
    "right_middle_distal_joint",
    "right_ring_distal_joint",
    "right_pinky_distal_joint",
)

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
DEFAULT_SPAWN_HEIGHT_M = 0.324
BASE_JOINTS = (
    "fr_steering_joint",
    "fr_wheel",
    "fl_steering_joint",
    "fl_wheel",
    "rl_steering_joint",
    "rl_wheel",
    "rr_steering_joint",
    "rr_wheel",
)

ARM_MAX_FORCE = {
    "right_arm_shoulder_pan_joint": 150.0,
    "right_arm_shoulder_lift_joint": 150.0,
    "right_arm_elbow_joint": 150.0,
    "right_arm_wrist_1_joint": 28.0,
    "right_arm_wrist_2_joint": 28.0,
    "right_arm_wrist_3_joint": 28.0,
}
HAND_MAX_FORCE = {
    "right_thumb_metacarpal_joint": 0.5,
    "right_thumb_proximal_joint": 1.1,
    "right_index_proximal_joint": 2.0,
    "right_middle_proximal_joint": 2.0,
    "right_ring_proximal_joint": 2.0,
    "right_pinky_proximal_joint": 2.0,
}


def _has_physics(physics_mode: str) -> bool:
    return physics_mode in (
        ARM_HAND_PHYSICS_MODE,
        GRASP_CONTACT_MODE,
        RANGER_WHEEL_PHYSICS_MODE,
    )


def _fix_base(physics_mode: str) -> bool:
    return physics_mode in (ARM_HAND_PHYSICS_MODE, GRASP_CONTACT_MODE)


def _import_with_fixed_base(physics_mode: str) -> bool:
    """Use the importer's stable fixed-root conversion where possible.

    Isaac Sim 6.1's public URDF importer produces an immediately unstable
    articulation for this compound robot when ``fix_base=False``.  The Ranger
    wheel profile therefore imports the same known-good fixed articulation as
    the arm/hand profiles and removes only the generated world root joint
    before the stage is flattened.
    """
    return physics_mode in (
        ARM_HAND_PHYSICS_MODE,
        GRASP_CONTACT_MODE,
        RANGER_WHEEL_PHYSICS_MODE,
    )


def _release_imported_root_joint(stage, robot_path: str) -> dict[str, object]:
    """Remove the importer-authored world joint and verify a free Ranger root."""
    from pxr import UsdPhysics

    root_prefix = robot_path.rstrip("/") + "/"
    candidates = [
        prim
        for prim in stage.Traverse()
        if prim.GetName() == "root_joint"
        and "FixedJoint" in str(prim.GetTypeName())
        and str(prim.GetPath()).startswith(root_prefix)
    ]
    if len(candidates) != 1:
        raise RuntimeError(
            "ranger-wheel profile expected exactly one importer root_joint, "
            f"found {[str(prim.GetPath()) for prim in candidates]}"
        )
    removed_path = str(candidates[0].GetPath())
    stage.RemovePrim(candidates[0].GetPath())
    remaining = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.GetName() == "root_joint"
        and prim.IsA(UsdPhysics.FixedJoint)
        and str(prim.GetPath()).startswith(root_prefix)
    ]
    if remaining:
        raise RuntimeError(f"failed to release imported root joint: {remaining}")
    return {
        "strategy": "fixed-base import followed by root-joint removal",
        "removed_joint": removed_path,
        "free_root_verified": True,
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _prepare_urdf(urdf_path: Path) -> tuple[str, dict[str, object]]:
    """Make mesh references absolute and return text plus topology metadata."""
    root = ET.parse(urdf_path).getroot()
    links = [link.attrib.get("name", "") for link in root.findall("link")]
    joints = [joint.attrib.get("name", "") for joint in root.findall("joint")]
    if "ranger_base_link" not in links:
        raise RuntimeError("canonical URDF is missing ranger_base_link")
    missing = [name for name in EXPECTED_ARM_JOINTS + EXPECTED_HAND_JOINTS if name not in joints]
    if missing:
        raise RuntimeError(f"canonical URDF is missing expected joints: {missing}")

    mesh_count = 0
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        if filename.startswith("../"):
            resolved = (urdf_path.parent / filename).resolve()
            if not resolved.is_file():
                raise RuntimeError(f"URDF mesh does not exist: {resolved}")
            mesh.attrib["filename"] = str(resolved)
            mesh_count += 1
    text = ET.tostring(root, encoding="unicode")
    return text, {
        "links": len(links),
        "joints": len(joints),
        "mesh_references_rewritten": mesh_count,
        "arm_joints": list(EXPECTED_ARM_JOINTS),
        "hand_independent_joints": list(EXPECTED_HAND_JOINTS),
        "lift_joint": "lift_joint",
    }


def _configure_import(import_config, *, fix_base: bool, self_collision: bool) -> None:
    """Set options for the guarded legacy pre-6.x ImportConfig fallback."""
    options = {
        "merge_fixed_joints": False,
        "fix_base": fix_base,
        "make_default_prim": True,
        "self_collision": self_collision,
        "create_physics_scene": True,
        "import_inertia_tensor": True,
        "convex_decomp": False,
        "distance_scale": 1.0,
        "density": 0.0,
    }
    for name, value in options.items():
        if hasattr(import_config, name):
            setattr(import_config, name, value)


def _configure_scene(stage, *, physics_mode: str) -> dict[str, object]:
    """Add the floor and configure the requested physics stage.

    R2-visual deliberately does not claim wheel/contact dynamics.  A collision
    ground caused the uncalibrated Ranger/hand collision asset to diverge in
    Isaac Sim 6.1, while a free articulation fell out of the viewport.  The
    visual endpoint therefore uses an explicit zero-gravity physics scene and
    a non-colliding floor reference.  Collision geometry remains present on
    the robot for later calibrated simulation work.
    """
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    gravity = 9.81 if _has_physics(physics_mode) else 0.0
    physics_scene.CreateGravityMagnitudeAttr(gravity)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
    physx_scene.CreateTimeStepsPerSecondAttr().Set(60)
    physx_scene.CreateEnableEnhancedDeterminismAttr().Set(True)

    # A finite-thickness box is robust for small released objects. The former
    # single-sided mesh allowed fast rigid bodies to tunnel through the floor.
    plane = UsdGeom.Cube.Define(stage, "/World/GroundPlane")
    plane.CreateSizeAttr(1.0)
    plane.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.025))
    plane.AddScaleOp().Set(Gf.Vec3f(20.0, 20.0, 0.05))
    collision_enabled = _has_physics(physics_mode)
    material_report: dict[str, object] | None = None
    if collision_enabled:
        UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
        material_path = "/World/PhysicsMaterials/Ground"
        material = UsdShade.Material.Define(stage, material_path)
        material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        material_api.CreateStaticFrictionAttr().Set(0.9)
        material_api.CreateDynamicFrictionAttr().Set(0.8)
        material_api.CreateRestitutionAttr().Set(0.0)
        UsdShade.MaterialBindingAPI.Apply(plane.GetPrim()).Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
        material_report = {
            "path": material_path,
            "static_friction": 0.9,
            "dynamic_friction": 0.8,
            "restitution": 0.0,
        }
    return {
        "physics_scene": "/World/PhysicsScene",
        "physics_mode": physics_mode,
        "gravity_magnitude_m_s2": gravity,
        "physics_time_steps_per_second": 60,
        "enhanced_determinism": True,
        "ground_plane": "/World/GroundPlane",
        "ground_collision_enabled": collision_enabled,
        "ground_thickness_m": 0.05,
        "contact_dynamics": collision_enabled,
        "ground_material": material_report,
    }


def _configure_grasp_contacts(stage) -> dict[str, object]:
    """Give Revo2 collision shapes an explicit, reproducible contact material."""
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    material_path = "/World/PhysicsMaterials/Revo2Pads"
    material = UsdShade.Material.Define(stage, material_path)
    material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    material_api.CreateStaticFrictionAttr().Set(1.2)
    material_api.CreateDynamicFrictionAttr().Set(1.0)
    material_api.CreateRestitutionAttr().Set(0.0)

    hand_root = next(
        (
            prim
            for prim in stage.Traverse()
            if prim.GetName() == "right_base_link"
            and prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ),
        None,
    )
    if hand_root is None:
        raise RuntimeError("grasp-contact profile cannot find Revo2 rigid-body root")
    hand_root_path = str(hand_root.GetPath()).rstrip("/")

    collision_shapes = 0
    convex_hull_shapes = 0
    contact_bodies = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path != hand_root_path and not path.startswith(hand_root_path + "/"):
            continue
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                material,
                bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                materialPurpose="physics",
            )
            collision_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision_api.CreateContactOffsetAttr().Set(0.0015)
            collision_api.CreateRestOffsetAttr().Set(0.0)
            if prim.GetTypeName() == "Mesh":
                mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
                mesh_collision.CreateApproximationAttr().Set(
                    UsdPhysics.Tokens.convexHull
                )
                convex_hull_shapes += 1
            collision_shapes += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName().startswith("right_"):
            contact_bodies += 1

    if collision_shapes < 17:
        raise RuntimeError(
            "grasp-contact profile found too few Revo2 collision shapes: "
            f"{collision_shapes}"
        )
    return {
        "material": {
            "path": material_path,
            "static_friction": 1.2,
            "dynamic_friction": 1.0,
            "restitution": 0.0,
        },
        "collision_shapes": collision_shapes,
        "convex_hull_collision_shapes": convex_hull_shapes,
        "collision_approximation": "convexHull for Revo2 mesh colliders",
        "rigid_contact_bodies": contact_bodies,
        "contact_report_policy": (
            "disabled; validate contact through dynamic retention and release"
        ),
        "contact_offset_m": 0.0015,
        "rest_offset_m": 0.0,
        "self_collision_enabled": False,
        "self_collision_policy": (
            "deferred until simplified collision shapes and reviewed filtered pairs exist"
        ),
    }


def _configure_ranger_wheel_contacts(stage) -> dict[str, object]:
    """Enable and configure the four native Ranger wheel cylinders.

    The canonical URDF owns the contact geometry. Keep this profile explicit
    by accepting exactly one analytic cylinder collider below each wheel rigid
    body and rejecting a silently approximated or duplicated contact shape.
    """
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

    material_path = "/World/PhysicsMaterials/RangerWheels"
    material = UsdShade.Material.Define(stage, material_path)
    material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    # Analytic cylinders have no tyre deformation or anisotropic tyre model.
    # The runtime controller temporarily lowers this material during measured
    # steer-before-roll alignment, then restores these traction values before
    # driving and braking.  The explicit ``min`` rule keeps both states
    # independent of the ground material's higher coefficients.
    material_api.CreateStaticFrictionAttr().Set(0.2)
    material_api.CreateDynamicFrictionAttr().Set(0.15)
    material_api.CreateRestitutionAttr().Set(0.0)
    physx_material_api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx_material_api.CreateFrictionCombineModeAttr().Set(PhysxSchema.Tokens.min)

    articulation_roots = [
        prim
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(articulation_roots) != 1:
        raise RuntimeError(
            "ranger-wheel profile expected one articulation root, found "
            f"{[str(prim.GetPath()) for prim in articulation_roots]}"
        )
    articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(
        articulation_roots[0]
    )
    articulation_api.CreateSolverPositionIterationCountAttr().Set(32)
    articulation_api.CreateSolverVelocityIterationCountAttr().Set(4)

    wheel_links = {
        "fr_wheel_link",
        "fl_wheel_link",
        "rl_wheel_link",
        "rr_wheel_link",
    }
    wheel_bodies = [
        prim
        for prim in stage.Traverse()
        if prim.GetName() in wheel_links and prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    if len(wheel_bodies) != 4:
        raise RuntimeError(
            "ranger-wheel profile expected four wheel rigid bodies, found "
            f"{[str(prim.GetPath()) for prim in wheel_bodies]}"
        )

    paths: list[str] = []
    geometry: list[dict[str, object]] = []
    for body in wheel_bodies:
        body_path = str(body.GetPath()).rstrip("/")
        mass_api = UsdPhysics.MassAPI(body)
        mass = float(mass_api.GetMassAttr().Get())
        if abs(mass - 8.0) > 1.0e-6:
            raise RuntimeError(
                f"{body.GetPath()} has unexpected wheel mass {mass} kg"
            )
        # The URDF tensor still describes a Z-axis cylinder even though both
        # the wheel joint and the corrected contact cylinder use Y as their
        # axle. Rotate the principal values with the physical cylinder:
        # axial I=0.0324 belongs on Y; transverse I=0.02047 belongs on X/Z.
        corrected_inertia = Gf.Vec3f(0.02047, 0.0324, 0.02047)
        mass_api.GetDiagonalInertiaAttr().Set(corrected_inertia)
        mass_api.GetPrincipalAxesAttr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        contact_prims = []
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if not path.startswith(body_path + "/"):
                continue
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                contact_prims.append(prim)
        if len(contact_prims) != 1:
            raise RuntimeError(
                f"{body.GetName()} expected exactly one imported collider, "
                f"found {[str(prim.GetPath()) for prim in contact_prims]}"
            )
        prim = contact_prims[0]
        if not prim.IsA(UsdGeom.Cylinder):
            raise RuntimeError(
                f"{prim.GetPath()} must remain an analytic cylinder, got "
                f"{prim.GetTypeName()}"
            )
        cylinder = UsdGeom.Cylinder(prim)
        radius = float(cylinder.GetRadiusAttr().Get())
        height = float(cylinder.GetHeightAttr().Get())
        if abs(radius - 0.09) > 1.0e-6 or abs(height - 0.08) > 1.0e-6:
            raise RuntimeError(
                f"{prim.GetPath()} has unexpected wheel dimensions: "
                f"radius={radius}, height={height}"
            )
        # The source describes a nominal quarter turn as 1.57 rad.  A rigid
        # sharp-edged cylinder tilted by the remaining 0.000796 rad rests on
        # only one rim in PhysX and produces asymmetric edge impulses. Keep
        # the source cylinder and center unchanged, but align its axis exactly
        # with the wheel joint for this contact-calibration profile.
        orient_ops = [
            op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()
            if op.GetOpType() == UsdGeom.XformOp.TypeOrient
        ]
        if len(orient_ops) != 1:
            raise RuntimeError(
                f"{prim.GetPath()} expected one orientation op, found "
                f"{[str(op.GetOpName()) for op in orient_ops]}"
            )
        half_sqrt_two = math.sqrt(0.5)
        if orient_ops[0].GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
            orient_ops[0].Set(
                Gf.Quatf(half_sqrt_two, half_sqrt_two, 0.0, 0.0)
            )
        else:
            orient_ops[0].Set(
                Gf.Quatd(half_sqrt_two, half_sqrt_two, 0.0, 0.0)
            )
        translate_ops = [
            op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate
        ]
        if len(translate_ops) != 1:
            raise RuntimeError(
                f"{prim.GetPath()} expected one translation op, found "
                f"{[str(op.GetOpName()) for op in translate_ops]}"
            )
        # The source's -5 mm Z offset is expressed in the rotating wheel-link
        # frame, so it makes the collider center orbit the axle.  Center the
        # physical cylinder on the joint; visuals and joint origins are
        # unchanged.
        if translate_ops[0].GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
            translate_ops[0].Set(Gf.Vec3f(0.0, 0.0, 0.0))
        else:
            translate_ops[0].Set(Gf.Vec3d(0.0, 0.0, 0.0))
        collider = UsdPhysics.CollisionAPI(prim)
        collider.CreateCollisionEnabledAttr().Set(True)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
        collision_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        collision_api.CreateContactOffsetAttr().Set(0.002)
        collision_api.CreateRestOffsetAttr().Set(0.0)
        path = str(prim.GetPath())
        paths.append(path)
        geometry.append(
            {
                "wheel_link": str(body.GetName()),
                "path": path,
                "type": str(prim.GetTypeName()),
                "radius_m": radius,
                "width_m": height,
                "mass_kg": mass,
                "diagonal_inertia_kg_m2": [
                    float(value) for value in corrected_inertia
                ],
                "inertia_alignment": "axial principal inertia on wheel joint Y axis",
                "axis_alignment": "exactly parallel to wheel joint Y axis",
                "center_alignment": "coincident with wheel joint axis",
                "collision_enabled": bool(
                    collider.GetCollisionEnabledAttr().Get()
                ),
            }
        )
    if len(paths) != 4:
        raise RuntimeError(
            "ranger-wheel profile failed to configure four native colliders: "
            f"{paths}"
        )
    return {
        "material": {
            "path": material_path,
            "static_friction": 0.2,
            "dynamic_friction": 0.15,
            "restitution": 0.0,
            "friction_combine_mode": "min",
            "alignment_static_friction": 0.02,
            "alignment_dynamic_friction": 0.01,
            "drive_static_friction": 0.20,
            "drive_dynamic_friction": 0.15,
            "brake_static_friction": 0.2,
            "brake_dynamic_friction": 0.15,
            "friction_policy": (
                "low-slip steering alignment; moderate-traction rolling and braking"
            ),
        },
        "collision_shapes": len(paths),
        "analytic_collision_shapes": len(paths),
        "convex_hull_collision_shapes": 0,
        "collision_approximation": "native URDF analytic cylinders",
        "source_cylinder_collision_enabled": True,
        "contact_geometry": geometry,
        "contact_offset_m": 0.002,
        "rest_offset_m": 0.0,
        "solver": {
            "articulation_root": str(articulation_roots[0].GetPath()),
            "position_iterations": 32,
            "velocity_iterations": 4,
        },
        "wheel_radius_m": 0.09,
        "nominal_wheelbase_m": 0.50,
        "nominal_track_m": 0.38,
        "wheel_joint_axis": [0.0, 1.0, 0.0],
        "steering_joint_axis": [0.0, 0.0, -1.0],
        "collision_paths": paths,
    }


def _set_spawn_height(stage, robot_path: str, spawn_height_m: float) -> list[float]:
    """Place the imported model container above the ground before simulation."""
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(robot_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"cannot apply spawn height: missing robot prim {robot_path}")
    xform = UsdGeom.Xformable(prim)
    translate = next(
        (op for op in xform.GetOrderedXformOps() if "translate" in str(op.GetOpName())),
        None,
    )
    if translate is None:
        translate = xform.AddTranslateOp()
    raw = translate.Get()
    initial = Gf.Vec3d(raw or Gf.Vec3d(0.0, 0.0, 0.0))
    placed = Gf.Vec3d(initial[0], initial[1], initial[2] + spawn_height_m)
    if translate.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
        translate.Set(Gf.Vec3f(placed))
    else:
        translate.Set(placed)
    return [float(placed[0]), float(placed[1]), float(placed[2])]


def _configure_articulation_drives(
    stage, *, lock_base_joints: bool, drive_base_joints: bool,
    lock_upper_body: bool
) -> dict[str, int]:
    """Drive only public R2 joints and lock the lift at zero.

    Ranger wheel joints are not driven because R2 base motion is explicitly
    kinematic. Revo2 distal joints are URDF mimic joints and must not receive
    independent drives. Driving every revolute joint over-constrained the
    Isaac 6.1 articulation and produced invalid PhysX transforms.
    """
    from pxr import UsdPhysics

    arm_driven = 0
    hand_driven = 0
    lift_locked = 0
    base_locked = 0
    steering_driven = 0
    wheels_driven = 0
    upper_body_locked = 0
    root_fixed = 0
    for prim in stage.Traverse():
        name = prim.GetName()
        type_name = str(prim.GetTypeName())
        if (
            lock_upper_body
            and "RevoluteJoint" in type_name
            and name in (
                EXPECTED_ARM_JOINTS
                + EXPECTED_HAND_JOINTS
                + EXPECTED_HAND_MIMIC_JOINTS
            )
        ):
            try:
                # Vehicle calibration isolates the mobile base. Locking the
                # high-ratio arm/hand tree avoids feeding its tiny distal
                # inertias through a freely moving chassis articulation.
                for attr_name in ("physics:lowerLimit", "physics:upperLimit"):
                    attr = prim.GetAttribute(attr_name)
                    if attr:
                        attr.Set(0.0)
                upper_body_locked += 1
            except Exception:
                continue
        elif "RevoluteJoint" in type_name and name in ARM_MAX_FORCE:
            try:
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive.GetStiffnessAttr().Set(100.0)
                drive.GetDampingAttr().Set(10.0)
                drive.GetMaxForceAttr().Set(ARM_MAX_FORCE[name])
                arm_driven += 1
            except Exception:
                continue
        elif "RevoluteJoint" in type_name and name in HAND_MAX_FORCE:
            try:
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                drive.GetStiffnessAttr().Set(20.0)
                drive.GetDampingAttr().Set(2.0)
                drive.GetMaxForceAttr().Set(HAND_MAX_FORCE[name])
                hand_driven += 1
            except Exception:
                continue
        elif "PrismaticJoint" in type_name and name == "lift_joint":
            try:
                # A hard zero-width joint range fixes the unused lift without
                # the numerically aggressive 1e6 N/m drive used previously.
                for attr_name in ("physics:lowerLimit", "physics:upperLimit"):
                    attr = prim.GetAttribute(attr_name)
                    if attr:
                        attr.Set(0.0)
                lift_locked = 1
            except Exception:
                continue
        elif drive_base_joints and "RevoluteJoint" in type_name and name in BASE_JOINTS:
            try:
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
                if name.endswith("_steering_joint"):
                    drive.GetStiffnessAttr().Set(500.0)
                    drive.GetDampingAttr().Set(50.0)
                    # Four finite-width cylinder contacts can deliver a
                    # short steering-axis impulse when rolling begins.  Keep
                    # the existing stiffness/damping response, but leave
                    # enough torque headroom for the steering drive to hold
                    # its already-aligned target through that impulse.
                    drive.GetMaxForceAttr().Set(100.0)
                    steering_driven += 1
                else:
                    drive.GetStiffnessAttr().Set(0.0)
                    drive.GetDampingAttr().Set(100.0)
                    drive.GetMaxForceAttr().Set(1000.0)
                    wheels_driven += 1
            except Exception:
                continue
        elif lock_base_joints and "RevoluteJoint" in type_name and name in BASE_JOINTS:
            try:
                # Stage one validates arm/hand physics in isolation. The Ranger
                # articulation stays present but all steering and wheel DOFs
                # are locked until the separate vehicle-contact milestone.
                for attr_name in ("physics:lowerLimit", "physics:upperLimit"):
                    attr = prim.GetAttribute(attr_name)
                    if attr:
                        attr.Set(0.0)
                base_locked += 1
            except Exception:
                continue
        elif "FixedJoint" in type_name and name == "root_joint":
            root_fixed = 1
    return {
        "position_drives": arm_driven + hand_driven + steering_driven,
        "arm_position_drives": arm_driven,
        "hand_position_drives": hand_driven,
        "base_joint_drives": steering_driven + wheels_driven,
        "base_steering_position_drives": steering_driven,
        "base_wheel_velocity_drives": wheels_driven,
        "base_joints_locked": base_locked,
        "base_root_fixed": root_fixed,
        "mimic_joint_drives": 0,
        "lift_locked": lift_locked,
        "upper_body_joints_locked": upper_body_locked,
        "arm_drive_stiffness": 100.0,
        "arm_drive_damping": 10.0,
        "hand_drive_stiffness": 20.0,
        "hand_drive_damping": 2.0,
        "steering_drive_stiffness": 500.0,
        "steering_drive_damping": 50.0,
        "steering_drive_max_force": 100.0,
        "wheel_drive_stiffness": 0.0,
        "wheel_drive_damping": 100.0,
        "wheel_alignment_drive_damping": 5.0,
        "wheel_drive_max_force": 1000.0,
    }


def _visual_link_report(stage, urdf_root: ET.Element, root_path: str) -> dict[str, object]:
    """Verify every URDF link that declares visuals has a composed visual prim.

    Isaac may represent an imported visual as an instance whose mesh children
    live under a prototype.  Such a prim is valid even when ``GetChildren`` is
    empty, so instances count as composed geometry here.  This check is more
    meaningful than only counting Mesh prims: it catches a dropped link while
    still accepting the importer’s instancing representation.
    """
    expected = [
        link.attrib["name"]
        for link in urdf_root.findall("link")
        if link.attrib.get("name") and link.findall("visual")
    ]
    missing: list[str] = []
    for name in expected:
        # Isaac 5.x emitted ``<root>/<link>/visuals`` while the public 6.x
        # importer emits a ``<root>/Geometry`` hierarchy that mirrors the
        # link tree.  Locate either representation, including nested links
        # (e.g. an arm link under the lift link).
        candidates = [stage.GetPrimAtPath(f"{root_path}/{name}/visuals")]
        candidates.append(stage.GetPrimAtPath(f"{root_path}/Geometry/{name}"))
        candidates.extend(
            prim for prim in stage.Traverse()
            if prim.GetName() == name
            and str(prim.GetPath()).startswith(f"{root_path}/Geometry/")
        )
        prim = next((candidate for candidate in candidates if candidate and candidate.IsValid()), None)
        if prim is None or prim.IsInstance():
            if prim is None:
                missing.append(name)
            continue
        prefix = str(prim.GetPath()).rstrip("/") + "/"
        has_mesh_child = any(
            child.GetTypeName() == "Mesh" and str(child.GetPath()).startswith(prefix)
            for child in stage.Traverse()
        )
        if not has_mesh_child:
            # A link prim can itself be a Mesh in older importer revisions.
            if prim.GetTypeName() != "Mesh":
                missing.append(name)
    return {
        "expected": len(expected),
        "composed": len(expected) - len(missing),
        "missing": missing,
    }


def _import_with_current_api(
    urdf_path: Path,
    output_usd: Path,
    simulation_app,
    *,
    fix_base: bool,
    self_collision: bool,
    collapse_upper_body: bool = False,
) -> tuple[object, str, dict[str, object], Path]:
    """Import a URDF with the Isaac Sim 6.x ``URDFImporter`` API.

    Isaac Sim 5.x exposed the private ``_urdf`` command API.  Isaac Sim 6.x
    removed that symbol and replaced it with ``URDFImporter`` /
    ``URDFImporterConfig``.  Keep all importer-version-specific work in this
    helper so the rest of the scene validation remains unchanged.
    """
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    import omni.usd

    urdf_text, topology = _prepare_urdf(urdf_path)
    # The importer writes a package directory below ``usd_path``.  Use a
    # clean, private staging directory on every run, then export the flattened
    # artifact at the stable path consumed by the visual demo.
    staging_dir = output_usd.parent / "_urdf_import"
    if staging_dir.exists():
        shutil.rmtree(staging_dir)
    staging_dir.mkdir(parents=True, exist_ok=True)
    import_urdf_path = urdf_path
    if collapse_upper_body:
        # The vehicle profile deliberately owns only the eight Ranger DOFs.
        # Convert the arm, hand, and lift joints to fixed joints in a private
        # import artifact, then ask the importer to merge them into the Ranger
        # chassis.  The canonical URDF is never modified and all visual meshes
        # remain in the resulting USD.
        root = ET.fromstring(urdf_text)
        collapsed = 0
        for joint in root.findall("joint"):
            if joint.attrib.get("name") in BASE_JOINTS:
                continue
            joint.attrib["type"] = "fixed"
            for tag in (
                "axis", "limit", "dynamics", "mimic", "calibration",
                "safety_controller",
            ):
                for child in list(joint.findall(tag)):
                    joint.remove(child)
            collapsed += 1
        import_urdf_path = staging_dir / "ranger_wheel_import.urdf"
        import_urdf_path.write_text(ET.tostring(root, encoding="unicode"))
        topology["ranger_wheel_collapsed_joints"] = collapsed
    config = URDFImporterConfig(
        urdf_path=str(import_urdf_path),
        usd_path=str(staging_dir),
        merge_fixed_joints=collapse_upper_body,
        merge_mesh=False,
        debug_mode=False,
        collision_from_visuals=False,
        allow_self_collision=self_collision,
        fix_base=fix_base,
        run_asset_transformer=False,
        run_multi_physics_conversion=True,
    )
    imported_path = Path(URDFImporter(config).import_urdf())
    if not imported_path.is_file():
        raise RuntimeError(f"Isaac URDFImporter returned missing USD: {imported_path}")
    context = omni.usd.get_context()
    context.open_stage(str(imported_path))
    for _ in range(20):
        simulation_app.update()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("Isaac URDFImporter did not create a stage")
    model_name = ET.parse(urdf_path).getroot().attrib.get(
        "name", "combined_robot_ur5e_single_arm"
    )
    candidates = []
    default = stage.GetDefaultPrim()
    if default and default.IsValid():
        candidates.append(str(default.GetPath()))
    candidates.extend((f"/{model_name}", "/World/rpe_robot", "/rpe_robot"))
    prim_path = next(
        (candidate for candidate in candidates
         if stage.GetPrimAtPath(candidate) and stage.GetPrimAtPath(candidate).IsValid()),
        None,
    )
    if prim_path is None:
        raise RuntimeError(
            "Isaac URDFImporter stage has no robot root; "
            f"checked {candidates}"
        )

    # Preserve the report's provenance directory contract without exposing
    # importer scratch internals to callers.
    configuration_dir = output_usd.parent / "configuration"
    configuration_dir.mkdir(parents=True, exist_ok=True)
    (configuration_dir / "imported_path.txt").write_text(str(imported_path) + "\n")
    return stage, prim_path, topology, imported_path


def build_scene(
    urdf_path: Path,
    output_usd: Path,
    report_path: Path,
    *,
    headless: bool,
    steps: int,
    physics_mode: str = VISUAL_MODE,
    spawn_height_m: float = DEFAULT_SPAWN_HEIGHT_M,
) -> dict[str, object]:
    # Carbonite/Omni imports must happen after SimulationApp construction.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({
        "headless": headless,
        "renderer": "None" if headless else "RayTracedLighting",
        "hide_ui": headless,
    })
    result: dict[str, object] = {
        "schema": "rpe.r2.isaac_scene_report/v1",
        "isaac_sim": os.environ.get("ISAAC_SIM_VERSION", "6.1.0.0"),
        "namespace": "/sim/rpe",
        "hardware_access": False,
        "urdf": str(urdf_path),
        "urdf_sha256": _sha256(urdf_path),
        "physics_mode": physics_mode,
        "spawn_height_m": spawn_height_m,
        "status": "ERROR",
    }
    try:
        # All Omni/pxr imports are below SimulationApp by design.
        import omni.usd
        from pxr import Sdf, UsdPhysics

        output_usd.parent.mkdir(parents=True, exist_ok=True)
        # Prefer the public Isaac 6.x importer.  A guarded legacy fallback is
        # retained for older pre-6.x images so this script remains usable for
        # legacy artifact inspection.
        try:
            stage, prim_path_str, topology, imported_path = _import_with_current_api(
                urdf_path,
                output_usd,
                simulation_app,
                fix_base=_import_with_fixed_base(physics_mode),
                self_collision=False,
                collapse_upper_body=physics_mode == RANGER_WHEEL_PHYSICS_MODE,
            )
            source_stage_path = imported_path
            robot_prim = Sdf.Path(prim_path_str)
        except ImportError as current_api_error:
            import omni.kit.commands
            from isaacsim.asset.importer.urdf import _urdf

            urdf_text, topology = _prepare_urdf(urdf_path)
            config = _urdf.ImportConfig()
            _configure_import(
                config,
                fix_base=_import_with_fixed_base(physics_mode),
                self_collision=False,
            )
            parse_ok, robot_model = omni.kit.commands.execute(
                "URDFParseText", urdf_string=urdf_text, import_config=config
            )
            if not parse_ok:
                raise RuntimeError("Isaac URDFParseText returned failure") from current_api_error
            import_ok, robot_prim = omni.kit.commands.execute(
                "URDFImportRobot", urdf_path=str(urdf_path), urdf_robot=robot_model,
                import_config=config, dest_path=str(output_usd),
            )
            if not import_ok:
                raise RuntimeError("Isaac URDFImportRobot returned failure")
            source_stage_path = output_usd
            omni.usd.get_context().open_stage(str(source_stage_path))
            for _ in range(20):
                simulation_app.update()
            stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Isaac stage was not created")
        if not stage.GetPrimAtPath(robot_prim).IsValid():
            raise RuntimeError(
                f"imported stage has no robot root {robot_prim}: {source_stage_path}"
            )
        imported_meshes = [p for p in stage.Traverse() if p.GetTypeName() == "Mesh"]
        if not imported_meshes:
            raise RuntimeError(
                f"imported stage has no composed mesh prims: {source_stage_path}"
            )
        scene_configuration = _configure_scene(stage, physics_mode=physics_mode)
        grasp_configuration = (
            _configure_grasp_contacts(stage)
            if physics_mode == GRASP_CONTACT_MODE
            else None
        )
        ranger_wheel_configuration = (
            _configure_ranger_wheel_contacts(stage)
            if physics_mode == RANGER_WHEEL_PHYSICS_MODE
            else None
        )
        # Importer versions return either an Sdf.Path or a prim-like object;
        # use the canonical destination as a stable fallback for articulation.
        prim_path = robot_prim if isinstance(robot_prim, Sdf.Path) else Sdf.Path("/World/rpe_robot")
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            # Importer builds commonly place the robot at the URDF model name
            # at the stage root rather than under /World.
            model_name = ET.parse(urdf_path).getroot().attrib.get(
                "name", "combined_robot_ur5e_single_arm"
            )
            for candidate in (
                "/World/rpe_robot",
                "/rpe_robot",
                f"/{model_name}",
            ):
                probe = stage.GetPrimAtPath(candidate)
                if probe and probe.IsValid():
                    prim = probe
                    prim_path = Sdf.Path(candidate)
                    break
        root_release = (
            _release_imported_root_joint(stage, str(prim_path))
            if physics_mode == RANGER_WHEEL_PHYSICS_MODE
            else None
        )
        existing_articulation_roots = [
            candidate
            for candidate in stage.Traverse()
            if candidate.HasAPI(UsdPhysics.ArticulationRootAPI)
            and str(candidate.GetPath()).startswith(str(prim_path).rstrip("/") + "/")
        ]
        if (
            prim
            and prim.IsValid()
            and not prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            and not existing_articulation_roots
        ):
            # Isaac 6.x authors the articulation on Geometry/ranger_base_link.
            # Do not add another root above it: nested articulation roots are
            # invalid and stop controller/state nodes from resolving the robot.
            UsdPhysics.ArticulationRootAPI.Apply(prim)
        spawn_translation = _set_spawn_height(stage, str(prim_path), spawn_height_m)
        drive_report = _configure_articulation_drives(
            stage,
            lock_base_joints=_fix_base(physics_mode),
            drive_base_joints=physics_mode == RANGER_WHEEL_PHYSICS_MODE,
            lock_upper_body=physics_mode == RANGER_WHEEL_PHYSICS_MODE,
        )

        # Let the stage initialize and run a short deterministic headless tick.
        for _ in range(max(0, int(steps))):
            simulation_app.update()
        # Flatten after adding the visual floor, physics scene, and drives. This removes the
        # importer’s variant/payload indirection while retaining all composed
        # mesh and physics data in one file.  The configuration directory is
        # still reported for provenance, but is not required to open the
        # flattened output.
        flattened = stage.Flatten()
        if not flattened.Export(str(output_usd)):
            raise RuntimeError(f"failed to export flattened USD: {output_usd}")
        # Re-open the exact artifact that will be consumed by a simulator and
        # verify that flattening did not drop geometry or joints.
        omni.usd.get_context().open_stage(str(output_usd))
        for _ in range(5):
            simulation_app.update()
        final_stage = omni.usd.get_context().get_stage()
        final_meshes = [p for p in final_stage.Traverse() if p.GetTypeName() == "Mesh"]
        final_joints = [
            p for p in final_stage.Traverse() if "Joint" in str(p.GetTypeName())
        ]
        visual_report = _visual_link_report(
            final_stage,
            ET.parse(urdf_path).getroot(),
            str(prim_path),
        )
        if physics_mode == RANGER_WHEEL_PHYSICS_MODE:
            # Merging the fixed upper-body payload intentionally removes some
            # link-name container prims, not their meshes. Validate preservation
            # by exact composed-mesh count and retain the renamed-link list for
            # auditability.
            if len(final_meshes) != len(imported_meshes):
                raise RuntimeError(
                    "ranger-wheel payload collapse changed visual mesh count "
                    f"({len(imported_meshes)} -> {len(final_meshes)})"
                )
            visual_report["collapsed_link_names"] = list(visual_report["missing"])
            visual_report["missing"] = []
            visual_report["validation"] = "exact mesh-count preservation after payload collapse"
        if not final_meshes or not final_joints:
            raise RuntimeError(
                "flattened USD lost required geometry or physics joints "
                f"(meshes={len(final_meshes)}, joints={len(final_joints)})"
            )
        if visual_report["missing"]:
            raise RuntimeError(
                "flattened USD lost visual links: "
                + ", ".join(visual_report["missing"])
            )
        result.update({
            "status": "PASS",
            "topology": topology,
            "robot_prim": str(prim_path),
            "output_usd": str(output_usd),
            "configuration_dir": str(output_usd.parent / "configuration"),
            "source_stage": str(source_stage_path),
            "mesh_prims": len(final_meshes),
            "joint_prims": len(final_joints),
            "visual_links": visual_report,
            "steps": int(steps),
            "scene_configuration": scene_configuration,
            "ranger_wheel_configuration": ranger_wheel_configuration,
            "root_release": root_release,
            "import_policy": (
                "eight Ranger DOFs plus merged rigid upper-body payload"
                if physics_mode == RANGER_WHEEL_PHYSICS_MODE
                else "canonical articulated URDF"
            ),
            "spawn_translation": spawn_translation,
            "ground_plane": scene_configuration["ground_plane"],
            "drive_configuration": drive_report,
            "grasp_contact_configuration": grasp_configuration,
            "lift_policy": "fixed at initial URDF position; not exposed to ROS commands",
        })
    except Exception as exc:  # report the failure with enough context to debug
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        simulation_app.close()
    if result["status"] != "PASS":
        raise RuntimeError(str(result.get("error", "Isaac scene import failed")))
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=Path(os.environ.get(
        "RPE_DESCRIPTION", "/workspace/assets/combined_robot_ur5e_single_arm/urdf/total_robot.urdf"
    )))
    parser.add_argument("--output-usd", type=Path, default=Path(os.environ.get(
        "RPE_R2_USD", "/tmp/rpe_r2/combined_robot_ur5e_single_arm.usd"
    )))
    parser.add_argument("--report", type=Path, default=Path(os.environ.get(
        "RPE_R2_ISAAC_REPORT", "/tmp/rpe_r2/isaac_scene_report.json"
    )))
    parser.add_argument("--steps", type=int, default=10)
    parser.add_argument(
        "--physics-mode",
        choices=PHYSICS_MODES,
        default=os.environ.get("RPE_PHYSICS_MODE", VISUAL_MODE),
    )
    parser.add_argument(
        "--spawn-height",
        type=float,
        default=float(os.environ.get("RPE_SPAWN_HEIGHT_M", str(DEFAULT_SPAWN_HEIGHT_M))),
        help="robot model-container Z offset in metres",
    )
    parser.add_argument("--window", action="store_true", help="show the Isaac window")
    args = parser.parse_args(argv)
    if not args.urdf.is_file():
        parser.error(f"URDF does not exist: {args.urdf}")
    try:
        report = build_scene(
            args.urdf,
            args.output_usd,
            args.report,
            headless=not args.window,
            steps=args.steps,
            physics_mode=args.physics_mode,
            spawn_height_m=args.spawn_height,
        )
    except Exception as exc:
        print(f"R2 Isaac scene: FAIL: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
