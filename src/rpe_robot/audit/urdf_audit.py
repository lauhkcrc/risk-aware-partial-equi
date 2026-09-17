#!/usr/bin/env python3
"""Read-only validation of the canonical combined robot description.

The audit deliberately has no ROS publishers and no hardware dependencies.  It
can therefore be run in the ``thor`` container while the physical robot is
untouched.  The input is the generated URDF; when requested, the Xacro is
expanded first and compared structurally with the checked-in URDF.

The checker is intentionally small and dependency-light.  It validates the
things that can invalidate a ROS/Isaac import (tree connectivity, names,
limits, mimic joints, inertials, and mesh references) and emits JSON suitable
for a manifest as well as a human-readable Markdown report.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


def _float(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _vec(value: str | None, length: int = 3) -> list[float] | None:
    if value is None:
        return None
    try:
        result = [float(x) for x in value.split()]
    except ValueError:
        return None
    if len(result) != length or not all(math.isfinite(x) for x in result):
        return None
    return result


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _resolve_mesh(filename: str, description_dir: Path) -> Path | None:
    """Resolve the relative mesh forms used by the canonical description."""
    if filename.startswith("file://"):
        return Path(filename[7:])
    if filename.startswith("package://"):
        return None
    return (description_dir / filename).resolve()


def _matrix_spd(values: list[float]) -> bool:
    if len(values) != 6 or not all(math.isfinite(x) for x in values):
        return False
    ixx, ixy, ixz, iyy, iyz, izz = values
    # Sylvester criterion for the symmetric 3x3 inertia matrix.  The inertial
    # values in a URDF are expressed in a principal/reference frame and should
    # be symmetric positive definite.
    minor2 = ixx * iyy - ixy * ixy
    determinant = (
        ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz)
    )
    return ixx > 0 and minor2 > 0 and determinant > 0


def _parse(path: Path) -> ET.Element:
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        raise ValueError(f"invalid XML in {path}: {exc}") from exc
    if root.tag != "robot":
        raise ValueError(f"{path} does not have a <robot> root")
    return root


def expand_xacro(xacro: Path) -> tuple[str, Path]:
    """Expand Xacro to a temporary file and return text plus its path."""
    with tempfile.NamedTemporaryFile(prefix="rpe_xacro_", suffix=".urdf", delete=False) as stream:
        output = Path(stream.name)
    try:
        command = ["xacro", str(xacro)]
        completed = subprocess.run(command, check=False, capture_output=True, text=True)
        if completed.returncode:
            raise RuntimeError(
                f"xacro failed ({completed.returncode}): {completed.stderr.strip()}"
            )
        output.write_text(completed.stdout)
        return completed.stdout, output
    except Exception:
        output.unlink(missing_ok=True)
        raise


def audit_urdf(urdf_path: Path, *, xacro_path: Path | None = None) -> dict[str, Any]:
    root = _parse(urdf_path)
    links = root.findall("link")
    joints = root.findall("joint")
    link_names = [link.attrib.get("name", "") for link in links]
    joint_names = [joint.attrib.get("name", "") for joint in joints]
    errors: list[str] = []
    warnings: list[str] = []

    duplicate_links = sorted(name for name, count in Counter(link_names).items() if name and count > 1)
    duplicate_joints = sorted(name for name, count in Counter(joint_names).items() if name and count > 1)
    if duplicate_links:
        errors.append(f"duplicate link names: {duplicate_links}")
    if duplicate_joints:
        errors.append(f"duplicate joint names: {duplicate_joints}")

    link_set = set(link_names)
    child_to_joint: dict[str, ET.Element] = {}
    children: dict[str, list[str]] = defaultdict(list)
    for joint in joints:
        name = joint.attrib.get("name", "<unnamed>")
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            errors.append(f"joint {name} lacks parent or child")
            continue
        parent_name = parent.attrib.get("link", "")
        child_name = child.attrib.get("link", "")
        if parent_name not in link_set:
            errors.append(f"joint {name} references missing parent link {parent_name!r}")
        if child_name not in link_set:
            errors.append(f"joint {name} references missing child link {child_name!r}")
        if child_name in child_to_joint:
            errors.append(f"link {child_name} has more than one parent joint")
        child_to_joint[child_name] = joint
        children[parent_name].append(child_name)

        joint_type = joint.attrib.get("type", "")
        if joint_type not in {"fixed", "revolute", "continuous", "prismatic", "floating", "planar"}:
            errors.append(f"joint {name} has unknown type {joint_type!r}")
        axis = joint.find("axis")
        if joint_type in {"revolute", "continuous", "prismatic"}:
            axis_values = _vec(axis.attrib.get("xyz") if axis is not None else None)
            if axis_values is None or math.sqrt(sum(v * v for v in axis_values)) < 1e-12:
                errors.append(f"movable joint {name} has no valid axis")
        limit = joint.find("limit")
        if joint_type in {"revolute", "prismatic"}:
            if limit is None:
                errors.append(f"bounded joint {name} has no <limit>")
            else:
                lower = _float(limit.attrib.get("lower"))
                upper = _float(limit.attrib.get("upper"))
                velocity = _float(limit.attrib.get("velocity"))
                effort = _float(limit.attrib.get("effort"))
                if lower is None or upper is None or lower > upper:
                    errors.append(f"joint {name} has invalid lower/upper limits")
                if velocity is None or velocity <= 0:
                    errors.append(f"joint {name} has invalid velocity limit")
                if effort is None or effort <= 0:
                    errors.append(f"joint {name} has invalid effort limit")
        mimic = joint.find("mimic")
        if mimic is not None:
            target = mimic.attrib.get("joint", "")
            if target not in joint_names:
                errors.append(f"mimic joint {name} targets missing joint {target!r}")
            if _float(mimic.attrib.get("multiplier", "1")) is None:
                errors.append(f"mimic joint {name} has a non-finite multiplier")
            if _float(mimic.attrib.get("offset", "0")) is None:
                errors.append(f"mimic joint {name} has a non-finite offset")

    roots = sorted(link_set - set(child_to_joint))
    if len(roots) != 1:
        errors.append(f"expected one root link, found {roots}")

    # Reachability and cycle check from the root.
    visited: set[str] = set()
    active: set[str] = set()

    def walk(link: str) -> None:
        if link in active:
            errors.append(f"cycle detected at link {link}")
            return
        if link in visited:
            return
        active.add(link)
        for child in children.get(link, []):
            walk(child)
        active.remove(link)
        visited.add(link)

    if roots:
        walk(roots[0])
    unreachable = sorted(link_set - visited)
    if unreachable:
        errors.append(f"unreachable links from root {roots[0] if roots else '<none>'}: {unreachable}")

    mesh_files: list[dict[str, Any]] = []
    visual_count = 0
    collision_count = 0
    collision_by_type: Counter[str] = Counter()
    description_dir = urdf_path.parent
    for link in links:
        link_name = link.attrib.get("name", "<unnamed>")
        for visual in link.findall("visual"):
            visual_count += 1
            mesh = visual.find("./geometry/mesh")
            if mesh is not None and mesh.attrib.get("filename"):
                filename = mesh.attrib["filename"]
                resolved = _resolve_mesh(filename, description_dir)
                mesh_files.append({
                    "kind": "visual",
                    "link": link_name,
                    "filename": filename,
                    "exists": bool(resolved and resolved.is_file()),
                    "resolved": str(resolved) if resolved else None,
                })
        for collision in link.findall("collision"):
            collision_count += 1
            geometry = collision.find("geometry")
            geometry_type = next(iter(geometry), None).tag if geometry is not None and len(geometry) else "unknown"
            collision_by_type[geometry_type] += 1
            mesh = collision.find("./geometry/mesh")
            if mesh is not None and mesh.attrib.get("filename"):
                filename = mesh.attrib["filename"]
                resolved = _resolve_mesh(filename, description_dir)
                mesh_files.append({
                    "kind": "collision",
                    "link": link_name,
                    "filename": filename,
                    "exists": bool(resolved and resolved.is_file()),
                    "resolved": str(resolved) if resolved else None,
                })

        inertial = link.find("inertial")
        if inertial is not None:
            mass_element = inertial.find("mass")
            mass = _float(mass_element.attrib.get("value")) if mass_element is not None else None
            inertia = inertial.find("inertia")
            terms = [_float(inertia.attrib.get(key)) if inertia is not None else None
                     for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")]
            if mass is None or mass <= 0:
                errors.append(f"link {link_name} has non-positive/non-finite mass")
            if any(value is None for value in terms) or not _matrix_spd([value for value in terms if value is not None]):
                errors.append(f"link {link_name} has invalid/non-positive-definite inertia")
        elif link_name not in {"right_arm_base_link", "right_palm_center"}:
            # Empty frame links are valid, but moving physical links should have
            # inertial data for a later dynamic simulation.
            warnings.append(f"link {link_name} has no inertial element")

    missing_meshes = [entry for entry in mesh_files if not entry["exists"]]
    if missing_meshes:
        errors.append(f"{len(missing_meshes)} mesh references do not resolve")

    # The root inertia is accepted for URDF/RViz. KDL warns about it, while an
    # Isaac articulation may choose to add a dummy root during import.
    if roots and root.find(f"./link[@name='{roots[0]}']/inertial") is not None:
        warnings.append("root link has inertia; KDL warns, and an Isaac importer may add a dummy articulation root")

    # Make an explicit structural comparison between checked-in URDF and Xacro.
    xacro_comparison: dict[str, Any] = {"requested": bool(xacro_path), "status": "not_run"}
    if xacro_path:
        expanded_text, expanded_path = expand_xacro(xacro_path)
        try:
            expanded_root = _parse(expanded_path)
            expanded_links = sorted(e.attrib.get("name", "") for e in expanded_root.findall("link"))
            expanded_joints = sorted(e.attrib.get("name", "") for e in expanded_root.findall("joint"))
            xacro_comparison = {
                "requested": True,
                "status": "pass" if expanded_links == sorted(link_names) and expanded_joints == sorted(joint_names) else "fail",
                "expanded_sha256": hashlib.sha256(expanded_text.encode()).hexdigest(),
                "link_set_equal": expanded_links == sorted(link_names),
                "joint_set_equal": expanded_joints == sorted(joint_names),
                "expanded_link_count": len(expanded_links),
                "expanded_joint_count": len(expanded_joints),
            }
            if xacro_comparison["status"] == "fail":
                errors.append("expanded Xacro link/joint sets differ from checked-in URDF")
        finally:
            expanded_path.unlink(missing_ok=True)

    nonfixed = [joint for joint in joints if joint.attrib.get("type") != "fixed"]
    movable = [joint for joint in nonfixed if joint.attrib.get("type") in {"revolute", "continuous", "prismatic"}]
    mimic_names = [joint.attrib["name"] for joint in joints if joint.find("mimic") is not None]
    result: dict[str, Any] = {
        "schema": "rpe.robot.urdf_audit/v1",
        "source": str(urdf_path),
        "source_sha256": sha256(urdf_path),
        "robot_name": root.attrib.get("name"),
        "links": {"count": len(links), "names": link_names},
        "joints": {
            "count": len(joints),
            "names": joint_names,
            "nonfixed_count": len(nonfixed),
            "movable_count": len(movable),
            "types": dict(Counter(joint.attrib.get("type", "") for joint in joints)),
            "mimic": mimic_names,
        },
        "root_links": roots,
        "geometry": {
            "visual_count": visual_count,
            "collision_count": collision_count,
            "collision_types": dict(collision_by_type),
            "mesh_reference_count": len(mesh_files),
            "mesh_unique_count": len({entry["resolved"] for entry in mesh_files if entry["resolved"]}),
            "missing_meshes": missing_meshes,
            "mesh_references": mesh_files,
        },
        "xacro_comparison": xacro_comparison,
        "errors": errors,
        "warnings": warnings,
        "status": "PASS" if not errors else "FAIL",
    }
    return result


def markdown_report(result: dict[str, Any]) -> str:
    status = result["status"]
    links = result["links"]
    joints = result["joints"]
    geometry = result["geometry"]
    lines = [
        "# R1 URDF audit",
        "",
        "This report is generated by `src/rpe_robot/audit/urdf_audit.py`. The audit is read-only and does not start ROS nodes or send hardware commands.",
        "",
        f"**Status: {status} (R1 import audit; warnings are listed below)**",
        "",
        "## Source",
        "",
        f"- URDF: `{result['source']}`",
        f"- URDF SHA-256: `{result['source_sha256']}`",
        f"- Robot name: `{result['robot_name']}`",
        f"- Root link: `{', '.join(result['root_links']) or 'none'}`",
        "",
        "## Structure",
        "",
        f"- Links: `{links['count']}`",
        f"- Joints: `{joints['count']}`",
        f"- Movable joints: `{joints['movable_count']}`",
        f"- Joint types: `{joints['types']}`",
        f"- Mimic joints: `{', '.join(joints['mimic']) or 'none'}`",
        f"- Visual elements: `{geometry['visual_count']}`",
        f"- Collision elements: `{geometry['collision_count']}`",
        f"- Collision geometry types: `{geometry['collision_types']}`",
        f"- Mesh references: `{geometry['mesh_reference_count']}` (`{geometry['mesh_unique_count']}` unique paths)",
        "",
        "## Contract-specific interpretation",
        "",
        "- The active policy-controlled arm is the six-joint `right_arm_*` chain.",
        "- The Revo2 hand exposes six independent command values; distal finger joints are URDF mimic joints.",
        "- Ranger wheel/steering joints remain part of the state description, while base policy commands use the existing constrained `/cmd_vel` interface.",
        "- `lift_joint` remains present for visual/model compatibility but is excluded from the active command contract; the research baseline treats the lift as fixed at the selected physical height.",
        "- `right_base_link` is the current palm-center proxy until a dedicated palm-center frame is introduced.",
        "",
        "## Xacro consistency",
        "",
        f"- `{result['xacro_comparison']}`",
        "",
        "## Errors",
        "",
    ]
    lines.extend([f"- {item}" for item in result["errors"]] or ["- None"])
    lines += ["", "## Warnings", ""]
    lines.extend([f"- {item}" for item in result["warnings"]] or ["- None"])
    lines += [
        "",
        "## Scope boundary",
        "",
        "This is a description/import audit. A PASS means the model is structurally importable with no audit errors; it does not establish serial-specific UR calibration, physical mounting measurement, actuator dynamics, collision-free workspace clearance, or physical hardware behavior.",
        "",
    ]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--xacro", type=Path)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--markdown-out", type=Path)
    args = parser.parse_args(argv)
    try:
        result = audit_urdf(args.urdf, xacro_path=args.xacro)
    except Exception as exc:  # provide a useful CLI failure without a traceback
        print(f"URDF audit failed: {exc}", file=sys.stderr)
        return 2
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(result, indent=2) + "\n")
    if args.markdown_out:
        args.markdown_out.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_out.write_text(markdown_report(result))
    print(json.dumps({"status": result["status"], "errors": result["errors"], "warnings": result["warnings"]}, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
