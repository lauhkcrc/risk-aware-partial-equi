#!/usr/bin/python3
"""Collect a read-only ROS 2 graph snapshot.

Every operation is a ROS CLI query (list/info).  This tool never calls a
service, publishes a topic, sends an action goal, enables a controller, or
opens a hardware device.  It is safe to run in the ``thor`` container even
when the physical robot is powered.

The graph is allowed to be empty: a container using Docker bridge networking
may not see the host's DDS multicast traffic.  In that case the snapshot is
marked ``NO_GRAPH`` instead of being presented as a robot failure.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any


def run_ros(args: list[str], timeout: float) -> dict[str, Any]:
    command = ["ros2", *args]
    try:
        completed = subprocess.run(command, capture_output=True, text=True, timeout=timeout, check=False)
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        return {"command": command, "returncode": None, "stdout": "", "stderr": str(exc), "status": "unavailable"}
    return {
        "command": command,
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
        "status": "ok" if completed.returncode == 0 else "error",
    }


def lines(output: str) -> list[str]:
    return [line.strip() for line in output.splitlines() if line.strip()]


def snapshot(timeout: float, spin_time: float) -> dict[str, Any]:
    # Do not trust a stale ros2 daemon: the robot host has previously shown
    # duplicate/old names in the daemon cache.  These are discovery-only
    # queries and do not invoke a service, action, publisher, or controller.
    discovery = ["--no-daemon", "--spin-time", str(spin_time)]
    queries = {
        "nodes": run_ros(["node", "list", *discovery], timeout),
        "topics": run_ros(["topic", "list", *discovery, "-t"], timeout),
        "services": run_ros(["service", "list", *discovery, "-t"], timeout),
        "actions": run_ros(["action", "list", *discovery, "-t"], timeout),
    }
    nodes = lines(queries["nodes"]["stdout"])
    topics = lines(queries["topics"]["stdout"])
    services = lines(queries["services"]["stdout"])
    actions = lines(queries["actions"]["stdout"])
    # Every ROS 2 process exposes /rosout and /parameter_events even when no
    # application node is present.  Those defaults do not count as a usable
    # robot graph for R0 discovery.
    application_topics = [item for item in topics if not item.startswith(("/rosout ", "/parameter_events "))]
    application_services = [item for item in services if item.split(" ", 1)[0] not in {"/rosout/get_parameters", "/rosout/set_parameters", "/rosout/list_parameters", "/rosout/describe_parameters", "/rosout/get_parameter_types"}]
    graph_available = any(query["status"] == "ok" for query in queries.values()) and bool(nodes or application_topics or application_services or actions)
    return {
        "schema": "rpe.robot.ros_graph_snapshot/v1",
        "captured_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "host": platform.node(),
        "python": sys.version.split()[0],
        "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
        "ros_version": os.environ.get("ROS_VERSION", "unknown"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "unknown"),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        "ros2_path": shutil.which("ros2"),
        "status": "GRAPH_AVAILABLE" if graph_available else "NO_GRAPH",
        "nodes": nodes,
        "topics": topics,
        "services": services,
        "actions": actions,
        "queries": queries,
        "safety": {
            "read_only": True,
            "motion_commands_sent": False,
            "service_calls_sent": False,
            "action_goals_sent": False,
        },
    }


def markdown(data: dict[str, Any]) -> str:
    lines_out = [
        "# R0 ROS graph discovery",
        "",
        "This snapshot was collected with ROS list/info queries only. It never publishes a topic, calls a service, sends an action goal, enables a controller, or commands hardware.",
        "",
        f"**Status: {data['status']}**",
        "",
        f"- Captured (UTC): `{data['captured_at_utc']}`",
        f"- Host: `{data['host']}`",
        f"- ROS: `{data['ros_version']}` / `{data['ros_distro']}`",
        f"- RMW: `{data['rmw_implementation']}`",
        f"- Domain: `{data['ros_domain_id']}`",
        "",
        "## Graph counts",
        "",
        f"- Nodes: `{len(data['nodes'])}`",
        f"- Topics: `{len(data['topics'])}`",
        f"- Services: `{len(data['services'])}`",
        f"- Actions: `{len(data['actions'])}`",
        "",
    ]
    if data["status"] == "NO_GRAPH":
        lines_out.extend([
            "The command ran without a usable graph in this container. The current Docker invocation uses bridge networking, so an empty result is expected when DDS discovery is on the host network. It is not evidence that the robot drivers are absent.",
            "",
        ])
    else:
        lines_out.extend(["## Nodes", "", *[f"- `{item}`" for item in data["nodes"]], "", "## Topics and types", "", *[f"- `{item}`" for item in data["topics"]], ""])
    lines_out.extend([
        "## R0 interpretation",
        "",
        "The checked-in robot manifest contains the known hardware interfaces from the validated Thor records. A live graph snapshot should be regenerated after DDS networking is configured for the container; the snapshot is evidence, not a source of guessed names or limits.",
        "",
    ])
    return "\n".join(lines_out)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--spin-time", type=float, default=1.0)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--markdown-out", type=Path)
    args = parser.parse_args(argv)
    data = snapshot(args.timeout, args.spin_time)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(data, indent=2) + "\n")
    if args.markdown_out:
        args.markdown_out.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_out.write_text(markdown(data))
    print(json.dumps({"status": data["status"], "nodes": len(data["nodes"]), "topics": len(data["topics"])}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
