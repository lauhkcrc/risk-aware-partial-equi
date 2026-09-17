"""Simulation-only R2 components.

The modules in this package are deliberately independent of vendor SDKs.  The
ROS bridge exposes the namespaced ``/sim/rpe`` contract and the Isaac module
imports the locked robot description into an Isaac Sim 6.1 scene.
"""

from .core import (
    ARM_JOINTS,
    HAND_INDEPENDENT_JOINTS,
    HAND_MIMIC_JOINTS,
    R2ContractError,
    RangerCommand,
    RangerMode,
    SimulatorCore,
)
from .baseline_task import BaselineAction, BaselineTaskManager

__all__ = [
    "ARM_JOINTS",
    "HAND_INDEPENDENT_JOINTS",
    "HAND_MIMIC_JOINTS",
    "R2ContractError",
    "RangerCommand",
    "RangerMode",
    "SimulatorCore",
    "BaselineAction",
    "BaselineTaskManager",
]
