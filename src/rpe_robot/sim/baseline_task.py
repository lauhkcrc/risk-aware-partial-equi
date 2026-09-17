"""Minimal R2 baseline task/manager.

This is intentionally a non-learning smoke task.  It proves that one
policy-shaped action can pass through the same bounded base/arm/hand contract,
advance the simulator state, produce observations, and terminate/reset
deterministically.  It is not a navigation benchmark or a physical-motion
controller.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .core import ARM_JOINTS, HAND_INDEPENDENT_JOINTS, RangerCommand, SimulatorCore


@dataclass(frozen=True)
class BaselineAction:
    base: tuple[float, float, float] = (0.0, 0.0, 0.0)
    arm_positions: tuple[float, ...] = (0.0,) * len(ARM_JOINTS)
    hand: tuple[float, ...] = (0.0,) * len(HAND_INDEPENDENT_JOINTS)
    arm_duration_s: float = 0.1


class BaselineTaskManager:
    """A deterministic, bounded one-episode task for integration smoke tests."""

    def __init__(self, *, episode_horizon_s: float = 10.0, dt_s: float = 0.05) -> None:
        if not math.isfinite(episode_horizon_s) or episode_horizon_s <= 0.0:
            raise ValueError("episode_horizon_s must be positive and finite")
        if not math.isfinite(dt_s) or dt_s <= 0.0 or dt_s > 1.0:
            raise ValueError("dt_s must be in (0, 1]")
        self.episode_horizon_s = float(episode_horizon_s)
        self.dt_s = float(dt_s)
        self.core = SimulatorCore()
        self.elapsed_s = 0.0
        self.step_count = 0
        self.done = False

    def reset(self) -> dict[str, object]:
        self.core.reset()
        self.elapsed_s = 0.0
        self.step_count = 0
        self.done = False
        return self.observation()

    def step(self, action: BaselineAction) -> tuple[dict[str, object], float, bool, dict[str, object]]:
        if self.done:
            raise RuntimeError("episode is done; call reset() before step()")
        command = RangerCommand.validate(*action.base)
        self.core.set_base(command)
        self.core.set_arm(ARM_JOINTS, action.arm_positions, action.arm_duration_s)
        self.core.set_hand(action.hand)
        self.core.step(self.dt_s)
        self.elapsed_s += self.dt_s
        self.step_count += 1
        # The baseline's only objective is to remain a valid, bounded rollout;
        # task reward is deliberately separate from future risk costs.
        reward = 0.0
        self.done = self.elapsed_s >= self.episode_horizon_s
        info = {
            "namespace": "/sim/rpe",
            "hardware_access": False,
            "lift_fixed": True,
            "base_mode": self.core.last_base.mode.value,
            "step_count": self.step_count,
        }
        return self.observation(), reward, self.done, info

    def observation(self) -> dict[str, object]:
        state = self.core.state
        return {
            "base_pose": (state.base_x, state.base_y, state.base_yaw),
            "arm_positions": tuple(state.arm_positions),
            "hand_positions": tuple(state.hand_positions),
            "lift_position": state.lift_position,
            "joint_state": self.core.all_joint_state(),
            "sim_namespace": "/sim/rpe",
        }


__all__ = ["BaselineAction", "BaselineTaskManager"]
