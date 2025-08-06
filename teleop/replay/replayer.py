"""Replay logic for executing recorded arm motions."""
from __future__ import annotations

import time
from typing import Sequence

import numpy as np

from arm_sequence_executor.src.config import ControlConfig
from arm_sequence_executor.src.trajectory import TrajectoryPlanner

from .episode_reader import EpisodeReader
from .arm_controller import ArmController


class EpisodeReplayer:
    """Replay dual-arm motions from an :class:`EpisodeReader` using a controller."""

    def __init__(
        self,
        episode: EpisodeReader,
        controller: ArmController,
        frequency: float | None = None,
        cfg: ControlConfig | None = None,
        planner: TrajectoryPlanner | None = None,
    ) -> None:
        self.episode = episode
        self.controller = controller
        self.frequency = frequency or episode.frequency
        self.cfg = cfg or ControlConfig()
        self.planner = planner

    def _move_between(self, start: Sequence[float], goal: Sequence[float]) -> None:
        """Move the controller smoothly from ``start`` to ``goal`` joints."""

        if self.planner is None:
            self.planner = TrajectoryPlanner(self.cfg, len(start))
        segment = self.planner.plan(np.asarray(start), np.asarray(goal), 1.0, 0.0)
        steps = max(1, int(segment.duration / self.cfg.control_dt))
        for step in range(1, steps + 1):
            t = min(segment.duration, step * self.cfg.control_dt)
            qpos = segment.sample(t)
            self.controller.send_joint_positions(qpos)
            time.sleep(self.cfg.control_dt)

    def replay(self) -> None:
        """Replay the sequence of joint positions and restore initial pose."""
        start_q = np.asarray(self.controller.get_joint_positions(), dtype=float)
        actions = iter(self.episode.iter_dual_arm_actions())
        try:
            first_q = np.asarray(next(actions), dtype=float)
        except StopIteration:
            return

        self._move_between(start_q, first_q)
        dt = 1.0 / self.frequency if self.frequency > 0 else 0.0
        if dt:
            time.sleep(dt)

        last_q = first_q
        for qpos in actions:
            self.controller.send_joint_positions(qpos)
            last_q = np.asarray(qpos, dtype=float)
            if dt:
                time.sleep(dt)

        # Return the arm to its initial configuration.
        self._move_between(last_q, start_q)
