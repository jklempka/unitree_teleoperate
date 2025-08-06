"""Replay logic for executing recorded arm motions."""
from __future__ import annotations

import time
from typing import Sequence

from .episode_reader import EpisodeReader
from .arm_controller import ArmController


class EpisodeReplayer:
    """Replay dual-arm motions from an :class:`EpisodeReader` using a controller."""

    def __init__(self, episode: EpisodeReader, controller: ArmController, frequency: float | None = None) -> None:
        self.episode = episode
        self.controller = controller
        self.frequency = frequency or episode.frequency

    def replay(self) -> None:
        """Replay the sequence of joint positions and restore initial pose."""
        start_q = self.controller.get_joint_positions()
        dt = 1.0 / self.frequency if self.frequency > 0 else 0.0
        for qpos in self.episode.iter_dual_arm_actions():
            self.controller.send_joint_positions(qpos)
            if dt:
                time.sleep(dt)
        # Return the arm to its initial configuration.
        self.controller.send_joint_positions(start_q)
