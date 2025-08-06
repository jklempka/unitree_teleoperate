"""Command line interface for replaying recorded episodes."""
from __future__ import annotations

import argparse

from .episode_reader import EpisodeReader
from .arm_controller import DDSArmController
from .replayer import EpisodeReplayer


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay a recorded arm sequence")
    parser.add_argument("episode", help="Path to the episode data.json file")
    parser.add_argument(
        "--robot",
        default="real",
        choices=["real", "mujoco"],
        help="Target platform for commands",
    )
    args = parser.parse_args()

    episode = EpisodeReader(args.episode)
    controller = DDSArmController(
        motion_mode=args.robot == "real",
        simulation_mode=args.robot == "mujoco",
    )

    replayer = EpisodeReplayer(episode, controller)
    replayer.replay()


if __name__ == "__main__":  # pragma: no cover - CLI
    main()
