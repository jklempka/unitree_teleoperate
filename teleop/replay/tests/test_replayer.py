import json
import sys
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
sys.path.append(str(ROOT))
sys.path.append(str(ROOT / "arm_sequence_executor" / "src"))

import numpy as np

from arm_sequence_executor.src.config import ControlConfig
from arm_sequence_executor.src.trajectory import TrajectorySegment

from teleop.replay.episode_reader import EpisodeReader
from teleop.replay.replayer import EpisodeReplayer


class FakeArmController:
    def __init__(self) -> None:
        self.calls: list[list[float]] = []

    def send_joint_positions(self, positions):
        self.calls.append(list(positions))

    def get_joint_positions(self):
        return [0, 0, 0, 0]


class FakePlanner:
    def __init__(self) -> None:
        self.calls: list[tuple[np.ndarray, np.ndarray]] = []

    def plan(self, current, target, _scale, _hold):
        self.calls.append((np.array(current), np.array(target)))
        return TrajectorySegment(
            start=np.array(current), goal=np.array(target), duration=0.1, hold=0.0
        )


def test_replayer_uses_planner_for_transitions(tmp_path: Path) -> None:
    data = {
        "info": {"image": {"fps": 100}},
        "data": [
            {"actions": {"left_arm": {"qpos": [0, 1]}, "right_arm": {"qpos": [2, 3]}}},
            {"actions": {"left_arm": {"qpos": [4, 5]}, "right_arm": {"qpos": [6, 7]}}},
        ],
    }
    path = tmp_path / "episode.json"
    path.write_text(json.dumps(data))

    reader = EpisodeReader(path)
    controller = FakeArmController()
    planner = FakePlanner()
    cfg = ControlConfig(control_dt=0.1, max_velocity=100.0)
    replayer = EpisodeReplayer(reader, controller, cfg=cfg, planner=planner)

    with patch("time.sleep", return_value=None):
        replayer.replay()

    assert controller.calls == [[0, 1, 2, 3], [4, 5, 6, 7], [0, 0, 0, 0]]
    assert len(planner.calls) == 2
    np.testing.assert_allclose(planner.calls[0][1], np.array([0, 1, 2, 3]))
    np.testing.assert_allclose(planner.calls[1][1], np.array([0, 0, 0, 0]))
