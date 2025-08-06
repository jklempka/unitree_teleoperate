import json
import sys
from pathlib import Path
from unittest.mock import patch

sys.path.append(str(Path(__file__).resolve().parents[3]))

from teleop.replay.episode_reader import EpisodeReader
from teleop.replay.replayer import EpisodeReplayer


class FakeArmController:
    def __init__(self) -> None:
        self.calls = []

    def send_joint_positions(self, positions):
        self.calls.append(list(positions))

    def get_joint_positions(self):
        return [9, 9, 9, 9]


def test_replayer_calls_controller(tmp_path: Path) -> None:
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
    replayer = EpisodeReplayer(reader, controller)

    with patch("time.sleep", return_value=None):
        replayer.replay()

    assert controller.calls == [[0, 1, 2, 3], [4, 5, 6, 7], [9, 9, 9, 9]]
