import json
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[3]))

from teleop.replay.episode_reader import EpisodeReader


def test_iter_dual_arm_actions(tmp_path: Path) -> None:
    data = {
        "info": {"image": {"fps": 10}},
        "data": [
            {"actions": {"left_arm": {"qpos": [0, 1]}, "right_arm": {"qpos": [2, 3]}}},
            {"actions": {"left_arm": {"qpos": [4, 5]}}},
        ],
    }
    path = tmp_path / "episode.json"
    path.write_text(json.dumps(data))

    reader = EpisodeReader(path)
    assert list(reader.iter_dual_arm_actions()) == [[0, 1, 2, 3]]
    assert reader.frequency == 10
