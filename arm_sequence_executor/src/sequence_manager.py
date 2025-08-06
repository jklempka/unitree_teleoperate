from typing import Dict, Optional
from sequence_loader import ArmSequence, ArmPose
# -----------------------------------------------------------------------------
# Sequence Manager — manages current sequence index / looping
# -----------------------------------------------------------------------------
class SequenceManager:
    def __init__(self, sequences: Dict[int, ArmSequence]):
        if not sequences:
            raise ValueError("SequenceManager requires at least one sequence.")
        self._sequences = sequences
        self._current_seq: Optional[ArmSequence] = None
        self._pose_idx: int = 0

    # Accessors ------------------------------------------------------------------
    @property
    def sequences(self) -> Dict[int, ArmSequence]:
        return self._sequences

    @property
    def current(self) -> Optional[ArmSequence]:
        return self._current_seq

    @property
    def pose_index(self) -> int:
        return self._pose_idx

    # API -----------------------------------------------------------------------
    def start(self, seq_id: int) -> bool:
        seq = self._sequences.get(seq_id)
        if seq is None:
            return False
        self._current_seq = seq
        self._pose_idx = 0
        return True

    def next_pose(self) -> Optional[ArmPose]:
        if self._current_seq is None: # ← guard
            return None
        if self._pose_idx >= len(self._current_seq.poses):
            # sequence ended
            if self._current_seq.loop:
                self._pose_idx = 0
            else:
                self._current_seq = None
                return None
        pose = self._current_seq.poses[self._pose_idx]
        self._pose_idx += 1
        return pose

    def append_return_pose(self, pose: ArmPose) -> None:
        """Add a one‑shot pose that brings the robot back to where it started."""
        if self._current_seq and self._current_seq.poses[-1] != pose:
            self._current_seq.poses.append(pose)

    def restart_current(self) -> None:
        if self._current_seq is not None:
            self._pose_idx = 0
