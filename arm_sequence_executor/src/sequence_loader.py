from typing import Any, Dict, List, Tuple, Sequence, Optional, Union
from dataclasses import dataclass
import json

# -----------------------------------------------------------------------------
# Sequence data model & validation
# -----------------------------------------------------------------------------
@dataclass
class ArmPose:
    L_ARM: Tuple[float, float, float, float, float]
    R_ARM: Tuple[float, float, float, float, float]
    speed_scale: float = 1.0   # multiplier on global max_velocity
    hold: float = 0.0          # seconds to linger after reaching pose

@dataclass
class ArmSequence:
    name: str
    poses: List[ArmPose]
    loop: bool = False

class SequenceValidationError(Exception):
    pass

class SequenceLoader:
    """Loads and validates arm sequences from JSON files.

    Supports file containing either:
    * single sequence object
    * list of sequence objects
    """

    REQUIRED_KEYS = {"sequence_name", "positions"}

    def __init__(self, left_len: int = 5, right_len: int = 5, is_episode: bool = False):
        self.left_len = left_len
        self.right_len = right_len
        self.is_episode = is_episode  # if True, expect episode format

    def load_file(self, path: str) -> Dict[int, ArmSequence]:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError as e:  # rethrow with context
            raise SequenceValidationError(f"Sequence JSON not found: {path}") from e
        except json.JSONDecodeError as e:
            raise SequenceValidationError(f"Invalid JSON in {path}: {e}") from e

        sequences: Dict[int, ArmSequence] = {}
        if self.is_episode:
            if not isinstance(data, dict):
                raise SequenceValidationError("Episode JSON must be a dictionary.")
            seq = self.parse_episode(data)
            sequences[1] = seq
            return sequences
        
        if isinstance(data, list):
            for i, obj in enumerate(data):
                seq = self._parse_sequence_obj(obj)
                sequences[i + 1] = seq
        elif isinstance(data, dict):
            seq = self._parse_sequence_obj(data)
            sequences[1] = seq
        else:
            raise SequenceValidationError("Top-level JSON must be object or array of objects.")
        return sequences

    # -- internal ------------------------------------------------------------------
    def _parse_sequence_obj(self, obj: Dict[str, Any]) -> ArmSequence:
        missing = self.REQUIRED_KEYS - obj.keys()
        if missing:
            raise SequenceValidationError(f"Missing keys in sequence object: {missing}")

        name = str(obj["sequence_name"]) or "unnamed_sequence"
        loop = bool(obj.get("loop", False))
        raw_positions = obj["positions"]
        if not isinstance(raw_positions, list) or len(raw_positions) == 0:
            raise SequenceValidationError(f"Sequence {name}: 'positions' must be a non-empty list.")

        poses: List[ArmPose] = []
        for idx, p in enumerate(raw_positions):
            try:
                L = p["L_ARM"]
                R = p["R_ARM"]
            except KeyError as e:
                raise SequenceValidationError(f"Sequence {name} pos#{idx}: missing {e}.") from e
            if not (isinstance(L, Sequence) and len(L) == self.left_len):
                raise SequenceValidationError(
                    f"Sequence {name} pos#{idx}: L_ARM must be length {self.left_len}."
                )
            if not (isinstance(R, Sequence) and len(R) == self.right_len):
                raise SequenceValidationError(
                    f"Sequence {name} pos#{idx}: R_ARM must be length {self.right_len}."
                )
            speed_scale = float(p.get("speed_scale", 1.0))
            if speed_scale <= 0:
                raise SequenceValidationError(
                    f"Sequence {name} pos#{idx}: speed_scale must be > 0, got {speed_scale}."
                )
            hold = float(p.get("hold", 0.0))
            if hold < 0:
                raise SequenceValidationError(
                    f"Sequence {name} pos#{idx}: hold must be >= 0, got {hold}."
                )
            poses.append(
                ArmPose(
                    L_ARM=tuple(float(x) for x in L),
                    R_ARM=tuple(float(x) for x in R),
                    speed_scale=speed_scale,
                    hold=hold,
                )
            )

        return ArmSequence(name=name, poses=poses, loop=loop)
    
    def parse_episode(self, episode: Dict[str, Any], *, name: Optional[str] = None) -> ArmSequence:
        """Convert recorder *episode* structure into an :pyclass:`ArmSequence`.

        The function only looks at the *actions.left_arm.qpos* and
        *actions.right_arm.qpos* arrays for each frame under ``episode["data"]``.

        * If a qpos array is **longer** than ``left_len`` / ``right_len`` **by
          exactly two elements**, the trailing surplus is silently *dropped*.
        * If it is **shorter** or has some *other* length mismatch a
          :class:`SequenceValidationError` is raised – this guards against
          incorrectly configured pipelines while still allowing the common
          7‑>5 reduction scenario.
        """
        if not isinstance(episode, dict):
            raise SequenceValidationError("Episode root must be a JSON object.")
        if "data" not in episode or not isinstance(episode["data"], list):
            raise SequenceValidationError("Episode must contain a 'data' list with frame objects.")
        if len(episode["data"]) == 0:
            raise SequenceValidationError("Episode contains zero frames – nothing to parse.")

        seq_name = name or episode.get("text", {}).get("goal", "episode_sequence")
        poses: List[ArmPose] = []

        for i, frame in enumerate(episode["data"]):
            if i % 50 != 0:
                continue
            idx = frame.get("idx", "?")
            try:
                qL = frame["states"]["left_arm"]["qpos"]
                qR = frame["states"]["right_arm"]["qpos"]
            except Exception as e:  # noqa: BLE001 – prop‑up combined Key/Type errors uniformly
                raise SequenceValidationError(
                    f"Episode frame {idx}: missing actions.left_arm/right_arm.qpos ({e})."
                ) from e

            # Type coercion & structural sanity
            if not isinstance(qL, Sequence) or not isinstance(qR, Sequence):
                raise SequenceValidationError(
                    f"Episode frame {idx}: qpos must be array‑like sequences."
                )

            qL_t = self._coerce_and_trim(qL, self.left_len, idx, "L_ARM")
            qR_t = self._coerce_and_trim(qR, self.right_len, idx, "R_ARM")

            poses.append(ArmPose(L_ARM=qL_t, R_ARM=qR_t))

        return ArmSequence(name=seq_name, poses=poses, loop=False)
    
    def _validate_vector(
        self,
        vec: Sequence[Any],
        expected_len: int,
        seq_name: str,
        pos_idx: int,
        label: str,
    ) -> None:
        """Common validation logic for positional vectors."""
        if not (isinstance(vec, Sequence) and len(vec) == expected_len):
            raise SequenceValidationError(
                f"Sequence {seq_name} pos#{pos_idx}: {label} must be length {expected_len}."
            )

    def _coerce_and_trim(
        self,
        vec: Sequence[Any],
        expected_len: int,
        frame_idx: int,
        label: str,
    ) -> Tuple[float, ...]:
        """Force *vec* to *expected_len*.

        Parameters
        ----------
        vec : Sequence[Any]
            Numeric container coming from JSON.
        expected_len : int
            Target length.
        frame_idx : int | str
            Index for error messages only.
        label : str
            Human‑readable name pointing to *left* / *right* for diagnostics.
        """
        if len(vec) == expected_len + 2:
            vec = vec[:expected_len]
        if len(vec) != expected_len:
            raise SequenceValidationError(
                f"Episode frame {frame_idx}: {label} expected length {expected_len} (got {len(vec)})."
            )
        return tuple(float(x) for x in vec)
