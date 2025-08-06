import numpy as np
from dataclasses import dataclass
from config import ControlConfig

# -----------------------------------------------------------------------------
# Trajectory / interpolation utilities
# -----------------------------------------------------------------------------
@dataclass
class TrajectorySegment:
    """Represents a segment moving from start->goal over duration with optional hold."""
    start: np.ndarray  # shape (N,)
    goal: np.ndarray   # shape (N,)
    duration: float    # move time
    hold: float        # post-move hold

    def sample(self, t: float) -> np.ndarray:
        """Return interpolated position at time t within [0, duration].
        Clamps to start/goal outside segment bounds.
        """
        if t <= 0.0:
            return self.start
        if t >= self.duration:
            return self.goal
        ratio = t / self.duration
        return (1.0 - ratio) * self.start + ratio * self.goal

class TrajectoryPlanner:
    """Generates velocity-limited, per-segment linear trajectories.

    This is a simple piecewise-linear planner. For production you might replace with
    trapezoidal or S-curve velocity/acceleration limiting.
    """
    def __init__(self, cfg: ControlConfig, dofs: int):
        self._cfg = cfg
        self._dofs = dofs

    def plan(self, current: np.ndarray, target: np.ndarray, speed_scale: float, hold: float) -> TrajectorySegment:
        # clip arrays
        if current.shape != (self._dofs,) or target.shape != (self._dofs,):
            raise ValueError("TrajectoryPlanner.plan: shape mismatch")

        delta = np.abs(target - current)
        max_delta = float(delta.max())
        vel = self._cfg.max_velocity * speed_scale
        dur = max(max_delta / max(vel, 1e-6), self._cfg.min_move_duration)
        return TrajectorySegment(start=current.copy(), goal=target.copy(), duration=dur, hold=hold)
