from typing import Dict, Tuple
from dataclasses import dataclass, field
# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
@dataclass
class ControlConfig:
    """Central configuration for gains, rates, and safety limits."""

    control_dt: float = 0.02               # 20 ms control cycle (50 Hz)
    init_duration: float = 3.0             # seconds to ramp to zero during init
    max_velocity: float = 1              # rad/s default velocity cap (global)
    min_move_duration: float = 0.1         # sec minimal segment duration
    kp: float = 60.0                       # position gain//stiffness
    kd: float = 1.5                        # velocity gain//damping
    enable_watchdog_timeout: float = 0.5   # if state not updated in this many sec -> safe hold
    soft_stop_duration: float = 0.1        # ramp-down on shutdown
    dry_run: bool = False                  # simulation-only mode (no hardware writes)
    verbose: bool = True                   # log to stdout
    is_mujocco: bool = False                # if true, use Mujoco-specific settings (e.g. no CRC)
    # Joint limit safety (optional; None to ignore) — pair (min, max) rad
    joint_limits: Dict[int, Tuple[float, float]] = field(default_factory=dict)

    def limit_joint(self, idx: int, q: float) -> float:
        lim = self.joint_limits.get(idx)
        if lim is None:
            return q
        return float(min(max(q, lim[0]), lim[1]))