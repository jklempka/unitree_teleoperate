from config import ControlConfig
from g1_5joints import G1JointIndex
from typing import Dict
try:
    
    from unitree_sdk2py.idl.default import (
        unitree_hg_msg_dds__LowCmd_,
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    _UNITREE_AVAILABLE = True
except Exception:  # broad import guard
    # Minimal stubs for dry-run mode -------------------------------------------------
    _UNITREE_AVAILABLE = False
# -----------------------------------------------------------------------------
# Command Builder (LowCmd population helper)
# -----------------------------------------------------------------------------
class CommandBuilder:
    """Fill LowCmd messages given desired joint positions and control gains."""
    def __init__(self, cfg: ControlConfig):
        self._cfg = cfg

    def make_cmd(self, state: LowState_, q_des_map: Dict[int, float]) -> LowCmd_:
        """Create a LowCmd message commanding given joint positions.

        Parameters
        ----------
        state : LowState_
            Most recent robot state (used only to dimension arrays; we *do not* read from state
            here except as fallback for others joints).
        q_des_map : Dict[int, float]
            Desired joint positions in radians by joint index.
        """
        n = len(state.motor_state)
        cmd = unitree_hg_msg_dds__LowCmd_()  # allocate fresh
        for j in range(n):
            motor_cmd = cmd.motor_cmd[j]
            motor_cmd.tau = 0.0
            # if we have a desire for this joint, use it; else hold current
            if j in q_des_map:
                q_des = self._cfg.limit_joint(j, q_des_map[j])
            else:
                q_des = state.motor_state[j].q
            motor_cmd.q = q_des
            motor_cmd.dq = 0.0
            motor_cmd.kp = self._cfg.kp
            motor_cmd.kd = self._cfg.kd
        # sentinel enable is handled by higher-level controller
        return cmd

    def enable_flag(self, cmd: LowCmd_, enabled: bool) -> None:
        # According to original script, setting kNotUsedJoint.q to 1 enables Arm SDK; 0 disables.
        # We retain but keep this *in addition* to MotionSwitcher RPC for backward compatibility.
        cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0 if enabled else 0.0
