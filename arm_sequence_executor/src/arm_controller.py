from socket import timeout
from typing import Dict, Optional, Callable
import numpy as np
import threading
import time
from ultility import _logger
from config import ControlConfig
from arm_io import IArmIO
from command_builder import CommandBuilder
from trajectory import TrajectoryPlanner, TrajectorySegment
from g1_5joints import G1JointIndex
from sequence_manager import SequenceManager
from sequence_loader import ArmPose
try:
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    
    from unitree_sdk2py.utils.thread import RecurrentThread  # may or may not use
    
    _UNITREE_AVAILABLE = True
except Exception:  # broad import guard
    # Minimal stubs for dry-run mode -------------------------------------------------
    _UNITREE_AVAILABLE = False
    class _MotorStateStub:
        __slots__ = ("q",)
        def __init__(self, q: float = 0.0):
            self.q = q

    class _MotorCmdStub:
        __slots__ = ("tau", "q", "dq", "kp", "kd")
        def __init__(self):
            self.tau = 0.0
            self.q = 0.0
            self.dq = 0.0
            self.kp = 0.0
            self.kd = 0.0

    class unitree_hg_msg_dds__LowState_:
        def __init__(self, n: int = 30):
            self.motor_state = [_MotorStateStub() for _ in range(n)]

    class unitree_hg_msg_dds__LowCmd_:
        def __init__(self, n: int = 30):
            self.motor_cmd = [_MotorCmdStub() for _ in range(n)]
            self.crc = 0

    # alias for naming compatibility
    LowState_ = unitree_hg_msg_dds__LowState_
    LowCmd_ = unitree_hg_msg_dds__LowCmd_

    class CRC:  # dummy
        def Crc(self, *_args, **_kwargs) -> int:
            return 0

    class ChannelPublisher:  # dummy
        def __init__(self, *_args, **_kwargs):
            pass
        def Init(self):
            return True
        def Write(self, *_args, **_kwargs):
            pass

    class ChannelSubscriber:  # dummy
        def __init__(self, *_args, **_kwargs):
            pass
        def Init(self, *_args, **_kwargs):
            return True

    def ChannelFactoryInitialize(*_args, **_kwargs):
        return True

    class MotionSwitcherClient:  # dummy
        def __init__(self, *_args, **_kwargs):
            pass
        def enableArmSdk(self):
            return True
        def disableArmSdk(self):
            return True

    class RecurrentThread:  # fallback; wrapper over threading.Timer-like loop
        def __init__(self, interval: float, target: Callable, name: str = ""):  # pragma: no cover
            self.interval = interval
            self.target = target
            self.name = name or f"RecurrentThread-{id(self)}"
            self._stop = threading.Event()
            self._thread = threading.Thread(target=self._run, name=self.name, daemon=True)
        def _run(self):
            next_t = time.perf_counter()
            while not self._stop.is_set():
                self.target()
                next_t += self.interval
                delay = max(0.0, next_t - time.perf_counter())
                time.sleep(delay)
        def Start(self):
            self._thread.start()
        def Wait(self, timeout: float = None):
            self._stop.set()
            self._thread.join(timeout=timeout)


# -----------------------------------------------------------------------------
# Controller core — runs in control loop thread
# -----------------------------------------------------------------------------
class ArmController:
    """High-level control loop tying state->trajectory->commands."""
    def __init__(self, io: IArmIO, cfg: ControlConfig, cmd_builder: CommandBuilder, planner: TrajectoryPlanner, on_sequence_complete: Callable[[], None]):
        self._io = io
        self._cfg = cfg
        self._builder = cmd_builder
        self._planner = planner

        self._left_idx = np.array(G1JointIndex.left_arm_indices(), dtype=int)
        self._right_idx = np.array(G1JointIndex.right_arm_indices(), dtype=int)
        self._all_indices = np.concatenate([self._left_idx, self._right_idx])
        self._dofs = len(self._all_indices)

        # Control thread state --------------------------------------------------
        self._state_lock = threading.Lock()
        self._running = False
        self._thread: Optional[RecurrentThread] = None
        self._time_in_state = 0.0  # control time accumulator

        # Sequence integration --------------------------------------------------
        self._seq_mgr: Optional[SequenceManager] = None
        self._active_segment: Optional[TrajectorySegment] = None
        self._segment_start_time: float = 0.0
        self._in_hold: bool = False

        # Watchdog
        self._last_cmd_time: float = 0.0
        self._on_seq_complete = on_sequence_complete 
        self._completion_sent = False   

    # Public API ---------------------------------------------------------------
    def attach_sequence_manager(self, mgr: SequenceManager) -> None:
        self._seq_mgr = mgr

    def start(self) -> None:
        if self._running:
            return
        self._io.start()
        ok = self._io.enable_arms()
        if not ok:
            _logger.warning("Controller start: enable_arms() returned False; continuing anyway.")
        self._running = True
        self._time_in_state = 0.0
        self._thread = RecurrentThread(interval=self._cfg.control_dt, target=self._control_step, name="ArmControlThread")
        self._thread.Start()

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._thread:
            self._thread.Wait(timeout=1.0)
        self._safe_soft_stop()
        self._io.disable_arms()
        self._io.stop()
        _logger.info("ArmController stopped.")

    def play_sequence(self, seq_id: int) -> bool:
        """Start playing a sequence by ID."""
        if self._seq_mgr is None:
            _logger.error("No SequenceManager attached; cannot play sequence.")
            return False
        #1) snapshot *before* the sequence starts
        st = self._io.latest_state()
        home_pose = None
        if st is not None:
            vec = self._gather_current_joint_vector(st)
            home_pose = ArmPose(L_ARM=tuple(vec[:5]),
                                R_ARM=tuple(vec[5:]),
                                speed_scale=1.0,
                                hold=0.0)
        # 2) launch user sequence
        ok = self._seq_mgr.start(seq_id)
        if ok and home_pose:
            # Append a return pose to the sequence
            # that brings the robot back to its current position
            self._seq_mgr.append_return_pose(home_pose)
        if ok:
            _logger.info("Starting sequence id %s (%s).", seq_id, self._seq_mgr.current.name)
            # Reset state for new sequence
            self._completion_sent = False
            with self._state_lock:
                self._active_segment = None
                self._in_hold = False
        else:
            _logger.error("Sequence id %s not found.")
        return ok

    # Internal helpers ---------------------------------------------------------
    def _safe_soft_stop(self) -> None:
        """Ramp gains to zero and hold current positions on shutdown."""
        st = self._io.latest_state()
        if st is None:
            return
        builder = self._builder
        cmd = builder.make_cmd(st, {})  # hold current pos
        # gradually reduce gains over soft_stop_duration
        steps = max(1, int(self._cfg.soft_stop_duration / self._cfg.control_dt))
        for i in range(steps):
            alpha = 1.0 - (i + 1) / steps
            for j in range(len(cmd.motor_cmd)):
                cmd.motor_cmd[j].kp = self._cfg.kp * alpha
                cmd.motor_cmd[j].kd = self._cfg.kd * alpha
            builder.enable_flag(cmd, False)
            self._io.write_cmd(cmd)
            time.sleep(self._cfg.control_dt)

    def _control_step(self) -> None:
        """One control tick: read state, update trajectory, send command."""
        st = self._io.latest_state()
        if st is None:
            # no state yet — skip until we get one
            return

        # watchdog for stale state -------------------------------------------------
        age = self._io.last_state_age() if hasattr(self._io, "last_state_age") else 0.0
        if age > self._cfg.enable_watchdog_timeout:
            _logger.warning("State age %.3fs exceeds watchdog; holding current.", age)
            cmd = self._builder.make_cmd(st, {})
            self._builder.enable_flag(cmd, True)
            self._io.write_cmd(cmd)
            return

        # ensure we have an active segment; if not, try to fetch next pose from seq_mgr
        with self._state_lock:
            seg = self._active_segment
            in_hold = self._in_hold
            seg_start = self._segment_start_time

        if seg is None:
            # Check sequence manager for next pose
            pose = self._seq_mgr.next_pose() if self._seq_mgr else None
            if pose is None:
                # hold current posture indefinitely
                cmd = self._builder.make_cmd(st, {})
                self._builder.enable_flag(cmd, True)
                self._io.write_cmd(cmd)
                # call completion hook exactly once
                if (self._seq_mgr and self._seq_mgr.current is None
                    and self._on_seq_complete and not self._completion_sent):
                    self._on_seq_complete()
                    self._completion_sent = True
                return
            # Build new segment --------------------------------------------------
            current = self._gather_current_joint_vector(st)
            target = self._pose_to_vector(pose)
            seg = self._planner.plan(current, target, speed_scale=pose.speed_scale, hold=pose.hold)
            _logger.info("▶ %s: moving to pose %d/%d (%.2fs)",
                          self._seq_mgr.current.name,
                          self._seq_mgr.pose_index,
                          len(self._seq_mgr.current.poses),
                          seg.duration)
            with self._state_lock:
                self._active_segment = seg
                self._segment_start_time = time.perf_counter()
                self._in_hold = False
            in_hold = False
            seg_start = self._segment_start_time

        # If we are in hold window, check if hold expired -----------------------
        now = time.perf_counter()
        if in_hold:
            elapsed_hold = now - seg_start
            if elapsed_hold >= seg.hold:
                # Clear segment to fetch next
                with self._state_lock:
                    self._active_segment = None
                    self._in_hold = False
                # Next tick will handle
            # hold posture
            q_des_map = self._segment_goal_map(seg)
            cmd = self._builder.make_cmd(st, q_des_map)
            self._builder.enable_flag(cmd, True)
            self._io.write_cmd(cmd)
            return

        # We are in motion phase --------------------------------------------------
        t = now - seg_start
        if t >= seg.duration:
            # segment completed -> begin hold if any
            _logger.info("✓ pose %d/%d reached (hold %.2fs)",
                          self._seq_mgr.pose_index,
                          len(self._seq_mgr.current.poses),
                          seg.hold)
            q_goal = seg.goal 
            _logger.debug("    joint targets = %s", np.array2string(q_goal, precision=3))
            if seg.hold > 0:
                with self._state_lock:
                    self._in_hold = True
                    self._segment_start_time = time.perf_counter()  # reuse start for hold timer
            else:
                # no hold; free segment to advance next tick
                with self._state_lock:
                    self._active_segment = None
            q_des_map = self._segment_goal_map(seg)
        else:
            q_vec = seg.sample(t)
            q_des_map = self._vector_to_map(q_vec)

        cmd = self._builder.make_cmd(st, q_des_map)
        self._builder.enable_flag(cmd, True)
        self._io.write_cmd(cmd)
        self._last_cmd_time = now

    # vector/map conversions -----------------------------------------------------
    def _gather_current_joint_vector(self, st: LowState_) -> np.ndarray:
        vals = [st.motor_state[i].q for i in self._all_indices]
        return np.array(vals, dtype=float)

    def _pose_to_vector(self, pose: ArmPose) -> np.ndarray:
        vals = list(pose.L_ARM) + list(pose.R_ARM)
        return np.array(vals, dtype=float)

    def _vector_to_map(self, vec: np.ndarray) -> Dict[int, float]:
        return {int(idx): float(vec[k]) for k, idx in enumerate(self._all_indices)}

    def _segment_goal_map(self, seg: TrajectorySegment) -> Dict[int, float]:
        return self._vector_to_map(seg.goal)
