
import threading
import math
from typing import Protocol, Optional
from ultility import _logger
from config import ControlConfig
from ultility import TimeSource, PerfCounterTimeSource
try:
    from unitree_sdk2py.core.channel import (
        ChannelPublisher,
        ChannelSubscriber,
    )

    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
        MotionSwitcherClient,
    )
    _UNITREE_AVAILABLE = True
except Exception:  # broad import guard
    # Minimal stubs for dry-run mode -------------------------------------------------
    _UNITREE_AVAILABLE = False

# -----------------------------------------------------------------------------
# Robot I/O abstraction layer
# -----------------------------------------------------------------------------
class IArmIO(Protocol):
    """Protocol for minimal arm IO used by controller."""
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def latest_state(self) -> Optional[LowState_]: ...
    def write_cmd(self, cmd: LowCmd_) -> None: ...
    def enable_arms(self) -> bool: ...
    def disable_arms(self) -> bool: ...

class UnitreeArmIO:
    """Concrete IO using the Unitree SDK2 DDS channel interfaces."""
    def __init__(self, config: ControlConfig, time_source: Optional[TimeSource] = None ):
        self._cfg = config
        self._ts = time_source or PerfCounterTimeSource()
        self._crc = CRC()
        if config.is_mujocco:
            self._publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        else:
            self._publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._state_lock = threading.Lock()
        self._last_state: Optional[LowState_] = None
        self._last_state_time: float = 0.0
        self._motion_switcher = MotionSwitcherClient()
        self._initialized = False

    def _cb_state(self, msg: LowState_):
        with self._state_lock:
            self._last_state = msg
            self._last_state_time = self._ts.now()

    def start(self) -> None:
        if self._cfg.dry_run:
            _logger.info("[IO] Dry-run mode: skipping hardware start.")
            self._initialized = True
            return
        self._publisher.Init()
        self._subscriber.Init(self._cb_state, 10)
        self._initialized = True
        _logger.info("[IO] DDS channels initialized.")

    def stop(self) -> None:
        if self._cfg.dry_run:
            return
        # nothing special; rely on GC? keep explicit for clarity
        _logger.info("[IO] Stopping IO (no active close API in SDK?).")

    def latest_state(self) -> Optional[LowState_]:
        with self._state_lock:
            return self._last_state

    def last_state_age(self) -> float:
        with self._state_lock:
            if self._last_state is None:
                return math.inf
            return self._ts.now() - self._last_state_time

    def write_cmd(self, cmd: LowCmd_) -> None:
        if self._cfg.dry_run:
            return  # skip writes
        # compute CRC before writing
        cmd.crc = self._crc.Crc(cmd)
        #_logger.debug("[IO] Writing command: %s", str(cmd))
        self._publisher.Write(cmd)

    def enable_arms(self) -> bool:
        if self._cfg.dry_run:
            _logger.info("[IO] Dry-run: enable_arms noop.")
            return True
        try:
            ok = self._call_switcher("enableArmSdk")
            if not ok:
                _logger.error("MotionSwitcher enableArmSdk() returned False.")
            return bool(ok)
        except Exception as e:  # pragma: no cover
            _logger.exception("Failed to enable arms: %s", e)
            return False

    def disable_arms(self) -> bool:
        if self._cfg.dry_run:
            _logger.info("[IO] Dry-run: disable_arms noop.")
            return True
        try:
            ok = self._call_switcher("disableArmSdk")
            if not ok:
                _logger.error("MotionSwitcher disableArmSdk() returned False.")
            return bool(ok)
        except Exception as e:  # pragma: no cover
            _logger.exception("Failed to disable arms: %s", e)
            return False

    def _call_switcher(self, api_name: str) -> bool:
        """
        Call a MotionSwitcher API that may return either:
        • bool                                (our shim)
        • (code: int, data) tuple             (official SDK, code==0 = OK)
        """
        fn = getattr(self._motion_switcher, api_name, None)
        if fn is None:
            _logger.error("MotionSwitcher missing %s()", api_name)
            return False

        try:
            ret = fn()
            if isinstance(ret, tuple):          # official SDK style
                return ret[0] == 0              # code == 0 → success
            return bool(ret)                    # shim style
        except Exception as e:
            _logger.exception("%s() raised: %s", api_name, e)
            return False