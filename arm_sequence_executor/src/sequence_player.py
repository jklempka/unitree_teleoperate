from __future__ import annotations

import time
import threading
from typing import Callable

# -----------------------------------------------------------------------------
# Attempt Unitree imports (guarded for dry-run / testing)
# -----------------------------------------------------------------------------
try:

    from unitree_sdk2py.idl.default import (
        unitree_hg_msg_dds__LowCmd_,
        unitree_hg_msg_dds__LowState_,
    )
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
        def Stop(self):
            self._stop.set()
            self._thread.join(timeout=1.0)