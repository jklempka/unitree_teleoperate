# -----------------------------------------------------------------------------
# Logging helpers
# -----------------------------------------------------------------------------
import logging
import time
import sys
from typing import Protocol

_logger = logging.getLogger("G1SequencePlayer")
_logger.setLevel(logging.INFO)
_ch = logging.StreamHandler(sys.stdout)
_ch.setFormatter(logging.Formatter("[%(asctime)s] %(levelname)s: %(message)s"))
_logger.addHandler(_ch)

# -----------------------------------------------------------------------------
# Utility: time source abstraction (testability)
# -----------------------------------------------------------------------------
class TimeSource(Protocol):  # structural type
    def now(self) -> float: ...

class PerfCounterTimeSource:
    def now(self) -> float:
        return time.perf_counter()