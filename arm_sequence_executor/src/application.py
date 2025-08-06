import sys
import threading
import signal
import queue
import logging
from typing import Optional, Tuple, List, Any
from ultility import _logger
from config import ControlConfig
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from sequence_loader import SequenceLoader, SequenceValidationError
from arm_io import UnitreeArmIO
from arm_controller import ArmController
from trajectory import TrajectoryPlanner
from command_builder import CommandBuilder
from g1_5joints import G1JointIndex
from sequence_manager import SequenceManager
from cli import CLI

try:
    from unitree_sdk2py.core.channel import (
        ChannelPublisher,
        ChannelSubscriber,
        ChannelFactoryInitialize,
    )
    from unitree_sdk2py.idl.default import (
        unitree_hg_msg_dds__LowCmd_,
        unitree_hg_msg_dds__LowState_,
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.utils.thread import RecurrentThread  # may or may not use
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
        MotionSwitcherClient,
    )
    _UNITREE_AVAILABLE = True
except Exception:  # broad import guard
    # Minimal stubs for dry-run mode -------------------------------------------------
    _UNITREE_AVAILABLE = False

# -----------------------------------------------------------------------------
# Application Orchestration
# -----------------------------------------------------------------------------
class Application:
    def __init__(self, json_file: str, nic: Optional[str], cfg: ControlConfig, is_episode: bool = False):
        self._cfg = cfg
        self._json_file = json_file
        self._nic = nic
        if nic in (None, "lo", "localhost"):
            _logger.warning("Using loopback Mujoco interface; no hardware communication possible.")
            self._cfg.is_mujocco = True  # force Mujoco mode
            nic = "lo"
        # Initialize Unitree channel factory (unless dry-run)
        if not cfg.dry_run:
            if nic:
                ChannelFactoryInitialize(0, nic)
            else:
                ChannelFactoryInitialize(0)

        # Load sequences ---------------------------------------------------------
        loader = SequenceLoader( is_episode=is_episode)
        try:
            sequences = loader.load_file(json_file)
        except SequenceValidationError as e:
            _logger.error("Failed to load sequences: %s", e)
            raise SystemExit(1) from e

        # Compose controller stack ----------------------------------------------
        self._io = UnitreeArmIO(cfg)
        dofs = len(G1JointIndex.left_arm_indices()) + len(G1JointIndex.right_arm_indices())
        planner = TrajectoryPlanner(cfg, dofs)
        builder = CommandBuilder(cfg)
        self._seq_mgr = SequenceManager(sequences)
        self._cmd_queue: "queue.Queue[Tuple[str, Any]]" = queue.Queue()
        self._cli = CLI(self._seq_mgr, self._cmd_queue) # CLI first!

        self._controller = ArmController(
        self._io, cfg, builder, planner,
        on_sequence_complete=self._cli.show_menu, # safe now
        )
        self._controller.attach_sequence_manager(self._seq_mgr)
        
        # Shutdown flag
        self._shutdown = threading.Event()

    def run(self) -> None:
        self._install_signal_handlers()

        # Safety print -----------------------------------------------------------
        print("WARNING: Ensure clear workspace around robot before enabling arms.")
        if not self._cfg.dry_run:
            input("Press Enter to continue and enable torque...")

        # Start controller + CLI -------------------------------------------------
        self._controller.start()
        self._cli.start()

        # Poll user commands ----------------------------------------------------
        while not self._shutdown.is_set():
            try:
                cmd, payload = self._cmd_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if cmd == "quit":
                _logger.info("Quit command received.")
                break
            elif cmd == "play":
                seq_id = int(payload)
                ok = self._controller.play_sequence(seq_id)
                if not ok:
                    print(f"Sequence {seq_id} not found.")
            else:  # unknown
                _logger.warning("Unknown command: %s", cmd)

        # graceful shutdown ------------------------------------------------------
        self.shutdown()

    def shutdown(self) -> None:
        if self._shutdown.is_set():
            return
        self._shutdown.set()
        _logger.info("Shutting down application...")
        self._cli.stop()
        self._controller.stop()
        print("Application exited cleanly.")

    # Signals -------------------------------------------------------------------
    def _install_signal_handlers(self) -> None:
        def _sig_handler(signum, _frame):
            _logger.info("Signal %s received; shutting down.", signum)
            self.shutdown()
        signal.signal(signal.SIGINT, _sig_handler)
        signal.signal(signal.SIGTERM, _sig_handler)

# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

def _parse_args(argv: List[str]) -> Tuple[str, Optional[str], ControlConfig]:
    import argparse
    parser = argparse.ArgumentParser(description="Unitree G1 Arm Sequence Player (Refactored)")
    parser.add_argument("json_file", help="Path to sequence JSON file")
    parser.add_argument("nic", nargs="?", default=None, help="Network interface name (optional)")
    parser.add_argument("--dry-run", action="store_true", help="Run without hardware (simulation mode)")
    parser.add_argument("--max-vel", type=float, default=0.5, help="Global max joint velocity rad/s")
    parser.add_argument("--kp", type=float, default=60.0, help="Joint position gain")
    parser.add_argument("--kd", type=float, default=1.5, help="Joint velocity gain")
    parser.add_argument(
        "--episode",
        "--episode",
        action="store_true",
        help="Input json file is in episode format (single sequence with multiple poses)",
    )
    parser.add_argument("--log", default="info", choices=["debug", "info", "warning", "error"], help="Log level")
    args = parser.parse_args(argv)

    level = {
        "debug": logging.DEBUG,
        "info": logging.INFO,
        "warning": logging.WARNING,
        "error": logging.ERROR,
    }[args.log]
    _logger.setLevel(level)

    cfg = ControlConfig(
        dry_run=args.dry_run or not _UNITREE_AVAILABLE,
        max_velocity=args.max_vel,
        kp=args.kp,
        kd=args.kd,
    )
    return args.json_file, args.nic, cfg, args.episode


def main(argv: Optional[List[str]] = None) -> None:
    if argv is None:
        argv = sys.argv[1:]
    json_file, nic, cfg, is_episode = _parse_args(argv)
    app = Application(json_file, nic, cfg, is_episode=is_episode)
    app.run()


if __name__ == "__main__":  # pragma: no cover
    main()