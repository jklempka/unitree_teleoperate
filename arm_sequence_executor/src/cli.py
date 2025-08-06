import threading
import queue
from sequence_manager import SequenceManager
from typing import Tuple, Any
# -----------------------------------------------------------------------------
# CLI / User interaction (separate from control loop)
# -----------------------------------------------------------------------------
class CLI:
    """Blocking console interface that feeds commands into a queue.

    The control loop never blocks on input; we push user requests into a thread-safe queue.
    """
    def __init__(self, seq_mgr: SequenceManager, cmd_queue: "queue.Queue[Tuple[str, Any]]"):
        self._seq_mgr = seq_mgr
        self._q = cmd_queue
        self._thread = threading.Thread(target=self._run, name="CLIThread", daemon=True)
        self._stop = threading.Event()

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        # cannot reliably stop blocking input() in all terminals; user ^C ends.
    
    def show_menu(self) -> None: 
        # new helper
        self._print_menu()

    # internal ------------------------------------------------------------------
    def _print_menu(self) -> None:
        print("\nAvailable sequences:")
        for seq_id, seq in self._seq_mgr.sequences.items():
            print(f"  {seq_id}: {seq.name} ({len(seq.poses)} poses){' [loop]' if seq.loop else ''}")
        print("\nCommands:\n  <number> : Play sequence with given ID\n  list     : Show this menu again\n  quit     : Exit program\n")

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                user_input = input("Enter command: ").strip().lower()
            except EOFError:
                break
            if user_input == "quit":
                self._q.put(("quit", None))
                break
            elif user_input == "list":
                self._print_menu()
                continue
            else:
                try:
                    seq_id = int(user_input)
                    self._q.put(("play", seq_id))
                except ValueError:
                    print("Invalid input! Please enter a number, 'list', or 'quit'.")
