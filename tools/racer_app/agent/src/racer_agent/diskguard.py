"""Disk guard: keep the Jetson's disk from filling up under a recording stack.

The pain it fixes: cameras recorded into the bag at about 6 MB/s per camera, the disk reached
100 percent, the recorder died, and the launch shut itself down by design (invariant 5).

`decide` is a pure function of the free space and what is running. The guard sheds load in
order of how little it costs: cameras first (nothing in the command path reads them), then the
driving launch, and only then the whole stack, which is shut down with SIGINT so the bag
closes cleanly instead of being cut by a full disk. Deleting bags is never automatic.
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable

from .config import Thresholds
from .controller import ActionError, Controller

log = logging.getLogger(__name__)


def decide(
    free_gb: float | None,
    t: Thresholds,
    cameras_running: bool,
    drive_running: bool,
    stack_running: bool,
) -> list[str]:
    """Actions to take now, in order: any of stop_cameras, stop_drive, stop_stack."""
    if free_gb is None:
        return []
    actions: list[str] = []
    if free_gb < t.guard_cameras_gb and cameras_running:
        actions.append("stop_cameras")
    if free_gb < t.guard_drive_gb and drive_running:
        actions.append("stop_drive")
    if free_gb < t.guard_stack_gb and stack_running:
        actions.append("stop_stack")
    return actions


class DiskGuard:
    def __init__(self, controller: Controller, interval_s: float = 10.0) -> None:
        self.c = controller
        self.interval_s = interval_s
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def tick(self) -> list[str]:
        snap = self.c.snapshot(fresh=True)
        drive = self.c.drive_mode() is not None if snap.stack_running else False
        actions = decide(
            snap.disk_free_gb,
            self.c.s.thresholds,
            cameras_running=snap.containers.get(self.c.s.camera_container) == "running",
            drive_running=drive,
            stack_running=snap.stack_running,
        )
        done: list[str] = []
        for action in actions:
            detail = f"disk guard: {snap.disk_free_gb:.1f} GiB free"
            try:
                if action == "stop_cameras":
                    self.c.stop_cameras(actor="disk-guard")
                elif action == "stop_drive":
                    self.c.stop_all(actor="disk-guard")
                else:
                    self.c.stop_stack(actor="disk-guard")
            except ActionError as exc:
                self.c.actions.add(f"guard.{action}", False, actor="disk-guard", detail=exc.message)
                continue
            self.c.actions.add(f"guard.{action}", True, actor="disk-guard", detail=detail)
            done.append(action)
        return done

    def _loop(self, on_error: Callable[[Exception], None]) -> None:
        while not self._stop.wait(self.interval_s):
            try:
                self.tick()
            except Exception as exc:  # noqa: BLE001 - the guard must not die
                on_error(exc)

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(
                target=self._loop,
                args=(lambda e: log.exception("disk guard tick failed", exc_info=e),),
                name="disk-guard",
                daemon=True,
            )
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(2.0)
            self._thread = None
