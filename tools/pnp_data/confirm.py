"""`pnp_data confirm`: a thin wrapper around the dynamics `confirm` command.

It runs `python -m racer_pnp_dynamics.cli confirm --bag B --candidate C --deployed D` in the
dynamics training project and prints its table. The rule (pre-registration v0.3 section 2.4) is
that command's; nothing is decided here. Without a candidate bundle there is nothing to confirm
and the wrapper says so and exits 0.
"""

from __future__ import annotations

import subprocess
import sys
from collections.abc import Callable, Sequence
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PROJECT = REPO / "training" / "racer_pnp_dynamics"

Runner = Callable[[Sequence[str]], int]


def command(
    bag: Path, candidate: Path, deployed: Path, v05: bool = False, runs: Sequence[str] = ()
) -> list[str]:
    """The wrapped command. With `v05` it is the v0.5 form (contract 2.1 candidate, P1 and Y1
    overall and per direction, driving time per direction, share of steps in supported cells,
    optionally on named runs); without, the v0.3 form."""
    return [
        "uv",
        "run",
        "--project",
        str(PROJECT),
        "python",
        "-m",
        "racer_pnp_dynamics.cli_v05" if v05 else "racer_pnp_dynamics.cli",
        "confirm",
        "--bag",
        str(bag),
        "--candidate",
        str(candidate),
        "--deployed",
        str(deployed),
        *[arg for r in runs for arg in ("--run", r)],
    ]


def _run(cmd: Sequence[str]) -> int:  # pragma: no cover - runs the real fit pipeline
    return subprocess.call(list(cmd))


def confirm(
    bag: Path,
    candidate: Path | None,
    deployed: Path | None,
    runner: Runner = _run,
    v05: bool = False,
    runs: Sequence[str] = (),
) -> int:
    """Run the confirmation; returns the exit status of the wrapped command (0 when skipped)."""
    if candidate is None or deployed is None:
        sys.stdout.write(
            "confirm skipped: give --candidate <bundle> and --deployed <bundle> "
            "(model bundles under car-data/pnp/models/); there is nothing to confirm without one\n"
        )
        return 0
    if not Path(bag).exists():
        sys.stdout.write(f"confirm: no such bag {bag}\n")
        return 2
    return runner(command(bag, candidate, deployed, v05, runs))
