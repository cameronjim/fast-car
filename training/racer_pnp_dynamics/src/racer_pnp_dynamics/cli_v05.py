"""Real-bag entry points of dyn-v0.5 (thin glue, needs the bags; brief 47).

    python -m racer_pnp_dynamics.cli_v05 mb5 --date 2026-11-16
    python -m racer_pnp_dynamics.cli_v05 confirm --bag <dir> --candidate <bundle> --deployed <bundle>

Stage commands live in `stages_v05`. The v0.5 constants file is checked against its pre-registered
SHA-256 on every run (`ground_motion_v21.load_constants_v05`).
"""

from __future__ import annotations

import logging
from pathlib import Path

DATA_ROOT = Path("/Users/cameronjim/code/car-data")


def commands() -> dict:  # pragma: no cover - real data
    from .stages_v05 import STAGE_COMMANDS

    return STAGE_COMMANDS


def main(argv: list[str]) -> int:  # pragma: no cover - real data
    import argparse

    cmds = commands()
    parser = argparse.ArgumentParser(prog="racer_pnp_dynamics.cli_v05")
    parser.add_argument("command", choices=tuple(cmds))
    parser.add_argument("--bags", type=Path, default=DATA_ROOT / "bags")
    parser.add_argument("--models", type=Path, default=DATA_ROOT / "pnp" / "models")
    parser.add_argument("--cache", type=Path, default=DATA_ROOT / "pnp" / "cache")
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--date", default="2026-11-16", help="the date of the output directory")
    parser.add_argument("--freeze-sha", default=None, help="freeze-eval: the freeze commit")
    parser.add_argument("--bag", type=Path, default=None, help="confirm: the session bag")
    parser.add_argument("--candidate", type=Path, default=None, help="confirm: contract 2.1 bundle")
    parser.add_argument("--deployed", type=Path, default=None, help="confirm: deployed bundle")
    parser.add_argument(
        "--run", action="append", default=[], help="confirm: a named PnP run (repeatable)"
    )
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO)
    return cmds[args.command](args)


if __name__ == "__main__":  # pragma: no cover
    import sys

    raise SystemExit(main(sys.argv[1:]))
