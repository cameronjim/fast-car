"""(Re)generate the S.6 committed reference golden files under
racer_sim_regression/references/.

NEVER wired into any test or CI job -- regeneration is a deliberate, human-invoked,
out-of-band action (claude-docs/12-testing.md L4: "Golden updates are deliberate...
regenerating a golden file requires a PR note explaining WHY the output changed. A silent
golden refresh is a defect."), exactly like the ``--reason`` requirement this script
delegates to (``racer_replay.golden.regenerate_golden``).

Usage (from this directory, tests/sim_regression/):

    uv run python regenerate_references.py --reason "explanation for the PR"
    uv run python regenerate_references.py --reason "..." --maneuver throttle_step

The reason you pass here is also logged in the loud regeneration banner
(racer_replay.golden.REGENERATE_BANNER) -- put the SAME explanation in the PR description.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Running this file directly (not via `uv run pytest`, which reads pyproject.toml's
# `pythonpath`) needs racer_sim_regression importable by hand -- Python already puts this
# script's own directory (tests/sim_regression/) on sys.path[0] when run directly, so
# `import racer_sim_regression` below resolves without extra work. racer_replay needs its
# own explicit bootstrap (see racer_sim_regression/_replay_import.py's docstring for why).
sys.path.insert(0, str(Path(__file__).resolve().parent))

from racer_sim_regression._replay_import import ensure_importable
from racer_sim_regression.battery import MANEUVERS, REFERENCES_DIR, run_maneuver

ensure_importable()

from racer_replay.golden import regenerate_golden


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--reason",
        required=True,
        help="WHY the references changed; goes in the loud regen banner and MUST also go in the PR",
    )
    parser.add_argument(
        "--maneuver",
        choices=sorted(MANEUVERS),
        default=None,
        help="Regenerate only this maneuver's references (default: all maneuvers)",
    )
    args = parser.parse_args()

    names = [args.maneuver] if args.maneuver else sorted(MANEUVERS)
    REFERENCES_DIR.mkdir(parents=True, exist_ok=True)
    for name in names:
        trajectory, summary, meta = run_maneuver(name)
        regenerate_golden(
            trajectory, REFERENCES_DIR / f"{name}_trajectory.json", reason=args.reason, meta=meta
        )
        regenerate_golden(
            summary, REFERENCES_DIR / f"{name}_summary.json", reason=args.reason, meta=meta
        )
        print(f"regenerated references for maneuver {name!r}")


if __name__ == "__main__":
    main()
