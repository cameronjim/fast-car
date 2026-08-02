"""Makes tests/replay_harness's ``racer_replay`` importable from this package without
declaring it as a uv dependency of this pyproject.toml.

``racer_replay`` is a ``package = false``, pytest-``pythonpath``-only project (see
tests/replay_harness/pyproject.toml's own comment) -- it has no ``[build-system]``, so it
is not pip-installable the way sim/racer_gym is, and a uv path dependency on it would fail.
This mirrors the pattern sim/racer_gym/racer_gym/params.py already uses for
tools/gen_params.py (another un-packaged, path-loaded module): add its directory to
``sys.path`` at runtime instead of trying to make it a "real" dependency.

Call :func:`ensure_importable` before the first ``import racer_replay...``; it is
idempotent, so every module that needs ``racer_replay`` can call it unconditionally at
import time without worrying about ordering or double-insertion.
"""

from __future__ import annotations

import sys
from pathlib import Path

# racer_sim_regression/_replay_import.py -> parents[0]=racer_sim_regression,
# parents[1]=tests/sim_regression, parents[2]=tests
_REPLAY_HARNESS_DIR = Path(__file__).resolve().parents[2] / "replay_harness"


def ensure_importable() -> None:
    path = str(_REPLAY_HARNESS_DIR)
    if not _REPLAY_HARNESS_DIR.is_dir():  # pragma: no cover - defensive, layout invariant
        raise ImportError(
            f"expected tests/replay_harness at {_REPLAY_HARNESS_DIR}, not found -- "
            "racer_sim_regression needs it for racer_replay's golden/tolerance engine"
        )
    if path not in sys.path:
        sys.path.insert(0, path)
