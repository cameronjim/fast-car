"""Provenance header for S.6's committed reference golden files.

Extends the L4 golden-update discipline (claude-docs/12-testing.md: "Golden updates are
deliberate... A silent golden refresh is a defect") with a record of what actually produced
a reference: the installed racer_gym version, the repo commit (best effort), the
vehicle_params ``meta`` block (so a Phase 3 sysid fit landing is visible in a reference's
own history, not just inferred from a diff), which vehicle_params fields were still
fallback placeholders at generation time (racer_gym/params.py), and the seed. Carried via
racer_replay.golden's optional ``{"meta": ...}`` wrapper
(tests/replay_harness/racer_replay/golden.py) -- see that module's docstring for why the
wrapper is backward compatible.
"""

from __future__ import annotations

import dataclasses
import subprocess
from importlib import metadata as importlib_metadata
from pathlib import Path
from typing import Any

from racer_gym.params import DynParamsResult

# racer_sim_regression/provenance.py -> parents[0]=racer_sim_regression,
# parents[1]=tests/sim_regression, parents[2]=tests, parents[3]=repo root
REPO_ROOT = Path(__file__).resolve().parents[3]


def _racer_gym_version() -> str:
    try:
        return importlib_metadata.version("racer-gym")
    except importlib_metadata.PackageNotFoundError:  # pragma: no cover - defensive
        return "unknown (racer-gym not installed as a distribution)"


def _repo_commit() -> str | None:
    """Best-effort ``git rev-parse HEAD``; ``None`` (never a raised exception) if git is
    unavailable or this isn't a git checkout -- provenance is a nice-to-have record, not
    something that should fail a maneuver run."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=5,
            check=True,
        )
        return result.stdout.strip()
    except Exception:  # noqa: BLE001 - best-effort provenance, never fail a maneuver run over it
        return None


def build_provenance_meta(
    *, seed: int, dyn_params_result: DynParamsResult, vehicle_params: Any
) -> dict[str, Any]:
    return {
        "racer_gym_version": _racer_gym_version(),
        "repo_commit": _repo_commit(),
        "seed": seed,
        "vehicle_params_meta": dataclasses.asdict(vehicle_params.meta),
        "vehicle_params_fallback_flags": dict(dyn_params_result.fallback_flags),
    }
