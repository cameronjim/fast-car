"""Write the committed dyn-v0.2 fixtures from an exported dyn-v0.2 bundle:

- config/pnp/fixtures/dyn-car-2026-11-13/           the fitted nominal, residual disabled (small)
- config/pnp/fixtures/dyn-car-2026-11-13-residual/  the same nominal with the residual ENABLED, so
  the C++ model's residual path has a real golden file in CI (only when the residual shipped)

    uv run python -m racer_pnp_dynamics.make_v02_fixtures --bundle <car-data/pnp/models/dyn-v0.2-...>

`dyn-nominal-v0` and `dyn-car-2026-11-12` are never touched (the planner tests and canaries read
them). Regenerate deliberately after a vehicle_params bump: tests/test_fixture_v02.py fails until
the fixture's `vehicle_params` block equals the live config/vehicle_params.yaml.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

from .export import Bundle, config_sha256, read_bundle, write_bundle
from .model import Residual
from .vehicle import Vehicle, load_vehicle, repo_root

LOG = logging.getLogger(__name__)

NOMINAL_ID = "dyn-car-2026-11-13"
RESIDUAL_ID = "dyn-car-2026-11-13-residual"
CREATED_UTC = "2026-11-13T00:00:00Z"  # fixed so regeneration is byte for byte reproducible
GIT_SHA = "0" * 40  # not built from a commit
FIXTURES_RELATIVE = Path("config") / "pnp" / "fixtures"


def _write(
    out_dir: Path, vehicle: Vehicle, source: Bundle, model_id: str, residual: Residual, note: str
) -> dict:
    nominal = source.nominal
    return write_bundle(
        out_dir,
        model_id=model_id,
        created_utc=CREATED_UTC,
        git_sha=GIT_SHA,
        training_config_sha256=config_sha256(
            {
                "model_id": model_id,
                "nominal": nominal.to_dict(),
                "v_max": source.v_max_mps,
                "residual": bool(residual.enabled),
            }
        ),
        vehicle=vehicle,
        nominal=nominal,
        residual=residual,
        v_max_mps=source.v_max_mps,
        data=[f"{note} of {source.manifest['model_id']}, for simulation and tests"],
        metrics={},
        golden_residual_rollouts=bool(residual.enabled),
    )


def make_fixtures(root: Path, vehicle: Vehicle, bundle_dir: Path) -> list[Path]:
    """The nominal fixture, and the residual fixture when the source bundle has the residual
    enabled. Returns the directories written."""
    source = read_bundle(bundle_dir)
    out = []
    nominal_dir = root / FIXTURES_RELATIVE / NOMINAL_ID
    _write(nominal_dir, vehicle, source, NOMINAL_ID, Residual.disabled(), "fitted nominal")
    out.append(nominal_dir)
    if source.residual.enabled:
        residual_dir = root / FIXTURES_RELATIVE / RESIDUAL_ID
        _write(
            residual_dir, vehicle, source, RESIDUAL_ID, source.residual,
            "fitted nominal with the residual enabled",
        )  # fmt: skip
        out.append(residual_dir)
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--root", type=Path, default=repo_root())
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO)
    for d in make_fixtures(args.root, load_vehicle(), args.bundle):
        LOG.info("wrote %s", d)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
