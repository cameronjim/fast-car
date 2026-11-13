"""Write config/pnp/fixtures/dyn-car-2026-11-12/, the committed dynamics bundle of the car as fitted
in brief 14: the fitted nominal (with its dead zone) of the committed pointer manifest
training/pnp_models/<model_id>/manifest.yaml, the residual disabled whatever the fit decided, an
empty metrics block and golden rollouts that cross the dead zone. The matched-plant canaries and
the planner tests run on it.

    uv run python -m racer_pnp_dynamics.make_car_fixture [--from POINTER_MANIFEST]

Regenerate it deliberately after any vehicle_params bump or refit: tests/test_fixture.py fails
until the fixture's `vehicle_params` block equals the live config/vehicle_params.yaml.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import yaml

from .export import config_sha256, write_bundle
from .model import Nominal, Residual
from .vehicle import Vehicle, load_vehicle, repo_root

LOG = logging.getLogger(__name__)

MODEL_ID = "dyn-car-2026-11-12"
SOURCE_MODEL_ID = "dyn-v0.1-2026-11-12"
CREATED_UTC = "2026-11-12T00:00:00Z"  # fixed so regeneration is byte for byte reproducible
GIT_SHA = "0" * 40  # not built from a commit
FIXTURE_RELATIVE = Path("config") / "pnp" / "fixtures" / MODEL_ID
SOURCE_RELATIVE = Path("training") / "pnp_models" / SOURCE_MODEL_ID / "manifest.yaml"


def make_fixture(out_dir: Path, vehicle: Vehicle, source_manifest: Path) -> dict:
    source = yaml.safe_load(source_manifest.read_text(encoding="utf-8"))
    nominal = Nominal.from_dict(source["io"]["nominal"])
    v_max = float(source["io"]["validity"]["v_max_mps"])
    return write_bundle(
        out_dir,
        model_id=MODEL_ID,
        created_utc=CREATED_UTC,
        git_sha=GIT_SHA,
        training_config_sha256=config_sha256(
            {"model_id": MODEL_ID, "nominal": nominal.to_dict(), "v_max": v_max}
        ),
        vehicle=vehicle,
        nominal=nominal,
        residual=Residual.disabled(),
        v_max_mps=v_max,
        data=[
            f"fitted nominal of {source['model_id']}, residual disabled, for simulation and tests"
        ],
        metrics={},
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=repo_root() / FIXTURE_RELATIVE)
    parser.add_argument("--from", dest="source", type=Path, default=repo_root() / SOURCE_RELATIVE)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO)
    make_fixture(args.out, load_vehicle(), args.source)
    LOG.info("wrote %s", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
