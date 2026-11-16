"""The planner loader on main refuses a bundle of schema 2.1.0 (contract 2.1, brief 47 step 7).

No planner on main implements contract 2.1, and the loader loads schema major 1 only, so a bundle
that says 2.1.0 must be refused with `BundleSchemaVersionMismatch` (invariant 3: never downgrade a
refusal to a warning). This is the real C++ loader (`planner_replay_cli`, built by the l3-and-cpp CI
job, which sets PNP_REPLAY_REQUIRE_CLI=1 so a missing binary is a failure there); anywhere else a
missing binary is a skip with the reason. The check is made on a copy of the committed car
fixture bundle with its schema version edited, the way the C++ loader's own gtest does it, so the
refusal is for the version and for nothing else. The dynamics package has the same test for a
bundle written by its 2.1 writer (training/racer_pnp_dynamics/tests/test_export_v21.py).
"""

from __future__ import annotations

import os
import shutil
from pathlib import Path

import pytest
from pnp_replay import planner_stream

REPO_ROOT = Path(__file__).resolve().parents[2]
FIXTURE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-car-2026-11-12"


def _cli() -> planner_stream.Cli:
    try:
        return planner_stream.find_cli()
    except planner_stream.ReplayError as exc:
        if os.environ.get("PNP_REPLAY_REQUIRE_CLI"):
            raise
        pytest.skip(f"planner_replay_cli not available: {exc}")


@pytest.mark.parametrize("version", ["2.1.0", "2.1.7", "3.0.0"])
def test_the_loader_on_main_refuses_schema_2_1_and_3(tmp_path: Path, version: str) -> None:
    cli = _cli()
    bundle = tmp_path / "bundle"
    shutil.copytree(FIXTURE, bundle)
    manifest = bundle / "manifest.yaml"
    text = manifest.read_text(encoding="utf-8")
    assert "bundle_schema_version: 1.1.0" in text
    manifest.write_text(
        text.replace("bundle_schema_version: 1.1.0", f"bundle_schema_version: {version}"),
        encoding="utf-8",
    )
    with pytest.raises(planner_stream.ReplayError, match="BundleSchemaVersionMismatch"):
        planner_stream.describe_cli(cli, bundle, 1.3, {})


def test_the_unedited_fixture_still_loads(tmp_path: Path) -> None:
    cli = _cli()
    bundle = tmp_path / "bundle"
    shutil.copytree(FIXTURE, bundle)
    described = planner_stream.describe_cli(cli, bundle, 1.3, {})
    assert isinstance(described, dict)
