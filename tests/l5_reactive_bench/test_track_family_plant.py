"""L1 tests of track_family_runner's plant-realism options: what --plant-bundle reads from a
dynamics manifest (including the dead band, absent from older manifests) and how the
--plant-speed-deadband / --plant-steer-tau / --plant-command-delay flags override it. No ROS.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pytest
import track_family_runner as runner
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
NOMINAL_BUNDLE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-nominal-v0"


def flags(**kw) -> argparse.Namespace:
    base = {"plant_speed_deadband": None, "plant_steer_tau": None, "plant_command_delay": None}
    return argparse.Namespace(**{**base, **kw})


def bundle_with(tmp_path: Path, **nominal_extra) -> Path:
    manifest = yaml.safe_load((NOMINAL_BUNDLE / "manifest.yaml").read_text())
    manifest["io"]["nominal"].update(nominal_extra)
    (tmp_path / "manifest.yaml").write_text(yaml.safe_dump(manifest))
    return tmp_path


def test_plant_parameters_come_from_io_nominal_and_a_missing_dead_band_is_zero():
    p = runner.plant_parameters(NOMINAL_BUNDLE)
    assert p["plant_speed_gain"] == pytest.approx(1.0)
    assert p["plant_accel_max_mps2"] == pytest.approx(2.0)
    assert p["plant_decel_max_mps2"] == pytest.approx(1.5)
    assert p["plant_speed_deadband_mps"] == 0.0
    assert "plant_steer_tau_s" not in p and "plant_command_delay_s" not in p


def test_plant_parameters_read_the_dead_band_when_the_manifest_has_one(tmp_path):
    p = runner.plant_parameters(bundle_with(tmp_path, speed_deadband_mps=0.15))
    assert p["plant_speed_deadband_mps"] == pytest.approx(0.15)


def test_no_flags_override_nothing():
    assert runner.plant_overrides(flags()) == {}


def test_flags_override_and_an_explicit_zero_counts():
    o = runner.plant_overrides(
        flags(plant_speed_deadband=0.2, plant_steer_tau=0.1, plant_command_delay=0.0)
    )
    assert o == {
        "plant_speed_deadband_mps": 0.2,
        "plant_steer_tau_s": 0.1,
        "plant_command_delay_s": 0.0,
    }


def test_flags_win_over_the_bundle(tmp_path):
    bundle = bundle_with(tmp_path, speed_deadband_mps=0.15)
    merged = {
        **runner.plant_parameters(bundle),
        **runner.plant_overrides(flags(plant_speed_deadband=0.2)),
    }
    assert merged["plant_speed_deadband_mps"] == 0.2
    assert merged["plant_accel_max_mps2"] == pytest.approx(2.0)  # the rest still from the bundle
