"""L1 check of track_follow.launch.py: the calib-2026-11-10 profile carries the same steering numbers as
tools/track_session/real_car_2026-11-10.yaml (which the offline tools and the sim read)."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
import yaml

_HERE = Path(__file__).resolve().parent
_LAUNCH = _HERE.parent / "launch" / "track_follow.launch.py"
_REAL_CAR = _HERE.parents[3] / "tools" / "track_session" / "real_car_2026-11-10.yaml"


def _module():
    spec = importlib.util.spec_from_file_location("track_follow_launch", _LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_profile_matches_the_measured_car_file():
    if not _REAL_CAR.is_file():  # an installed package has no tools/ next to it
        pytest.skip("tools/track_session/real_car_2026-11-10.yaml not next to this checkout")
    real = yaml.safe_load(_REAL_CAR.read_text())["real_car"]
    profile = _module().CALIB_2026_11_10_PROFILE
    full = real["command_full_lock_rad"]
    assert profile["max_steering_left_rad"] == pytest.approx(real["effective_left_lock_rad"])
    assert profile["max_steering_right_rad"] == pytest.approx(real["effective_right_lock_rad"])
    assert profile["steering_command_per_rad_left"] == pytest.approx(
        full / real["effective_left_lock_rad"]
    )
    assert profile["steering_command_per_rad_right"] == pytest.approx(
        full / real["effective_right_lock_rad"]
    )


def test_profiles_are_none_and_the_calibration_and_a_full_lock_is_a_full_command():
    module = _module()
    assert module.PROFILES["none"] == {}
    p = module.PROFILES["calib-2026-11-10"]
    assert set(p) == {
        "max_steering_left_rad",
        "max_steering_right_rad",
        "steering_command_per_rad_left",
        "steering_command_per_rad_right",
    }
    # A true angle at the lock needs exactly the full-lock command, no more.
    assert p["max_steering_left_rad"] * p["steering_command_per_rad_left"] == pytest.approx(0.4189)
    assert p["max_steering_right_rad"] * p["steering_command_per_rad_right"] == pytest.approx(
        0.4189
    )
