"""L1: battery B12 (pnp-v0.7-laps-k6-b12.yaml). The constants under their hash; the thresholds,
conditions, run count, validity and fault plan equal B11's (only cases, seeds and the candidate are
new); the guard of `battery_b12`: no X case and no seed 93 or 94 without the freeze file, the seeds
of other attempts (91, 92 included) refused; the K6 switches equal pnp.launch.py's K6."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import battery_b12 as bat
import lap_battery_b11 as b11
import lap_battery_b12 as b12
import pytest

CONST = b12.load_constants()
B11 = b11.load_constants()
REPO = Path(__file__).resolve().parents[2]


def test_the_yaml_is_read_under_its_hash_and_another_file_is_refused(tmp_path) -> None:
    assert CONST.yaml_sha256 == b12.REGISTERED_YAML_SHA256
    other = tmp_path / "x.yaml"
    other.write_text(b12.PREREG_YAML.read_text() + "\n# edited\n")
    with pytest.raises(b12.PreregError, match="SHA-256"):
        b12.load_constants(path=other)


def test_only_cases_seeds_and_candidate_are_new() -> None:
    assert set(CONST.battery_cases) == {"X1", "X2", "X3", "X4"}
    assert CONST.battery_seeds == (93, 94)
    assert not set(CONST.battery_cases) & set(B11.battery_cases)
    assert not set(CONST.battery_cases) & set(CONST.development_case_names)
    assert not set(CONST.battery_seeds) & set(B11.battery_seeds)
    assert not set(CONST.battery_seeds) & set(CONST.development_seeds)
    t12, t11 = dict(CONST.thresholds), dict(B11.thresholds)
    assert t12.pop("s4").startswith("pnp-v0.6-laps-dv1-addendum-1.yaml lap_battery_b11.s4, unchanged")
    assert t12 == t11
    assert CONST.s4 == B11.s4
    assert CONST.common == B11.common
    assert {k: v for k, v in CONST.conditions.items() if k != "a"} == {
        k: v for k, v in B11.conditions.items() if k != "a"
    }
    assert CONST.conditions["a"] == {**B11.conditions["a"], "seeds": [93]}
    assert CONST.raw["runs_planned"] == B11.raw["runs_planned"] == 88
    assert CONST.raw["fault_plan"] == B11.raw["fault_plan"]
    assert {k: v for k, v in CONST.drop_rule.items() if k != "seed"} == {
        k: v for k, v in B11.drop_rule.items() if k != "seed"
    }
    assert CONST.drop_rule["seed"] == 93
    assert CONST.oracle_ready_after_s == {93: 0.0, 94: 6.0}
    assert len(bat.b11_jobs(CONST)) == 88
    assert CONST.th == B11.th


@pytest.mark.parametrize("case,seed", [("X1", 93), ("X4", 94), ("X2", 94)])
def test_a_b12_run_needs_the_freeze(case, seed) -> None:
    assert bat.needs_freeze(CONST, case, seed)


@pytest.mark.parametrize(
    "case,seed", [("X1", 11), ("T", 93), ("W1", 91), ("T", 91), ("T", 92), ("T", 21), ("X1", 91)]
)
def test_mixed_spent_or_unknown_runs_are_refused(case, seed) -> None:
    with pytest.raises(bat.BatteryError):
        bat.needs_freeze(CONST, case, seed)


def test_a_dev_run_needs_no_freeze() -> None:
    assert not bat.needs_freeze(CONST, "T", 11)


def test_an_x_case_does_not_run_without_the_freeze_file(tmp_path) -> None:
    with pytest.raises(bat.BatteryError, match="B12 may not run before the freeze"):
        bat.command_for(
            CONST, "k", "X1", "ccw", 93, 1, tmp_path, tmp_path / "o.json",
            REPO / "config/pnp/fixtures/dyn-car-2026-11-12", tmp_path,
            freeze_path=tmp_path / "missing.json",
        )


def test_k6_switches_with_the_tuned_values() -> None:
    lap, planner = bat.switches_for("k", "K6", bat.TUNED)
    assert lap["bound_hold_after_doubtful"] is True and lap["ttc_warning_zone_laps"] == 100
    assert lap["value_profile_mode"] == "freeze_after_first_clean_lap"
    assert lap["station_uncertainty_rate"] == 0.02 and lap["station_uncertainty_cap_m"] == 0.5
    assert planner == {
        "lap_station_mode": "track",
        "lap_reward_mode": "replace",
        "w_gate_barrier": 10.0,
        "gate_barrier_margin": 1.0,
        "gate_barrier_steps": 4,
        "gate_barrier_predicted": True,
    }
    with pytest.raises(bat.BatteryError):
        bat.switches_for("k", "K3", bat.TUNED)


def test_k6_equals_the_launch_variant() -> None:
    pytest.importorskip("launch")
    path = REPO / "ros_ws/src/racer_bringup/launch/pnp.launch.py"
    spec = importlib.util.spec_from_file_location("pnp_launch_b12", path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    lap, planner = m.DV1_VARIANTS["K6"]
    blap, bplanner = bat.switches_for("k", "K6", bat.TUNED)
    assert bplanner == planner
    for k, v in lap.items():
        assert blap[k] == v, k
    assert set(blap) == set(lap)
