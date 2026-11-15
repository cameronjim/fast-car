"""L1/L2 tests of tools/lapref: the method on synthetic runs with a known lap length, speed
changes, a shrinking line, noise and a stall; a run with no period; properties of the output; the
bag signals and the command line."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lapref import cli as lapref_cli
from lapref import method
from lapref.bag import TOPICS, Signals
from lapref.method import MIN_RETURNED_LAP_M, lapref

try:
    from hypothesis import HealthCheck, given, settings
    from hypothesis import strategies as st
except ModuleNotFoundError:  # pragma: no cover - the CI image has hypothesis
    given = None


def synth(
    n_laps: int = 8,
    lap_m: float = 5.8,
    shrink: float = 0.0,
    noise: float = 0.03,
    stall: tuple[float, float] | None = None,
    speed_step: bool = True,
    seed: int = 0,
    odom_hz: float = 50.0,
    drive_hz: float = 40.0,
) -> tuple[Signals, np.ndarray]:
    """A run of `n_laps` laps whose length shrinks by `shrink` per lap, on a launch ramp, with a
    speed step and an optional stall (speed 0 for t in `stall`). Returns the signals and the
    true lap times between the line at 1.0 m (lapref's own start) and its repeats."""
    rng = np.random.default_rng(seed)
    lens = np.array([lap_m * (1.0 - shrink) ** k for k in range(n_laps)])
    edges = np.concatenate([[0.0], np.cumsum(lens)])
    total = edges[-1] + 0.3 * lens[-1]
    dt, t, s = 1.0 / odom_hz, 0.0, 0.0
    ts, vs, ss = [], [], []
    while s < total:
        v = min(1.0 + (0.3 if speed_step and s > 20.0 else 0.0), 0.4 + 0.6 * min(t / 1.5, 1.0))
        if stall is not None and stall[0] <= t < stall[1]:
            v = 0.0
        ts.append(t)
        vs.append(v)
        ss.append(s)
        s += v * dt
        t += dt
    ts_a, vs_a, ss_a = np.array(ts), np.array(vs), np.array(ss)
    td = np.arange(0.0, ts_a[-1], 1.0 / drive_hz)
    sd = np.interp(td, ts_a, ss_a)
    k = np.clip(np.searchsorted(edges, sd, side="right") - 1, 0, n_laps - 1)
    phase = (sd - edges[k]) / lens[k]
    steer = (
        0.25 * np.sin(2 * np.pi * phase)
        + 0.12 * np.sin(4 * np.pi * phase + 1.0)
        + 0.08 * np.sign(np.sin(6 * np.pi * phase + 0.3))
        + rng.normal(0.0, noise, len(td))
    )
    # the line lapref starts on is at 1.0 m, that is at phase 1.0 / lens[0] of the first lap
    line = edges[:-1] + (1.0 / lens[0]) * lens
    line = np.append(line, edges[-1] + (1.0 / lens[0]) * lens[-1] * (1.0 - shrink))
    line = line[line < ss_a[-1]]
    # first time of reaching each line (the stall is a plateau of ss)
    s_u, first = np.unique(ss_a, return_index=True)
    truth = np.diff(np.interp(line, s_u, ts_a[first]))
    return Signals(td, steer, ts_a, vs_a), truth


def run_lapref(sig: Signals):
    return lapref(sig.odom_t, sig.odom_v, sig.steer_t, sig.steer)


@pytest.mark.parametrize(
    "kwargs",
    [
        {},
        {"shrink": 0.02, "n_laps": 6},
        {"stall": (15.0, 18.0)},
        {"shrink": 0.02, "n_laps": 6, "stall": (12.0, 16.0), "noise": 0.04},
        {"n_laps": 3},
        {"n_laps": 12, "shrink": 0.01, "noise": 0.05, "seed": 3},
        {"speed_step": False, "lap_m": 6.1, "seed": 5},
    ],
)
def test_synthetic_runs_have_exact_counts_and_times(kwargs) -> None:
    sig, truth = synth(**kwargs)
    result = run_lapref(sig)
    assert result.n_laps == len(truth), (result.n_laps, len(truth), result.reason)
    assert np.max(np.abs(np.array(result.lap_times_s) - truth)) < 0.05
    assert result.autocorr_peak is not None and result.autocorr_peak >= method.MIN_PEAK
    assert len(result.match_scores) == result.n_laps
    assert all(score > 0.5 for score in result.match_scores)


def test_lap_length_is_the_autocorrelation_lag() -> None:
    sig, _ = synth(n_laps=10)
    result = run_lapref(sig)
    assert result.lap_length_m == pytest.approx(5.8, abs=0.03)
    assert all(length == pytest.approx(5.8, abs=0.05) for length in result.lap_lengths_m)
    assert result.run_distance_m > 10 * 5.8


def test_a_straight_run_has_no_laps() -> None:
    rng = np.random.default_rng(1)
    t = np.arange(0.0, 60.0, 0.02)
    sig = Signals(t, 0.01 * rng.normal(size=len(t)) + 0.02, t, np.full(len(t), 1.0))
    result = run_lapref(sig)
    assert result.n_laps == 0
    assert "no periodic lap found" in result.reason
    assert result.lap_times_s == []


def test_a_constant_steering_has_no_laps() -> None:
    t = np.arange(0.0, 60.0, 0.02)
    result = run_lapref(Signals(t, np.full(len(t), 0.3), t, np.full(len(t), 1.0)))
    assert result.n_laps == 0
    assert result.autocorr_peak is None or result.autocorr_peak < method.MIN_PEAK


def test_a_drifting_steering_has_no_laps() -> None:
    t = np.arange(0.0, 80.0, 0.02)
    drift = 0.3 * np.sin(2 * np.pi * t / 300.0)  # one slow swing, not a lap
    result = run_lapref(Signals(t, drift, t, np.full(len(t), 1.0)))
    assert result.n_laps == 0


def test_a_run_shorter_than_the_dropped_metre() -> None:
    t = np.arange(0.0, 1.0, 0.02)
    result = run_lapref(Signals(t, np.sin(t), t, np.full(len(t), 0.5)))
    assert result.n_laps == 0
    assert "not longer than" in result.reason


def test_a_run_too_short_for_a_lap_range() -> None:
    t = np.arange(0.0, 4.0, 0.02)
    result = run_lapref(Signals(t, np.sin(5 * t), t, np.full(len(t), 1.0)))
    assert result.n_laps == 0


def test_a_standing_car_has_no_laps() -> None:
    t = np.arange(0.0, 30.0, 0.02)
    result = run_lapref(Signals(t, np.sin(t), t, np.zeros(len(t))))
    assert result.n_laps == 0


@pytest.mark.parametrize(
    ("odom_t", "steer_t"),
    [
        ([], [0.0, 1.0]),
        ([0.0, 1.0], []),
        ([0.0], [0.0, 1.0]),
        ([float("nan")] * 3, [0.0, 1.0, 2.0]),
    ],
)
def test_degenerate_inputs_return_no_laps(odom_t, steer_t) -> None:
    result = lapref(odom_t, [1.0] * len(odom_t), steer_t, [0.1] * len(steer_t))
    assert result.n_laps == 0
    assert result.reason


def test_unsorted_and_nan_samples_are_cleaned() -> None:
    sig, truth = synth(n_laps=5)
    order = np.random.default_rng(2).permutation(len(sig.steer_t))
    steer = sig.steer.copy()
    steer[::97] = np.nan
    result = lapref(sig.odom_t, sig.odom_v, sig.steer_t[order], steer[order])
    assert result.n_laps == len(truth)


def test_an_odometry_dropout_adds_no_distance() -> None:
    t = np.array([0.0, 0.5, 1.0, 5.0, 5.5])  # a 4 s gap
    s = method.wheel_distance(t, np.ones(5))
    assert s[-1] == pytest.approx(1.5)
    assert method.wheel_distance(np.zeros(0), np.zeros(0)).size == 0


def test_result_dict_is_json_ready() -> None:
    sig, _ = synth(n_laps=4)
    d = run_lapref(sig).to_dict()
    json.dumps(d)
    assert d["n_laps"] == 4
    assert d["mean_lap_time_s"] == pytest.approx(np.mean(d["lap_times_s"]))
    assert (
        run_lapref(Signals(np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0))).to_dict()[
            "mean_lap_time_s"
        ]
        is None
    )


def test_parabola_and_pearson_edges() -> None:
    assert method._parabola(1.0, 1.0, 1.0) == (0.0, 1.0)  # flat: no vertex
    delta, height = method._parabola(0.0, 1.0, 0.5)
    assert 0.0 < delta <= 0.5 and height >= 1.0
    assert method._pearson(np.ones(5), np.arange(5.0)) == 0.0


def test_a_match_with_no_coverage_is_none() -> None:
    x = np.sin(np.linspace(0.0, 30.0, 200))
    template = np.sin(np.linspace(0.0, 6.0, 290))
    assert method.match_crossing(1.0, x, template, 5.8, 500.0) is None


def test_the_tracker_stops_when_a_lap_cannot_be_matched() -> None:
    x = np.sin(np.linspace(0.0, 12.0, 120))
    template = np.sin(np.linspace(0.0, 6.0, 290))
    bounds, scores = method.track(1.0, x, template, 5.8)
    assert bounds == [1.0] and scores == []


def test_the_reader_asks_for_the_steering_and_the_wheels_only() -> None:
    assert TOPICS == ("/drive", "/odom/wheel")


def test_signals_window_and_roundtrip(tmp_path: Path) -> None:
    sig, _ = synth(n_laps=3)
    w = sig.window(2.0, 4.0)
    assert w.steer_t.min() >= 2.0 and w.steer_t.max() <= 4.0
    assert w.odom_t.min() >= 2.0 and w.odom_t.max() <= 4.0
    sig.save(tmp_path / "s.npz")
    back = Signals.load(tmp_path / "s.npz")
    assert np.array_equal(back.steer, sig.steer) and np.array_equal(back.odom_v, sig.odom_v)


# -- command line -----------------------------------------------------------------------------------


def test_cli_run_from_a_cached_bag(tmp_path: Path, capsys) -> None:
    sig, truth = synth(n_laps=5)
    cache = tmp_path / "cache"
    cache.mkdir()
    sig.save(cache / "bagX.lapref-signals.npz")
    out = tmp_path / "out.json"
    status = lapref_cli.main(
        ["/nowhere/bagX", "--run", "0", "1000", "--cache", str(cache), "--out", str(out)]
    )
    assert status == 0
    printed = json.loads(capsys.readouterr().out)
    assert printed["n_laps"] == len(truth)
    assert printed["role"] is None and printed["git_head"] is None
    assert json.loads(out.read_text())["n_laps"] == len(truth)


def test_cli_exit_status_is_1_without_laps(tmp_path: Path, capsys) -> None:
    t = np.arange(0.0, 60.0, 0.02)
    Signals(t, np.full(len(t), 0.1), t, np.full(len(t), 1.0)).save(
        tmp_path / "b.lapref-signals.npz"
    )
    assert lapref_cli.main(["/x/b", "--run", "0", "60", "--cache", str(tmp_path)]) == 1
    assert json.loads(capsys.readouterr().out)["n_laps"] == 0


def test_cli_reads_the_bag_when_nothing_is_cached(tmp_path: Path, monkeypatch, capsys) -> None:
    sig, _ = synth(n_laps=4)
    calls = []

    def fake_read(bag: Path) -> Signals:
        calls.append(bag)
        return sig

    monkeypatch.setattr(lapref_cli, "read_signals", fake_read)
    cache = tmp_path / "c"
    assert lapref_cli.main(["/x/bagY", "--run", "0", "500", "--cache", str(cache)]) == 0
    assert calls == [Path("/x/bagY")]
    assert (cache / "bagY.lapref-signals.npz").is_file()
    capsys.readouterr()
    assert lapref_cli.main(["/x/bagY", "--run", "0", "500", "--cache", str(cache)]) == 0
    assert len(calls) == 1  # the second run read the cache


def test_cli_rejects_a_reversed_window() -> None:
    sig, _ = synth(n_laps=3)
    with pytest.raises(ValueError, match="before it starts"):
        lapref_cli.laps_of_run(sig, 10.0, 5.0)


def test_cli_split_run_roles(tmp_path: Path, monkeypatch) -> None:
    pytest.importorskip("racer_eval")
    yaml_file = Path(__file__).resolve().parents[2] / "evaluation" / "prereg" / "pnp-v0.4.yaml"
    if not yaml_file.is_file():
        pytest.skip("pnp-v0.4.yaml is not in this checkout yet")
    bag_dev = Path("/x/2026-11-14T21-49-21_car_teleop")
    start, end, role = lapref_cli.resolve_split_run("L0", bag_dev, scoring_test=False)
    assert (start, end, role) == (124.9, 239.9, "dev")
    with pytest.raises(ValueError, match="is in bag"):
        lapref_cli.resolve_split_run("L0", Path("/x/other"), scoring_test=False)
    bag_test = Path("/x/2026-11-14T21-37-23_car_teleop")
    with pytest.raises(lapref_cli.TestRunRefused):
        lapref_cli.resolve_split_run("K1", bag_test, scoring_test=False)
    assert lapref_cli.resolve_split_run("K1", bag_test, scoring_test=True)[2] == "test"


def test_cli_split_run_records_the_git_head_for_a_test_run(
    tmp_path: Path, monkeypatch, capsys
) -> None:
    sig, _ = synth(n_laps=4)
    sig.save(tmp_path / "bagZ.lapref-signals.npz")
    monkeypatch.setattr(
        lapref_cli, "resolve_split_run", lambda name, bag, scoring: (0.0, 500.0, "test")
    )
    monkeypatch.setattr(lapref_cli, "_git_head", lambda: "abc123")
    assert (
        lapref_cli.main(
            ["/x/bagZ", "--split-run", "K1", "--scoring-test", "--cache", str(tmp_path)]
        )
        == 0
    )
    printed = json.loads(capsys.readouterr().out)
    assert printed["git_head"] == "abc123" and printed["split_run"] == "K1"


def test_git_head_in_a_repo_and_outside(tmp_path: Path, monkeypatch) -> None:
    assert len(lapref_cli._git_head()) in (40, len("unknown"))
    monkeypatch.setenv("PATH", str(tmp_path))
    assert lapref_cli._git_head() == "unknown"


def test_entry_script_runs() -> None:
    import subprocess

    script = Path(__file__).resolve().parents[1] / "lapref.py"
    out = subprocess.run(
        [sys.executable, str(script), "--help"], capture_output=True, text=True, check=False
    )
    assert out.returncode == 0 and "--split-run" in out.stdout


# -- L2: properties ---------------------------------------------------------------------------------

if given is not None:

    @settings(max_examples=60, deadline=None, suppress_health_check=[HealthCheck.too_slow])
    @given(
        seed=st.integers(0, 10_000),
        n=st.integers(2, 400),
        speed=st.floats(0.0, 3.0),
        noise=st.floats(0.0, 2.0),
        period=st.floats(0.5, 30.0),
    )
    def test_property_laps_are_bounded_and_finite(seed, n, speed, noise, period) -> None:
        rng = np.random.default_rng(seed)
        t_odom = np.cumsum(rng.uniform(0.005, 0.2, size=n))
        v = np.abs(speed * (1.0 + 0.5 * rng.normal(size=n)))
        t_steer = np.sort(rng.uniform(0.0, t_odom[-1], size=n))
        steer = np.sin(2 * np.pi * t_steer * speed / period) + noise * rng.normal(size=n)
        result = lapref(t_odom, v, t_steer, steer)
        assert result.n_laps == len(result.lap_times_s) == len(result.lap_lengths_m)
        assert result.n_laps <= result.run_distance_m / MIN_RETURNED_LAP_M + 1e-9
        assert all(length >= MIN_RETURNED_LAP_M - 1e-9 for length in result.lap_lengths_m)
        assert all(math.isfinite(x) and x > 0.0 for x in result.lap_times_s)
        assert all(math.isfinite(x) for x in result.match_scores)
        assert result.autocorr_peak is None or math.isfinite(result.autocorr_peak)

    @settings(max_examples=30, deadline=None)
    @given(x=st.lists(st.floats(-1e3, 1e3, allow_nan=False), min_size=0, max_size=50))
    def test_property_autocorrelation_is_bounded(x) -> None:
        r = method.autocorrelation(np.array(x), 10)
        assert r.shape == (11,)
        assert np.all(np.isfinite(r))
        assert np.all(np.abs(r) <= 1.0 + 1e-9)
