"""L1/L2: the slope limit and the corridor polygon (brief 38a), and the choice rule of the slope.

Bound derived and asserted (L2). On a straight tracer (spacing s = 0.20 m) with the shift profile
limited to |d_{i+1} - d_i| <= k s, the fused polyline's tangent is the chord direction of the
shifted centres, whose cross-track slope is at most k (central differences over two steps: 2 k s
over 2 s), so the normal turns by at most atan(k) <= k, and its change is at most 2 sin(k / 2) <= k.
The edge point a consumer derives is c' + (h - d) n' against the tracer's c + h n, c' = c + d n, so
it moves by (h - d) |n' - n| <= (h + |d|) k <= (h + reach) k. Per edge the area between the two
edge polylines is at most that displacement times the length, so with half widths h_l, h_r the loss
and the excess are each at most k (h_l + h_r + 2 reach) / (h_l + h_r) (plus the 0.01 m grid). This is
loose (about 3 k for narrow corridors); the registered numbers are the measurements, not this
bound, which only says that the limit does bound the loss by a function of k.
"""

from __future__ import annotations

import numpy as np
import pytest
from racer_pnp_perception import corridor, slope_select
from racer_track_perception import fuse
from racer_track_perception import track as tr

GRID_SLACK = 0.01


def straight_track(rng: np.random.Generator, n: int, hl: float, hr: float, y=None):
    t = tr.LocalTrackData(valid_count=n)
    t.x_m[:] = 0.2 * np.arange(16)
    t.y_m[:] = 0.0 if y is None else y
    t.half_width_left_m[:] = hl
    t.half_width_right_m[:] = hr
    t.sigma_m[:n] = 0.05
    t.sigma_m[n:] = tr.INVALID_SIGMA_M
    return t


@pytest.mark.parametrize("slope", slope_select.SLOPE_GRID)
def test_coverage_loss_and_excess_are_bounded_by_the_slope(slope: float) -> None:
    rng = np.random.default_rng(7)
    reach = fuse.FuseConfig().reach_m
    worst = 0.0
    for _ in range(120):
        n = int(rng.integers(8, 17))
        hl, hr = rng.uniform(0.3, 1.0, size=2)
        geo = straight_track(rng, n, hl, hr)
        net = straight_track(rng, n, 0.5, 0.5, y=rng.uniform(-0.3, 0.3, size=16))
        cfg = fuse.FuseConfig(append_far=False, max_shift_slope=slope)
        res = corridor.compare(geo, fuse.fuse_track(geo, net, cfg))
        bound = slope * (hl + hr + 2.0 * reach) / (hl + hr) + GRID_SLACK
        worst = max(worst, res["loss"], res["excess"])
        assert res["loss"] <= bound and res["excess"] <= bound, (slope, res, bound)
    assert worst > 0.0  # the cases do move the centres


def test_a_smaller_slope_never_loses_more_than_a_larger_one_on_average() -> None:
    rng = np.random.default_rng(3)
    means = []
    for slope in slope_select.SLOPE_GRID:
        losses = []
        rng2 = np.random.default_rng(3)
        for _ in range(60):
            geo = straight_track(rng2, 12, 0.5, 0.5)
            net = straight_track(rng2, 12, 0.5, 0.5, y=rng2.uniform(-0.25, 0.25, size=16))
            cfg = fuse.FuseConfig(append_far=False, max_shift_slope=slope)
            losses.append(corridor.compare(geo, fuse.fuse_track(geo, net, cfg))["loss"])
        means.append(float(np.mean(losses)))
    assert means == sorted(means)
    assert rng is not None


# ---- the choice rule ------------------------------------------------------------------------


def cell(loss_mean=0.001, loss_p99=0.01, ex_mean=0.001, ex_p99=0.01):
    return {
        "loss": {"mean": loss_mean, "p99": loss_p99},
        "excess": {"mean": ex_mean, "p99": ex_p99},
    }


def table(per_slope: dict[float, dict[str, dict]]) -> dict:
    return per_slope


def test_the_largest_grid_value_meeting_the_margin_on_every_set_is_chosen() -> None:
    good, bad = cell(), cell(loss_mean=0.0051)
    t = {
        0.05: {"A-run": good, "F1": good},
        0.10: {"A-run": good, "F1": good},
        0.15: {"A-run": good, "F1": bad},  # one set over
        0.20: {"A-run": good, "F1": good},  # a larger value that passes everywhere wins
        0.30: {"A-run": bad, "F1": good},
    }
    assert slope_select.choose_slope(t) == 0.20


@pytest.mark.parametrize(
    "bad",
    [
        cell(loss_mean=0.0051),
        cell(ex_mean=0.0051),
        cell(loss_p99=0.0251),
        cell(ex_p99=0.0251),
    ],
)
def test_each_margin_condition_is_enforced_and_the_margin_itself_passes(bad: dict) -> None:
    at = cell(0.005, 0.025, 0.005, 0.025)
    assert slope_select.meets_margin(at)
    assert not slope_select.meets_margin(bad)
    assert slope_select.choose_slope({0.05: {"A-run": bad}}) is None


def test_no_value_meeting_the_margin_gives_none_and_the_outcome_text() -> None:
    bad = cell(loss_mean=0.01)
    t = {s: {"A-run": bad} for s in slope_select.SLOPE_GRID}
    assert slope_select.choose_slope(t) is None
    assert slope_select.NOT_ATTEMPTED == "GP1c not attempted: fusion cannot keep the corridor"


def test_an_empty_set_is_not_a_pass_and_values_off_the_grid_are_ignored() -> None:
    assert not slope_select.meets_margin({"loss": None, "excess": None})
    assert slope_select.choose_slope({0.25: {"A-run": cell()}}) is None
    assert slope_select.choose_slope({0.05: {}}) is None


def test_the_grid_and_margin_match_the_addendum_yaml_when_present() -> None:
    import yaml
    from racer_track_perception.vehicle import find_repo_root

    path = find_repo_root() / "evaluation" / "prereg" / "pnp-v0.4-addendum-1.yaml"
    if not path.is_file():
        pytest.skip("addendum 1 is not on this branch yet (PR 189)")
    block = yaml.safe_load(path.read_text())["perception_coverage"]
    assert tuple(block["slope_grid"]) == slope_select.SLOPE_GRID
    m = block["development_margin"]
    assert m["max_mean_loss"] == m["max_mean_excess"] == slope_select.MARGIN_MAX_MEAN
    assert m["max_p99_loss"] == m["max_p99_excess"] == slope_select.MARGIN_MAX_P99
    assert block["grid_m"] == corridor.GRID_M
