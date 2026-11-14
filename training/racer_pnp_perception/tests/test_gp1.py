"""GP1: each decision item at, just inside and just outside its threshold, the named outcomes, the
paired block bootstrap, the cell table and the whole evaluation on a simulated drive."""

from __future__ import annotations

import copy
import itertools
import json
import math
from pathlib import Path

import numpy as np
import pytest
from racer_pnp_perception import gp1
from racer_pnp_perception import selflabel as sl
from racer_pnp_perception import split as sp
from racer_pnp_perception.tracks import IslandParams, island_world
from racer_track_perception import core
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle
from test_selflabel import SPEED_MPS, drive_poses, scans_from_poses, true_motion

NET, GEO = gp1.BACKEND_NET, gp1.BACKEND_GEOMETRIC
EPS = 1e-6


def labelled(e_c: float, v: float, scans: int = 1000, c95: float = 0.9) -> gp1.Labelled:
    return gp1.Labelled(scans, e_c, e_c, v, c95)


def free(
    intrusion: float = 0.01,
    residual: float = 0.05,
    deficit: float = 0.02,
    c95: float = 0.9,
) -> gp1.LabelFree:
    return gp1.LabelFree(1000, intrusion, residual, deficit, c95, 900)


def passing_numbers() -> gp1.Numbers:
    """Everything comfortably inside: the net is better in layout, as good elsewhere."""
    return gp1.Numbers(
        h_run={NET: labelled(0.8, 14.0), GEO: labelled(1.0, 13.0)},
        h_layout={NET: labelled(0.9, 12.0, scans=500), GEO: labelled(1.0, 12.0, scans=500)},
        a_run={NET: free(), GEO: free()},
        a_layout={NET: free(), GEO: free()},
        ratio_ci_hi=0.9,
        cells=[gp1.Cell("run", "ccw", "[0.2, 0.6)", 100, 0.5, 0.5)],
    )


def verdict(numbers: gp1.Numbers) -> tuple[dict[int, bool], str]:
    items, outcome, _failed = gp1.decide(numbers)
    return {i.number: i.passed for i in items}, outcome


def test_baseline_passes_all_six_items() -> None:
    items, outcome = verdict(passing_numbers())
    assert items == {k: True for k in range(1, 7)}
    assert outcome == gp1.OUTCOME_PASS


def test_item_1_needs_300_labelled_scans() -> None:
    n = passing_numbers()
    n.h_run[NET].scans = 300
    assert verdict(n)[0][1]
    n.h_run[NET].scans = 299
    items, outcome = verdict(n)
    assert not items[1] and outcome == gp1.OUTCOME_UNDECIDED


@pytest.mark.parametrize(
    ("net_e_c", "ci_hi", "passed"),
    [
        (0.9, 0.9999, True),  # exactly 0.90 times, interval just below 1.00
        (0.9 - EPS, 0.5, True),
        (0.9 + EPS, 0.5, False),  # ratio just over
        (0.8, 1.0, False),  # the upper end must be below 1.00, not at it
        (0.8, 1.0 + EPS, False),
        (math.nan, 0.5, False),
    ],
)
def test_item_2_ratio_and_interval(net_e_c: float, ci_hi: float, passed: bool) -> None:
    n = passing_numbers()
    n.h_run[NET] = labelled(net_e_c, 14.0)
    n.ratio_ci_hi = ci_hi
    items, outcome = verdict(n)
    assert items[2] is passed
    assert outcome == (gp1.OUTCOME_PASS if passed else gp1.OUTCOME_NO_BETTER)


@pytest.mark.parametrize(
    ("v_net", "passed"), [(13.0, True), (13.0 + EPS, True), (13.0 - EPS, False)]
)
def test_item_3_range(v_net: float, passed: bool) -> None:
    n = passing_numbers()
    n.h_run[NET] = labelled(0.8, v_net)
    items, outcome = verdict(n)
    assert items[3] is passed
    assert outcome == (gp1.OUTCOME_PASS if passed else gp1.OUTCOME_NO_BETTER)


@pytest.mark.parametrize(
    ("e_c", "v", "passed"),
    [
        (1.0, 11.5, True),  # at 1.00 times and at V minus 0.5
        (1.0 - EPS, 11.5 + EPS, True),
        (1.0 + EPS, 12.0, False),
        (0.5, 11.5 - EPS, False),
    ],
)
def test_item_4_cross_layout_with_enough_labels(e_c: float, v: float, passed: bool) -> None:
    n = passing_numbers()
    n.h_layout = {NET: labelled(e_c, v, scans=100), GEO: labelled(1.0, 12.0, scans=100)}
    items, outcome = verdict(n)
    assert items[4] is passed
    assert outcome == (gp1.OUTCOME_PASS if passed else gp1.OUTCOME_NOT_ELSEWHERE)


def test_item_4_with_fewer_than_100_labels_is_judged_label_free() -> None:
    n = passing_numbers()
    # The labelled numbers are terrible, but there are 99 of them: they are not used.
    n.h_layout = {NET: labelled(5.0, 1.0, scans=99), GEO: labelled(1.0, 12.0, scans=99)}
    items, _ = gp1.decide(n)[0], None
    item4 = next(i for i in items if i.number == 4)
    assert item4.passed and "label-free" in item4.note
    # ... and the label-free numbers of A-layout decide it.
    n.a_layout[NET] = free(deficit=0.0501)
    items, outcome = verdict(n)
    assert not items[4] and not items[5] and outcome == gp1.OUTCOME_NOT_ELSEWHERE
    # With exactly 100 labels the labelled numbers count again.
    n = passing_numbers()
    n.h_layout = {NET: labelled(5.0, 1.0, scans=100), GEO: labelled(1.0, 12.0, scans=100)}
    assert not verdict(n)[0][4]


@pytest.mark.parametrize("which", ["a_run", "a_layout"])
@pytest.mark.parametrize(
    ("field", "geo_value", "limit"),
    [
        ("intrusion", 0.0, 0.02),  # max(0.02, geometric + 0.005): the floor binds
        (
            "intrusion",
            0.03,
            0.03 + gp1.ITEM5_INTRUSION_MARGIN,
        ),  # the geometric tracer's own intrusion plus 0.005 binds
        ("residual", 0.05, 0.10),
        ("deficit", 0.02, 0.05),
    ],
)
def test_item_5_upper_limits(which: str, field: str, geo_value: float, limit: float) -> None:
    for delta, passed in ((0.0, True), (-EPS, True), (EPS, False)):
        n = passing_numbers()
        side = getattr(n, which)
        side[GEO] = free(**({field: geo_value} if field == "intrusion" else {}))
        side[NET] = free(**{field: limit + delta})
        items, outcome = verdict(n)
        assert items[5] is passed, (which, field, delta)
        assert outcome == (gp1.OUTCOME_PASS if passed else gp1.OUTCOME_NOT_ELSEWHERE)


@pytest.mark.parametrize("which", ["a_run", "a_layout"])
def test_item_5_c95_may_trail_by_0_02(which: str) -> None:
    for c95, passed in ((0.88, True), (0.88 + EPS, True), (0.88 - EPS, False)):
        n = passing_numbers()
        getattr(n, which)[NET] = free(c95=c95)  # the geometric C95 is 0.90
        assert verdict(n)[0][5] is passed


def test_item_5_nan_fails() -> None:
    n = passing_numbers()
    n.a_run[NET] = free(residual=math.nan)
    assert not verdict(n)[0][5]


@pytest.mark.parametrize(
    ("scans", "ratio", "passed"),
    [
        (40, 1.10, True),  # exactly 1.10 times
        (40, 1.10 - EPS, True),
        (40, 1.10 + EPS, False),
        (39, 3.0, True),  # fewer than 40 labelled scans: not judged
    ],
)
def test_item_6_per_cell(scans: int, ratio: float, passed: bool) -> None:
    n = passing_numbers()
    n.cells = [
        gp1.Cell("run", "ccw", "[0.2, 0.6)", 100, 1.0, 1.0),
        gp1.Cell("run", "cw", "0.85+", scans, ratio, 1.0),
    ]
    items, outcome = verdict(n)
    assert items[6] is passed
    assert outcome == (gp1.OUTCOME_PASS if passed else gp1.OUTCOME_NOT_ELSEWHERE)


def test_failing_2_or_3_is_named_even_when_others_fail_too() -> None:
    n = passing_numbers()
    n.h_run[NET] = labelled(0.95, 14.0)
    n.a_run[NET] = free(deficit=0.2)
    items, outcome = verdict(n)
    assert not items[2] and not items[5]
    assert outcome == gp1.OUTCOME_NO_BETTER


def test_the_items_carry_value_and_threshold() -> None:
    items, _outcome, failed = gp1.decide(passing_numbers())
    assert failed == [] and [i.number for i in items] == [1, 2, 3, 4, 5, 6]
    for item in items:
        assert item.value and item.threshold and item.name
    json.dumps([vars(i) for i in items], allow_nan=False)


# ---------------------------------------------------------------------------------------------
# Speed bins, bootstrap, cells
# ---------------------------------------------------------------------------------------------


def test_speed_bins() -> None:
    assert gp1.speed_bin(0.2) == "[0.2, 0.6)" and gp1.speed_bin(0.5999) == "[0.2, 0.6)"
    assert gp1.speed_bin(0.6) == "[0.6, 0.85)" and gp1.speed_bin(0.8499) == "[0.6, 0.85)"
    assert gp1.speed_bin(0.85) == "0.85+" and gp1.speed_bin(2.0) == "0.85+"


def test_speed_at_takes_the_latest_sample_at_or_before() -> None:
    v = gp1.speed_at(
        np.array([0.5, 1.0, 2.5]), np.array([0.0, 1.0, 2.0]), np.array([0.1, -0.4, 0.9])
    )
    assert v.tolist() == [0.1, 0.4, 0.9]


def test_bootstrap_recovers_a_known_ratio() -> None:
    rng = np.random.default_rng(3)
    n_blocks, per = 60, 20
    block = np.repeat(np.arange(n_blocks), per)
    geo = rng.lognormal(mean=-2.0, sigma=0.5, size=n_blocks * per)
    # An exactly proportional net: every resample has the same ratio.
    out = gp1.block_bootstrap_ratio(0.8 * geo, geo, block)
    assert out["blocks"] == n_blocks
    assert out["ratio"] == pytest.approx(0.8) and out["lo"] == pytest.approx(0.8)
    assert out["hi"] == pytest.approx(0.8)
    # With scan level noise the interval is open but contains the true ratio.
    net = 0.8 * geo * rng.lognormal(0.0, 0.3, size=geo.shape)
    noisy = gp1.block_bootstrap_ratio(net, geo, block)
    assert noisy["lo"] < noisy["ratio"] < noisy["hi"]
    assert noisy["lo"] < 0.8 * math.exp(0.045) < noisy["hi"] + 0.05
    assert noisy["hi"] - noisy["lo"] < 0.2


def test_bootstrap_is_seeded_and_paired() -> None:
    rng = np.random.default_rng(4)
    block = np.repeat(np.arange(30), 10)
    geo = rng.uniform(0.05, 0.2, size=300)
    net = geo * rng.uniform(0.7, 1.1, size=300)
    a = gp1.block_bootstrap_ratio(net, geo, block)
    assert a == gp1.block_bootstrap_ratio(net, geo, block)
    assert a != gp1.block_bootstrap_ratio(net, geo, block, seed=1)
    # Blocks, not scans, are resampled: the interval is wider than a scan level one would be.
    scan_level = gp1.block_bootstrap_ratio(net, geo, np.arange(300))
    assert (a["hi"] - a["lo"]) >= 0.8 * (scan_level["hi"] - scan_level["lo"])


def test_bootstrap_scans_without_an_error_add_nothing() -> None:
    block = np.repeat(np.arange(4), 5)
    geo = np.full(20, 0.1)
    net = np.full(20, 0.05)
    net[::3] = math.nan
    out = gp1.block_bootstrap_ratio(net, geo, block)
    assert out["ratio"] == pytest.approx(0.5) and out["hi"] == pytest.approx(0.5)
    empty = gp1.block_bootstrap_ratio(np.zeros(0), np.zeros(0), np.zeros(0, dtype=np.int64))
    assert math.isnan(empty["ratio"]) and empty["blocks"] == 0


def row(
    t: float, seg: str = "S", e_net: float = 0.1, e_geo: float = 0.1, speed: float = 0.5, **kw
) -> gp1.ScanRow:
    r = gp1.ScanRow(
        bag="b",
        index=0,
        set=kw.get("set", gp1.SET_RUN),
        segment=seg,
        segment_from_s=0.0,
        t_s=t,
        speed_mps=speed,
        direction=kw.get("direction", "ccw"),
        path=None,
        label=np.zeros((16, 5), np.float32) if kw.get("labelled", True) else None,
        label_valid=14,
    )
    r.e_c = {NET: e_net, GEO: e_geo}
    return r


def test_block_ids_are_2_second_blocks_within_a_segment() -> None:
    rows = [row(0.1), row(1.9), row(2.0), row(3.9), row(4.1), row(1.0, seg="T")]
    ids = gp1.block_ids(rows)
    assert ids[0] == ids[1] and ids[2] == ids[3]
    assert len({ids[0], ids[2], ids[4], ids[5]}) == 4


def test_cell_table_groups_by_direction_and_speed_bin() -> None:
    rows = (
        [row(i * 0.1, e_net=0.2, e_geo=0.1, speed=0.5) for i in range(40)]
        + [row(i * 0.1, direction="cw", speed=0.7) for i in range(10)]
        + [row(i * 0.1, labelled=False) for i in range(50)]
        + [row(i * 0.1, set=gp1.SET_LAYOUT, direction="cw", speed=1.0) for i in range(5)]
    )
    cells = {(c.set, c.direction, c.speed_bin): c for c in gp1.cell_table(rows)}
    assert cells[("run", "ccw", "[0.2, 0.6)")].scans == 40
    assert cells[("run", "ccw", "[0.2, 0.6)")].judged
    assert cells[("run", "ccw", "[0.2, 0.6)")].ratio == pytest.approx(2.0)
    assert cells[("run", "cw", "[0.6, 0.85)")].scans == 10
    assert not cells[("run", "cw", "[0.6, 0.85)")].judged
    # The cross-layout bags have no direction in the split.
    assert ("layout", "unknown", "0.85+") in cells
    assert len(cells) == 3


# ---------------------------------------------------------------------------------------------
# The whole evaluation on a simulated drive
# ---------------------------------------------------------------------------------------------


def random_net(seed: int = 0) -> CircConv1dV0:
    rng = np.random.default_rng(seed)
    channels = (2, 4, 4, 4, 4)
    arrays: dict[str, np.ndarray] = {"n_conv": np.array(4), "n_linear": np.array(2)}
    length = 360
    for i, (c_in, c_out) in enumerate(itertools.pairwise(channels)):
        arrays[f"conv{i}_weight"] = (rng.normal(size=(c_out, c_in, 5)) * 0.3).astype(np.float32)
        arrays[f"conv{i}_bias"] = (rng.normal(size=c_out) * 0.1).astype(np.float32)
        arrays[f"conv{i}_stride"] = np.array(2)
        arrays[f"conv{i}_pad"] = np.array(2)
        length = (length + 4 - 5) // 2 + 1
    arrays["linear0_weight"] = (rng.normal(size=(16, channels[-1] * length)) * 0.1).astype(
        np.float32
    )
    arrays["linear0_bias"] = np.zeros(16, dtype=np.float32)
    arrays["linear1_weight"] = (rng.normal(size=(80, 16)) * 0.1).astype(np.float32)
    bias = np.zeros(80, dtype=np.float32)
    bias[4::5] = np.log(0.05)
    arrays["linear1_bias"] = bias
    return CircConv1dV0(arrays)


SIM_SPLIT_YAML = """
split_id: sim
bags:
  simbag:
    segments:
      - {name: SA, from: 0.0, to: 4.0, direction: ccw, perception: train}
      - {name: SB, from: 4.0001, to: 9.6, direction: ccw, perception: heldout}
"""


@pytest.fixture(scope="module")
def sim_run(tmp_path_factory, live: LiveVehicle):
    tmp = tmp_path_factory.mktemp("gp1sim")
    world = island_world(IslandParams(1.0, 3.0, 2.0, 0.4))
    poses = drive_poses(world, 96)
    scans = scans_from_poses(world, live, poses)
    out = tmp / "labels"
    cache = out / "cache"
    cache.mkdir(parents=True)
    np.savez(
        cache / "simbag.scans.npz",
        stamp_s=scans.stamp_s,
        ranges=scans.ranges,
        angle_min=scans.angle_min,
        angle_increment=scans.angle_increment,
        range_min=scans.range_min,
        range_max=scans.range_max,
        speed_t=scans.stamp_s,
        speed_mps=np.full(len(scans), SPEED_MPS),
    )
    sl.save_motion(cache / "simbag.motion.npz", true_motion(poses))
    split = sp.parse_split(SIM_SPLIT_YAML, sha256="0" * 64)
    entry = sl.label_bag(tmp / "simbag", out, 0.2, 1, live, split)
    return tmp, out, split, entry


def test_selflabel_with_a_split_keeps_only_labels_inside_one_segment(sim_run) -> None:
    _tmp, out, _split, entry = sim_run
    assert entry["moving_scans"] == 96 and entry["split_sha256"] == "0" * 64
    assert entry["dropped"]["segment_edge"] > 0  # windows reach across the SA/SB edge
    assert entry["kept"] + sum(entry["dropped"].values()) == entry["moving_scans"]
    segs = entry["segments"]
    assert set(segs) == {"SA", "SB"} and segs["SA"]["role"] == "train"
    for s in segs.values():
        assert s["kept"] + sum(s["dropped"].values()) == s["moving_scans"]
    with np.load(out / "simbag.npz") as z:
        assert len(z["index"]) == entry["kept"]
        assert set(z["role"].tolist()) == {"train", "heldout"}
        # Every kept window lies inside the one segment it names.
        for seg, role, t0, t1 in zip(
            z["segment"], z["role"], z["window_first_s"], z["window_last_s"], strict=True
        ):
            s = next(
                s for s in sp.parse_split(SIM_SPLIT_YAML).bags["simbag"].segments if s.name == seg
            )
            assert s.role == role and s.t_from_s <= t0 <= t1 <= s.t_to_s


def test_selflabel_main_refuses_a_split_that_is_not_the_pre_registered_one(
    tmp_path: Path,
) -> None:
    path = tmp_path / "split.yaml"
    path.write_text(SIM_SPLIT_YAML)
    with pytest.raises(sp.SplitError):
        sl.main(["--bags", str(tmp_path / "simbag"), "--out", str(tmp_path), "--split", str(path)])


def test_gp1_on_a_simulated_drive(sim_run, live: LiveVehicle) -> None:
    tmp, out, split, _entry = sim_run
    perceivers = {
        GEO: core.TrackPerceiver(GEO, live),
        NET: core.TrackPerceiver(NET, live, net=random_net(), model_id="random"),
    }
    rows = gp1.collect_bag(tmp / "simbag", out, split, perceivers, live)
    assert rows and all(r.set == gp1.SET_RUN and r.segment == "SB" for r in rows)
    # Held-out labelled scans have labels from role heldout only.
    labelled_rows = [r for r in rows if r.label is not None]
    assert 5 < len(labelled_rows) <= len(rows)
    result = gp1.summarise(rows, split, "random")
    json.dumps(result, allow_nan=False)
    assert result["sets"]["H-run"]["labelled_scans"] == len(labelled_rows)
    assert result["sets"]["A-run"]["moving_scans"] == len(rows)
    assert result["sets"]["H-layout"]["labelled_scans"] == 0
    assert [i["number"] for i in result["items"]] == [1, 2, 3, 4, 5, 6]
    # Fewer than 300 labelled scans: undecided, whatever the numbers are.
    assert result["outcome"] == gp1.OUTCOME_UNDECIDED and result["failed_items"][0] == 1
    # The geometric tracer is scored against labels traced by itself on a larger window.
    assert result["H-run"][GEO]["e_c_m"] < 0.1
    assert gp1.format_report(result).splitlines()[0].startswith("GP1 outcome: undecided")
    # A scan whose next 2 m of path leaves the segment has no path (not counted in C95).
    assert any(r.path is None for r in rows) and any(r.path is not None for r in rows)
    copy.deepcopy(result)
