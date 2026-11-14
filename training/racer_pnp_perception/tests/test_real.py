"""Real labels as training data: the role filter, the internal validation cut, the mirror."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_pnp_perception import real
from racer_pnp_perception import split as sp
from racer_pnp_perception.raycast import ScanGeometry, lidar_scan
from racer_pnp_perception.sample import LaneGeometry, label_for_pose
from racer_pnp_perception.tracks import IslandParams, World, island_world
from racer_track_perception import preprocess
from racer_track_perception.vehicle import LiveVehicle

SPLIT = sp.parse_split(
    """
bags:
  bagA:
    segments:
      - {name: T, from: 0.0, to: 100.0, perception: train}
      - {name: H, from: 110.0, to: 150.0, perception: heldout}
  bagL:
    whole_bag: {perception: heldout_layout}
  bagW:
    whole_bag: {perception: train}
"""
)


def fake_labels(
    path: Path,
    windows: list[tuple[float, float]],
    segments: list[str],
    roles: list[str],
    seed: int = 0,
) -> None:
    rng = np.random.default_rng(seed)
    n = len(windows)
    label = rng.normal(size=(n, 16, 5)).astype(np.float32)
    np.savez(
        path,
        index=np.arange(n),
        tensor=rng.uniform(size=(n, 1, 2, 360)).astype(np.float32),
        label=label,
        valid_count=np.full(n, 14),
        window_first_s=np.array([w[0] for w in windows]),
        window_last_s=np.array([w[1] for w in windows]),
        segment=np.array(segments),
        role=np.array(roles),
    )


@pytest.fixture
def label_dir(tmp_path: Path) -> Path:
    fake_labels(
        tmp_path / "bagA.npz",
        [(1.0, 5.0), (50.0, 54.0), (84.0, 88.0), (83.0, 86.0), (90.0, 94.0), (111.0, 115.0)],
        ["T", "T", "T", "T", "T", "H"],
        ["train", "train", "train", "train", "train", "heldout"],
    )
    fake_labels(
        tmp_path / "bagL.npz", [(0.0, 4.0), (10.0, 14.0)], ["whole_bag"] * 2, ["heldout_layout"] * 2
    )
    fake_labels(
        tmp_path / "bagW.npz",
        [(0.0, 4.0), (50.0, 54.0), (96.0, 100.0)],
        ["whole_bag"] * 3,
        ["train"] * 3,
    )
    return tmp_path


def test_only_train_labels_get_in(label_dir: Path) -> None:
    got = real.load_real([label_dir / f"{b}.npz" for b in ("bagA", "bagL", "bagW")], SPLIT)
    assert set(got.role.tolist()) == {"train"}
    assert len(got) == 5 + 3  # bagA: 5 train of 6 (the heldout one is out), bagL: none, bagW: 3
    assert "bagL" not in set(got.bag.tolist())
    assert not (got.segment == "H").any()


def test_a_heldout_sample_cannot_train_even_if_the_file_says_train(label_dir: Path) -> None:
    fake_labels(label_dir / "liar.npz", [(111.0, 115.0)], ["H"], ["train"])
    liar = real.read_label_file(label_dir / "liar.npz")
    liar.bag[:] = "bagA"
    with pytest.raises(real.RoleError, match="disagrees"):
        real.only_train(liar, SPLIT)
    # And a file named for a held-out layout bag, claiming train, is refused the same way.
    fake_labels(label_dir / "bagL.npz", [(0.0, 4.0)], ["whole_bag"], ["train"])
    with pytest.raises(real.RoleError, match="disagrees"):
        real.load_real([label_dir / "bagL.npz"], SPLIT)


def test_a_label_file_without_roles_is_refused(tmp_path: Path) -> None:
    np.savez(
        tmp_path / "old.npz",
        index=np.arange(2),
        tensor=np.zeros((2, 1, 2, 360), np.float32),
        label=np.zeros((2, 16, 5), np.float32),
        valid_count=np.full(2, 14),
    )
    with pytest.raises(real.RoleError, match="relabel"):
        real.load_real([tmp_path / "old.npz"], SPLIT)


def test_a_window_crossing_a_segment_edge_has_no_train_role() -> None:
    assert SPLIT.role_of("bagA", 98.0, 102.0) == "unused"
    assert SPLIT.role_of("bagA", 105.0, 112.0) == "unused"


def test_internal_validation_is_the_last_15_percent_minus_crossing_windows(
    label_dir: Path,
) -> None:
    got = real.load_real([label_dir / "bagA.npz", label_dir / "bagW.npz"], SPLIT)
    tr, va, report = real.train_val_split(got, SPLIT)
    # Segment T spans 0 to 100, the cut is at 85. (1,5) and (50,54) train; (84,88) and (83,86)
    # cross it and are dropped; (90,94) validates.
    a = report["bagA/T"]
    assert math.isclose(a["cut_s"], 85.0)
    assert (a["train"], a["val"], a["crossing_dropped"]) == (2, 1, 2)
    # A whole-bag train segment takes the span of its labels: 0 to 100, cut at 85: (96,100) is
    # validation, the other two train.
    w = report["bagW/whole_bag"]
    assert (w["train"], w["val"], w["crossing_dropped"]) == (2, 1, 0)
    assert len(tr) == 4 and len(va) == 2
    assert not set(map(tuple, np.c_[tr.window_first_s, tr.window_last_s].tolist())) & set(
        map(tuple, np.c_[va.window_first_s, va.window_last_s].tolist())
    )
    assert (tr.window_last_s <= 85.0).all() or (tr.bag == "bagW").any()
    assert (va.window_first_s >= 85.0).all()


# ---------------------------------------------------------------------------------------------
# Mirror
# ---------------------------------------------------------------------------------------------

GEOMETRY = ScanGeometry()


def mirrored_world(w: World) -> World:
    return World(
        w.name + "_mirror",
        (w.centreline * np.array([1.0, -1.0]))[::-1].copy(),
        w.walls * np.array([1.0, -1.0, 1.0, -1.0]),
        w.lane_width_m,
    )


def clean_tensor(ranges: np.ndarray, live: LiveVehicle) -> np.ndarray:
    ranges = np.where(np.isfinite(ranges), ranges, np.inf).astype(np.float32)
    return preprocess.scan_to_tensor(
        ranges,
        GEOMETRY.angle_min_rad,
        GEOMETRY.angle_increment_rad,
        GEOMETRY.range_min_m,
        GEOMETRY.range_max_m,
        live.mount,
        live.body,
    )[0]


def test_mirroring_twice_is_the_identity() -> None:
    rng = np.random.default_rng(0)
    x = rng.uniform(size=(5, 2, 360)).astype(np.float32)
    y = rng.normal(size=(5, 16, 5)).astype(np.float32)
    x2, y2 = real.mirror_sample(*real.mirror_sample(x, y))
    assert np.array_equal(x2, x) and np.array_equal(y2, y)
    m = real.mirror_label(y)
    assert np.array_equal(m[..., 0], y[..., 0]) and np.array_equal(m[..., 4], y[..., 4])
    assert np.array_equal(m[..., 1], -y[..., 1])
    assert np.array_equal(m[..., 2], y[..., 3]) and np.array_equal(m[..., 3], y[..., 2])
    assert not np.shares_memory(m, y)


@pytest.mark.parametrize("s0", [3.5, 6.1, 8.0])
def test_mirrored_left_bend_equals_the_right_bend_generated_directly(
    live: LiveVehicle, s0: float
) -> None:
    left = island_world(IslandParams(1.0, 3.0, 2.0, 0.4))
    right = mirrored_world(left)
    lane_l, lane_r = LaneGeometry(left.centreline), LaneGeometry(right.centreline)
    pos, tan, _ = lane_l.at(np.array([s0]))
    pose = (float(pos[0, 0]), float(pos[0, 1]), math.atan2(tan[0, 1], tan[0, 0]) + 0.1)
    pose_m = (pose[0], -pose[1], -pose[2])
    label_l = label_for_pose(lane_l, left.walls, pose)
    label_r = label_for_pose(lane_r, right.walls, pose_m)
    # It is a bend: the far stations are well off the heading line, to the left.
    assert label_l[15, 1] > 0.2 and label_r[15, 1] < -0.2
    t_left = clean_tensor(lidar_scan(pose, live.mount, GEOMETRY, left.walls), live)
    t_right = clean_tensor(lidar_scan(pose_m, live.mount, GEOMETRY, right.walls), live)
    t_mirror, l_mirror = real.mirror_sample(t_left, label_l)
    assert np.abs(l_mirror[:, :4] - label_r[:, :4]).max() < 1e-6
    both = (t_mirror[1] > 0) & (t_right[1] > 0)
    assert (t_mirror[1] == t_right[1]).mean() > 0.99
    assert np.abs(t_mirror[0] - t_right[0])[both].max() < 0.01


@settings(max_examples=40, deadline=None)
@given(
    seed=st.integers(0, 2**31 - 1),
    mirror_p=st.floats(0.0, 1.0),
)
def test_augmented_samples_stay_finite_and_inside_the_contract(seed: int, mirror_p: float) -> None:
    rng = np.random.default_rng(seed)
    n = 8
    x = np.stack(
        [rng.uniform(0.0, 1.0, (n, 360)), (rng.uniform(size=(n, 360)) < 0.8).astype(float)], axis=1
    ).astype(np.float32)
    x[:, 0] = np.where(x[:, 1] > 0, x[:, 0], 1.0)  # an empty bin has range 1.0
    y = rng.normal(size=(n, 16, 5)).astype(np.float32)
    ax, ay = real.augment_real(x, y, rng, mirror_p)
    assert ax.shape == (n, 2, 360) and ay.shape == (n, 16, 5)
    assert np.isfinite(ax).all() and np.isfinite(ay).all()
    assert ax[:, 0].min() >= 0.0 and ax[:, 0].max() <= 1.0
    assert set(np.unique(ax[:, 1]).tolist()) <= {0.0, 1.0}
    # A dropped bin is an empty bin; dropout only ever removes returns.
    assert (ax[:, 1] <= np.maximum(x[:, 1], x[:, 1][:, ::-1])).all()
    assert (ax[:, 0][ax[:, 1] == 0] == 1.0).all()
    # The label is the label or its mirror, nothing else.
    for k in range(n):
        assert np.array_equal(ay[k], y[k]) or np.array_equal(ay[k], real.mirror_label(y[k]))


def test_drop_bins_leaves_the_input_alone_and_drops_something() -> None:
    rng = np.random.default_rng(1)
    x = np.zeros((64, 2, 360), np.float32)
    x[:, 0] = 0.4
    x[:, 1] = 1.0
    before = x.copy()
    out = real.drop_bins(x, rng)
    assert np.array_equal(x, before)
    dropped = out[:, 1] == 0
    assert dropped.any() and not dropped.all()
    assert (out[:, 0][dropped] == 1.0).all() and (out[:, 0][~dropped] == 0.4).all()
