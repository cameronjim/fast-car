"""L1: the leave-layout-out folds of pnp-v0.3.yaml: no scan of a fold's held-out bags is in its
training set, by bag name and by provenance (bag, scan index)."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
from racer_pnp_perception import dataset, prereg, real
from racer_pnp_perception import split as sp
from test_real import fake_labels

SPLIT = sp.parse_split(
    """
bags:
  oldA:
    whole_bag: {perception: train}
  oldB:
    whole_bag: {perception: train}
  newC:
    whole_bag: {perception: train}
  segD:
    segments:
      - {name: S, from: 0.0, to: 100.0, perception: train}
  layout:
    whole_bag: {perception: heldout_layout}
"""
)
DOC = {
    "perception": {
        "folds": {"F1": {"heldout_bags": ["oldA", "oldB"]}, "F2": {"heldout_bags": ["newC"]}}
    }
}


def test_the_real_folds_are_the_pre_registered_ones() -> None:
    folds = sp.load_folds(sp.load_split())
    assert set(folds) == {"F1", "F2"}
    assert folds["F1"].heldout_bags == (
        "2026-11-11T03-54-11_car_teleop_cw",
        "2026-11-11T04-20-19_car_teleop_v3",
    )
    assert folds["F2"].heldout_bags == (
        "2026-10-24T23-13-18_car_teleop",
        "2026-10-25T19-27-08_car_teleop",
    )
    assert prereg.load_v03()["perception"]["folds"]["F1"]["heldout_bags"] == list(
        folds["F1"].heldout_bags
    )


def test_a_fold_must_name_whole_train_bags_and_not_share_a_bag() -> None:
    for bag in ("segD", "layout"):
        bad = {"perception": {"folds": {"F": {"heldout_bags": [bag]}}}}
        with pytest.raises(sp.SplitError):
            sp.load_folds(SPLIT, bad)
    dup = {
        "perception": {"folds": {"A": {"heldout_bags": ["oldA"]}, "B": {"heldout_bags": ["oldA"]}}}
    }
    with pytest.raises(sp.SplitError):
        sp.load_folds(SPLIT, dup)


def test_prereg_file_with_another_digest_is_refused(tmp_path: Path) -> None:
    p = tmp_path / "pnp-v0.3.yaml"
    p.write_text("perception: {}\n")
    with pytest.raises(prereg.PreregError):
        prereg.load_v03(p)


def make_label_files(tmp_path: Path) -> list[Path]:
    paths = []
    for i, bag in enumerate(("oldA", "oldB", "newC")):
        fake_labels(
            tmp_path / f"{bag}.npz",
            [(0.0, 4.0), (10.0, 14.0), (50.0, 54.0), (96.0, 99.0)],
            ["whole_bag"] * 4,
            ["train"] * 4,
            seed=i,
        )
        with np.load(tmp_path / f"{bag}.npz") as z:
            data = {k: z[k] for k in z.files}
        data["label"][..., 0] = 0.2 * np.arange(16)  # a straight label the scaling can work on
        data["label"][..., 1] = 0.0
        data["label"][..., 2:4] = 0.5
        np.savez(tmp_path / f"{bag}.npz", **data)
        paths.append(tmp_path / f"{bag}.npz")
    return paths


def test_dropping_a_folds_bags_leaves_none_of_them(tmp_path: Path) -> None:
    everything = real.load_real(make_label_files(tmp_path), SPLIT)
    assert len(everything) == 12
    fold = sp.load_folds(SPLIT, DOC)["F1"]
    provenance = {
        (str(b), int(i))
        for b, i in zip(everything.bag, everything.scan_index, strict=True)
        if str(b) in fold.heldout_bags
    }
    assert len(provenance) == 8
    kept = real.drop_bags(everything, fold.heldout_bags)
    assert set(kept.bag.tolist()) == {"newC"} and len(kept) == 4
    real.assert_fold_clean(kept, fold.heldout_bags, provenance)
    # Both checks fire: by name and, with a renamed bag, by provenance.
    with pytest.raises(real.RoleError):
        real.assert_fold_clean(everything, fold.heldout_bags, provenance)
    renamed = real.drop_bags(everything, ["newC"])
    renamed.bag[:] = "oldA"
    with pytest.raises(real.RoleError):
        real.assert_fold_clean(renamed, ["other"], provenance)


torch = pytest.importorskip("torch")


@pytest.mark.torch
def test_a_fold_run_never_sees_its_heldout_bags_and_records_it(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from racer_pnp_perception import train

    rng = np.random.default_rng(0)
    n = 512
    ramp = 0.2 * np.arange(16, dtype=np.float32)[None, :, None] * np.ones((n, 16, 1), np.float32)
    sim = {
        "tensor": rng.uniform(size=(n, 2, 360)).astype(np.float32),
        "label": np.concatenate(
            [ramp, rng.normal(size=(n, 16, 4)).astype(np.float32) * 0.1], axis=2
        ),
    }
    sim["tensor"][:, 1] = (sim["tensor"][:, 1] > 0.5).astype(np.float32)
    dataset.save(tmp_path / "train.npz", sim)
    dataset.save(tmp_path / "val.npz", sim)
    monkeypatch.setattr(
        sp, "load_split", lambda path=None, workstream="perception", expected_sha256=None: SPLIT
    )
    monkeypatch.setattr(train, "sp_folds", lambda split: sp.load_folds(split, DOC))
    out = tmp_path / "run"
    train.train(
        tmp_path / "train.npz",
        tmp_path / "val.npz",
        out,
        epochs=1,
        batch_size=64,
        real_paths=make_label_files(tmp_path),
        real_fraction=0.5,
        split_path=tmp_path / "split.yaml",
        scale_aug=True,
        fold="F1",
    )
    cfg = json.loads((out / "config.json").read_text())
    assert cfg["fold"] == "F1" and cfg["scale_aug"] is True and cfg["scale_range"] == [0.75, 1.4]
    assert cfg["real"]["heldout_bags"] == ["oldA", "oldB"]
    assert cfg["real"]["heldout_labels_removed"] == 8
    assert all(s.startswith("newC/") for s in cfg["real"]["segments_used"])
