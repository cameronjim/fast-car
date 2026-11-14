"""Training with real labels: the role filter reaches config.json, the internal validation is never
trained on, station weights, and the seed selection (needs torch: the `train` group)."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from racer_pnp_perception import dataset, model, train
from racer_pnp_perception import split as sp
from test_real import SPLIT, fake_labels


def test_station_weights_mask_stations_past_valid_count() -> None:
    w = train.station_weights(np.array([12, 16, 0]))
    assert w.shape == (3, 16)
    assert w[0].tolist() == [1.0] * 12 + [0.0] * 4
    assert w[1].sum() == 16 and w[2].sum() == 0


@pytest.mark.torch
def test_loss_ignores_unlabelled_stations_and_matches_the_plain_loss_when_all_are_valid() -> None:
    rng = np.random.default_rng(0)
    pred = torch.from_numpy(rng.normal(size=(4, 16, 5)).astype(np.float32))
    target = torch.from_numpy(rng.normal(size=(4, 16, 5)).astype(np.float32))
    plain = model.loss_fn(pred, target)
    ones = torch.ones(4, 16)
    assert torch.allclose(model.loss_fn(pred, target, station_weight=ones), plain, atol=1e-6)
    weight = torch.ones(4, 16)
    weight[:, 12:] = 0.0
    changed = target.clone()
    changed[:, 12:] += 5.0  # garbage past the valid stations
    assert torch.allclose(
        model.loss_fn(pred, target, station_weight=weight),
        model.loss_fn(pred, changed, station_weight=weight),
        atol=1e-6,
    )
    assert torch.isfinite(model.loss_fn(pred, target, station_weight=torch.zeros(4, 16)))


def test_select_seed_uses_only_the_final_internal_validation_error() -> None:
    runs = {
        0: [{train.SELECTION_KEY: 0.30}, {train.SELECTION_KEY: 0.10}],
        1: [{train.SELECTION_KEY: 0.01}, {train.SELECTION_KEY: 0.12}],  # best early, not at the end
        2: [{train.SELECTION_KEY: 0.2}, {train.SELECTION_KEY: float("nan")}],
    }
    sel = train.select_seed(runs)
    assert sel["selected_seed"] == 0 and sel["per_seed"][0] == 0.10
    with pytest.raises(ValueError):
        train.select_seed({0: [{train.SELECTION_KEY: float("nan")}]})


def test_select_from_dir_reads_seed_runs_made_one_by_one(tmp_path: Path) -> None:
    for seed, ec in ((0, 0.09), (1, 0.07), (2, 0.08)):
        run = tmp_path / f"seed{seed}"
        run.mkdir()
        (run / "history.json").write_text(
            json.dumps([{train.SELECTION_KEY: 0.5}, {train.SELECTION_KEY: ec}])
        )
    sel = train.select_from_dir(tmp_path)
    assert sel["selected_seed"] == 1
    assert json.loads((tmp_path / "selection.json").read_text())["selected_seed"] == 1


@pytest.mark.torch
def test_a_run_with_real_labels_records_its_segments_and_never_sees_heldout(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    rng = np.random.default_rng(0)
    n = 2048
    sim = {
        "tensor": rng.uniform(size=(n, 2, 360)).astype(np.float32),
        "label": np.concatenate(
            [
                0.2
                * np.arange(16, dtype=np.float32)[None, :, None]
                * np.ones((n, 16, 1), np.float32),
                rng.normal(size=(n, 16, 4)).astype(np.float32) * 0.1,
            ],
            axis=2,
        ),
    }
    sim["tensor"][:, 1] = (sim["tensor"][:, 1] > 0.5).astype(np.float32)
    dataset.save(tmp_path / "train.npz", sim)
    dataset.save(tmp_path / "val.npz", sim)
    # The real split's digest is the pre-registered one; this run uses its own small split.
    monkeypatch.setattr(
        sp, "load_split", lambda path=None, workstream="perception", expected_sha256=None: SPLIT
    )
    labels = tmp_path / "labels"
    labels.mkdir()
    fake_labels(
        labels / "bagA.npz",
        [(1.0, 5.0), (50.0, 54.0), (90.0, 94.0), (111.0, 115.0)],
        ["T", "T", "T", "H"],
        ["train", "train", "train", "heldout"],
    )
    fake_labels(
        labels / "bagL.npz", [(0.0, 4.0), (10.0, 14.0)], ["whole_bag"] * 2, ["heldout_layout"] * 2
    )
    out = tmp_path / "run"
    result = train.train(
        tmp_path / "train.npz",
        tmp_path / "val.npz",
        out,
        epochs=1,
        batch_size=64,
        real_paths=[labels / "bagA.npz", labels / "bagL.npz"],
        real_fraction=0.25,
        split_path=tmp_path / "split.yaml",
    )
    cfg = json.loads((out / "config.json").read_text())
    assert result["config"]["real"] == cfg["real"]
    assert cfg["real"]["roles_used"] == ["train"]
    assert cfg["real"]["segments_used"] == ["bagA/T"]
    assert "bagL" not in json.dumps(cfg["real"]["segments_used"])
    # Two labels train (windows end before the cut at 85), one validates, the heldout one and
    # both layout labels never entered anything.
    assert cfg["real"]["train_labels"] == 2 and cfg["real"]["val_labels"] == 1
    history = json.loads((out / "history.json").read_text())
    assert train.SELECTION_KEY in history[0] and np.isfinite(history[0]["val_loss"])
    assert (out / "model.pt").is_file()


def test_real_fraction_without_real_labels_is_an_error(tmp_path: Path) -> None:
    rng = np.random.default_rng(0)
    sim = {
        "tensor": rng.uniform(size=(64, 2, 360)).astype(np.float32),
        "label": rng.normal(size=(64, 16, 5)).astype(np.float32),
    }
    dataset.save(tmp_path / "t.npz", sim)
    with pytest.raises(ValueError, match="needs --real"):
        train.train(
            tmp_path / "t.npz", tmp_path / "t.npz", tmp_path / "r", epochs=1, real_fraction=0.5
        )
