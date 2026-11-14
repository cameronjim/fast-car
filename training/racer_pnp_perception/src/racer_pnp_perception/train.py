"""Train circ_conv1d_v0 on the simulated dataset, optionally with real self-labels (CPU).

    uv run --group train python -m racer_pnp_perception.train \\
        --train <train.npz> --val <val.npz> --out <run dir> [--epochs 12] \\
        [--real <labels.npz> ...] [--real-fraction 0.25] [--split <split.yaml>] \\
        [--init <model.pt>] [--seeds 0 1 2]

Adam with a one-cycle learning-rate schedule; loss is `model.loss_fn` (Gaussian negative log
likelihood on x, y with the predicted sigma per station, Huber on the two half widths). Writes
`<out>/model.pt`, `<out>/config.json` (every hyperparameter, the data files and, with real labels,
the segments used) and `<out>/history.json`. torch is imported inside `train`.

Real labels (`--real`, files of `selflabel --split`): only labels of role `train` enter (checked
against the split, see real.py). The last 15 percent in time of each train segment is the internal
validation part and is never trained on. A share `--real-fraction` of every batch is drawn from
the real training labels with replacement, mirrored half of the time and given bin dropout; the
rest is simulation. Stations past a real label's valid_count carry no loss. Each epoch reports
the sim validation loss and the internal validation centreline error E_c (the GP1 definition, on
the net's own valid stations), which is what model selection uses. With `--seeds` one run per seed
goes to `<out>/seed<k>`, and `<out>/selection.json` names the seed with the lowest final internal
validation E_c: decided from the internal validation alone.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from racer_pnp_perception import dataset, model, real

log = logging.getLogger(__name__)

SELECTION_KEY = "val_real_ec_m"


def station_weights(valid_count: NDArray[np.int64]) -> NDArray[np.float32]:
    """[n, 16] weights: 1 for the stations of a label that are valid, 0 past valid_count."""
    return (np.arange(16)[None, :] < valid_count[:, None]).astype(np.float32)


def internal_validation(net: Any, val: real.RealSet, batch: int = 1000) -> dict[str, float]:
    """E_c, E_far and V of the net on the internal validation labels, through the same track
    assembly as the node (`net_output_to_track`, sigma gate 0.25 m). NaN without labels."""
    import torch
    from racer_track_perception.core import net_output_to_track

    from racer_pnp_perception.scoring import scan_centreline_errors

    if len(val) == 0:
        return {SELECTION_KEY: float("nan"), "val_real_efar_m": float("nan"), "val_real_v": 0.0}
    x = torch.from_numpy(val.tensor)
    with torch.no_grad():
        out = torch.cat([net(x[i : i + batch]) for i in range(0, len(x), batch)]).numpy()
    ec, ef, v = [], [], []
    for k in range(len(val)):
        track = net_output_to_track(out[k], 0.25)
        e, f, _n = scan_centreline_errors(track, val.label[k], int(val.valid_count[k]))
        ec.append(e)
        ef.append(f)
        v.append(track.valid_count)
    with np.errstate(all="ignore"):
        return {
            SELECTION_KEY: float(np.nanmean(ec)) if np.isfinite(ec).any() else float("nan"),
            "val_real_efar_m": float(np.nanmean(ef)) if np.isfinite(ef).any() else float("nan"),
            "val_real_v": float(np.mean(v)),
        }


def train(
    train_path: Path,
    val_path: Path,
    out: Path,
    epochs: int = 12,
    batch_size: int = 256,
    lr: float = 2e-3,
    seed: int = 0,
    hflip: bool = False,
    real_paths: list[Path] | None = None,
    real_fraction: float = 0.0,
    split_path: Path | None = None,
    init: Path | None = None,
) -> dict[str, Any]:
    import torch

    from racer_pnp_perception.split import DEFAULT_SPLIT_PATH, load_split

    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    x_train, y_train = dataset.load(train_path)
    x_val, y_val = dataset.load(val_path)
    real_train: real.RealSet | None = None
    real_val: real.RealSet | None = None
    real_info: dict[str, Any] = {}
    if real_paths:
        split = load_split(split_path or DEFAULT_SPLIT_PATH)
        everything = real.load_real(list(real_paths), split)
        real_train, real_val, cut_report = real.train_val_split(everything, split)
        real_info = {
            "files": [str(p) for p in real_paths],
            "split_sha256": split.sha256,
            "roles_used": sorted(set(everything.role.tolist())),
            "segments_used": sorted(
                {f"{b}/{s}" for b, s in zip(everything.bag, everything.segment, strict=True)}
            ),
            "train_labels": len(real_train),
            "val_labels": len(real_val),
            "segment_cuts": cut_report,
        }
        log.info("real: %d train labels, %d internal validation", len(real_train), len(real_val))
    elif real_fraction > 0.0:
        raise ValueError("--real-fraction needs --real")
    n_real = round(batch_size * real_fraction) if real_train is not None else 0
    n_sim = batch_size - n_real
    if real_train is not None and len(real_train) == 0 and n_real > 0:
        raise ValueError("no real training labels after the role filter")
    net = model.build_model()
    if init is not None:
        net.load_state_dict(torch.load(init, map_location="cpu"))
    log.info("parameters: %d", model.count_parameters(net))
    opt = torch.optim.Adam(net.parameters(), lr=lr)
    steps_per_epoch = len(x_train) // n_sim
    sched = torch.optim.lr_scheduler.OneCycleLR(
        opt, max_lr=lr, total_steps=epochs * steps_per_epoch, pct_start=0.1
    )
    xv, yv = torch.from_numpy(x_val), torch.from_numpy(y_val)
    history: list[dict[str, float]] = []
    t0 = time.time()
    for epoch in range(epochs):
        net.train()
        order = rng.permutation(len(x_train))
        running = 0.0
        for b in range(steps_per_epoch):
            idx = order[b * n_sim : (b + 1) * n_sim]
            xb, yb = torch.from_numpy(x_train[idx]), torch.from_numpy(y_train[idx])
            if hflip:
                flip = torch.rand(len(idx)) < 0.5
                xb = torch.where(flip[:, None, None], xb.flip(-1), xb)
                yf = yb.clone()
                yf[..., 1] = -yb[..., 1]
                yf[..., 2], yf[..., 3] = yb[..., 3], yb[..., 2]
                yb = torch.where(flip[:, None, None], yf, yb)
            weight = None
            if n_real > 0:
                assert real_train is not None
                pick = rng.integers(0, len(real_train), size=n_real)
                rx, ry = real.augment_real(real_train.tensor[pick], real_train.label[pick], rng)
                xb = torch.cat([xb, torch.from_numpy(rx)])
                yb = torch.cat([yb, torch.from_numpy(ry)])
                weight = torch.from_numpy(
                    np.concatenate(
                        [
                            np.ones((n_sim, 16), np.float32),
                            station_weights(real_train.valid_count[pick]),
                        ]
                    )
                )
            loss = model.loss_fn(net(xb), yb, station_weight=weight)
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()
            running += float(loss)
        net.eval()
        with torch.no_grad():
            pred = torch.cat([net(xv[i : i + 2000]) for i in range(0, len(xv), 2000)])
            val_loss = float(model.loss_fn(pred, yv))
            xy_err = torch.hypot(pred[..., 0] - yv[..., 0], pred[..., 1] - yv[..., 1])
            w_err = (pred[..., 2:4] - yv[..., 2:4]).abs()
        row: dict[str, float] = {
            "epoch": epoch + 1,
            "train_loss": running / steps_per_epoch,
            "val_loss": val_loss,
            "val_xy_median_m": float(xy_err.median()),
            "val_width_median_m": float(w_err.median()),
            "seconds": time.time() - t0,
        }
        if real_val is not None:
            row.update(internal_validation(net, real_val))
        history.append(row)
        log.info(json.dumps(row))
    out.mkdir(parents=True, exist_ok=True)
    torch.save(net.state_dict(), out / "model.pt")
    config = {
        "train": str(train_path),
        "val": str(val_path),
        "epochs": epochs,
        "batch_size": batch_size,
        "lr": lr,
        "seed": seed,
        "hflip": hflip,
        "real_fraction": real_fraction,
        "init": str(init) if init else None,
        "real": real_info,
        "channels": list(model.CHANNELS),
        "hidden": model.HIDDEN,
        "parameters": model.count_parameters(net),
    }
    (out / "config.json").write_text(json.dumps(config, indent=2))
    (out / "history.json").write_text(json.dumps(history, indent=2))
    return {"config": config, "history": history}


def select_seed(runs: dict[int, list[dict[str, float]]]) -> dict[str, Any]:
    """The seed whose FINAL epoch has the lowest internal validation E_c (ties to the lower
    seed). Uses only the internal validation numbers; no held-out data is involved."""
    final = {seed: history[-1][SELECTION_KEY] for seed, history in runs.items()}
    finite = {s: v for s, v in final.items() if np.isfinite(v)}
    if not finite:
        raise ValueError("no seed has an internal validation E_c")
    best = min(sorted(finite), key=lambda s: finite[s])
    return {"criterion": f"final {SELECTION_KEY}", "per_seed": final, "selected_seed": best}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", type=Path, required=True)
    parser.add_argument("--val", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=2e-3)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--seeds", type=int, nargs="+", default=None)
    parser.add_argument("--hflip", action="store_true")
    parser.add_argument("--real", type=Path, nargs="+", default=None)
    parser.add_argument("--real-fraction", type=float, default=0.0)
    parser.add_argument("--split", type=Path, default=None)
    parser.add_argument("--init", type=Path, default=None)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    def run(seed: int, out: Path) -> list[dict[str, float]]:
        result = train(
            args.train,
            args.val,
            out,
            args.epochs,
            args.batch_size,
            args.lr,
            seed,
            args.hflip,
            args.real,
            args.real_fraction,
            args.split,
            args.init,
        )
        return result["history"]

    if args.seeds is None:
        run(args.seed, args.out)
        return 0
    runs = {seed: run(seed, args.out / f"seed{seed}") for seed in args.seeds}
    selection = select_seed(runs)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "selection.json").write_text(json.dumps(selection, indent=2))
    print(json.dumps(selection, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
