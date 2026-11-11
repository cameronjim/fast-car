"""Train circ_conv1d_v0 on the simulated dataset (CPU, a few minutes).

    uv run --group train python -m racer_pnp_perception.train \\
        --train <train.npz> --val <val.npz> --out <run dir> [--epochs 12]

Adam with a cosine learning-rate decay; loss is `model.loss_fn` (Gaussian negative log likelihood
on x, y with the predicted sigma per station, Huber on the two half widths). Writes
`<out>/model.pt`, `<out>/config.json` (every hyperparameter and the data files) and
`<out>/history.json`. torch is imported inside `train`.
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

from racer_pnp_perception import dataset, model

log = logging.getLogger(__name__)


def train(
    train_path: Path,
    val_path: Path,
    out: Path,
    epochs: int = 12,
    batch_size: int = 256,
    lr: float = 2e-3,
    seed: int = 0,
    hflip: bool = False,
) -> dict[str, Any]:
    import torch

    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    x_train, y_train = dataset.load(train_path)
    x_val, y_val = dataset.load(val_path)
    net = model.build_model()
    log.info("parameters: %d", model.count_parameters(net))
    opt = torch.optim.Adam(net.parameters(), lr=lr)
    steps_per_epoch = len(x_train) // batch_size
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
            idx = order[b * batch_size : (b + 1) * batch_size]
            xb, yb = torch.from_numpy(x_train[idx]), torch.from_numpy(y_train[idx])
            if hflip:
                flip = torch.rand(len(idx)) < 0.5
                xb = torch.where(flip[:, None, None], xb.flip(-1), xb)
                yf = yb.clone()
                yf[..., 1] = -yb[..., 1]
                yf[..., 2], yf[..., 3] = yb[..., 3], yb[..., 2]
                yb = torch.where(flip[:, None, None], yf, yb)
            loss = model.loss_fn(net(xb), yb)
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
        row = {
            "epoch": epoch + 1,
            "train_loss": running / steps_per_epoch,
            "val_loss": val_loss,
            "val_xy_median_m": float(xy_err.median()),
            "val_width_median_m": float(w_err.median()),
            "seconds": time.time() - t0,
        }
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
        "channels": list(model.CHANNELS),
        "hidden": model.HIDDEN,
        "parameters": model.count_parameters(net),
    }
    (out / "config.json").write_text(json.dumps(config, indent=2))
    (out / "history.json").write_text(json.dumps(history, indent=2))
    return {"config": config, "history": history}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", type=Path, required=True)
    parser.add_argument("--val", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=2e-3)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--hflip", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    train(
        args.train, args.val, args.out, args.epochs, args.batch_size, args.lr, args.seed, args.hflip
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
