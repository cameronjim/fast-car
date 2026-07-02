"""offline behavioural cloning training on the preprocessed csv."""
from __future__ import annotations

import argparse
import os
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
from model import BCNet


def load_data(csv_path: str) -> tuple[np.ndarray, np.ndarray, int]:
    """split the csv into lidar features and action labels, with the ray count."""
    header = pd.read_csv(csv_path, nrows=0)
    # sorted numerically, since the csv column order is not guaranteed to be ray order
    lidar_cols = sorted(
        [c for c in header.columns if c.startswith("lidar_")],
        key=lambda c: int(c.split("_")[1]),
    )
    use_cols = lidar_cols + ["steering_angle", "speed"]

    chunks = []
    for chunk in pd.read_csv(csv_path, usecols=use_cols, chunksize=100_000):
        chunks.append(chunk)
    df = pd.concat(chunks, ignore_index=True)

    scans = df[lidar_cols].values.astype(np.float32)
    actions = df[["steering_angle", "speed"]].values.astype(np.float32)
    return scans, actions, len(lidar_cols)


def make_loaders(scans: np.ndarray, actions: np.ndarray, train_ratio: float = 0.8,
                 batch_size: int = 256) -> tuple[DataLoader, DataLoader]:
    """shuffled train and validation loaders over a random split."""
    indices = np.random.permutation(len(scans))
    split = int(len(scans) * train_ratio)
    train_idx, val_idx = indices[:split], indices[split:]

    train_ds = TensorDataset(
        torch.from_numpy(scans[train_idx]), torch.from_numpy(actions[train_idx])
    )
    val_ds = TensorDataset(
        torch.from_numpy(scans[val_idx]), torch.from_numpy(actions[val_idx])
    )

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=batch_size)

    return train_loader, val_loader


def train(args: argparse.Namespace) -> None:
    scans, actions, num_lidar = load_data(args.data)
    print(f"loaded {len(scans)} samples, {num_lidar} lidar rays")

    train_loader, val_loader = make_loaders(scans, actions, batch_size=args.batch_size)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"using device: {device}")

    model = BCNet(num_lidar_rays=num_lidar).to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    criterion = nn.MSELoss()

    best_val_loss = float("inf")

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss = 0.0

        for xb, yb in train_loader:
            xb, yb = xb.to(device), yb.to(device)
            pred = model(xb)
            loss = criterion(pred, yb)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            train_loss += loss.item() * len(xb)

        train_loss /= len(train_loader.dataset)

        model.eval()
        val_loss = 0.0

        with torch.no_grad():
            for xb, yb in val_loader:
                xb, yb = xb.to(device), yb.to(device)
                val_loss += criterion(model(xb), yb).item() * len(xb)

        val_loss /= len(val_loader.dataset)

        print(f"epoch {epoch:3d}/{args.epochs}  train_loss={train_loss:.6f}  val_loss={val_loss:.6f}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
            torch.save(model.state_dict(), args.out)
            print(f"  best model saved, val_loss={best_val_loss:.6f}")

    print(f"\ntraining complete, best val_loss={best_val_loss:.6f}")
    print(f"model saved to {args.out}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train BC model")
    parser.add_argument("--data", required=True, help="Path to processed CSV")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--out", default="bc/bc_model.pth", help="Output model path")
    args = parser.parse_args()
    train(args)
