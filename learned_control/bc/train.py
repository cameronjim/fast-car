"""Training script for the Behavioural Cloning model.

Usage:
    python bc/train.py --data processed/data.csv --epochs 100 --batch-size 256 --lr 1e-3 --out bc/bc_model.pth
"""
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
    """
    Load CSV and split into LiDAR features and action labels.

    Args:
        csv_path: The path to the CSV file.

    Returns:
        A tuple containing the LiDAR features, action labels, and the number of LiDAR rays.
    """
    # Read header to identify columns
    header = pd.read_csv(csv_path, nrows=0)
    lidar_cols = sorted(
        [c for c in header.columns if c.startswith("lidar_")],
        key=lambda c: int(c.split("_")[1]),
    )
    # Define the columns to use
    use_cols = lidar_cols + ["steering_angle", "speed"]

    chunks = []
    # Read the CSV file in chunks to avoid memory issues
    for chunk in pd.read_csv(csv_path, usecols=use_cols, chunksize=100_000):
        chunks.append(chunk)
    df = pd.concat(chunks, ignore_index=True)

    # Convert the data to numpy arrays
    X = df[lidar_cols].values.astype(np.float32)
    y = df[["steering_angle", "speed"]].values.astype(np.float32)
    return X, y, len(lidar_cols)


def make_loaders(X: np.ndarray, y: np.ndarray, train_ratio: float = 0.8, batch_size: int = 256) -> tuple[DataLoader, DataLoader]:
    """
    Create train and validation data loaders. 80/20 train/val split.

    Args:
        X: The LiDAR features.
        y: The action labels.
        train_ratio: The ratio of the data to use for training.
        batch_size: The batch size.

    Returns:
        A tuple containing the train and validation data loaders.
    """
    # Get the number of samples and split the indices
    n = len(X)
    indices = np.random.permutation(n)
    split = int(n * train_ratio)
    train_idx, val_idx = indices[:split], indices[split:]

    # Create the training and validation datasets
    train_ds = TensorDataset(
        torch.from_numpy(X[train_idx]), torch.from_numpy(y[train_idx])
    )

    # Create the validation dataset
    val_ds = TensorDataset(
        torch.from_numpy(X[val_idx]), torch.from_numpy(y[val_idx])
    )

    # Create the training and validation data loaders
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=batch_size)

    # Return the training and validation data loaders
    return train_loader, val_loader


def train(args: argparse.Namespace) -> None:
    """
    Train the Behavioural Cloning model.

    Args:
        args: The command line arguments.

    Returns:
        None
    """
    # Load the data
    X, y, num_lidar = load_data(args.data)
    print(f"Loaded {len(X)} samples, {num_lidar} LiDAR rays")

    # Create the training and validation data loaders
    train_loader, val_loader = make_loaders(X, y, batch_size=args.batch_size)

    # Get the device to use
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Create the model
    model = BCNet(num_lidar_rays=num_lidar).to(device)

    # Create the optimizer and MSE loss function
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    criterion = nn.MSELoss()

    # Initialize the best validation loss
    best_val_loss = float("inf")

    # Train the model
    for epoch in range(1, args.epochs + 1):
        # Set the model to training mode
        model.train()
        train_loss = 0.0

        # Train the model
        for xb, yb in train_loader:

            # Move the data to the device and make the predictions
            xb, yb = xb.to(device), yb.to(device)
            pred = model(xb)
            loss = criterion(pred, yb)

            # Zero the gradients and backpropagate
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            # Update the training loss
            train_loss += loss.item() * len(xb)

        # Calculate the average training loss
        train_loss /= len(train_loader.dataset)

        # Set the model to evaluation mode
        model.eval()
        val_loss = 0.0

        # Calculate the validation loss
        with torch.no_grad():
            for xb, yb in val_loader:
                xb, yb = xb.to(device), yb.to(device)
                val_loss += criterion(model(xb), yb).item() * len(xb)

        # Calculate the average validation loss
        val_loss /= len(val_loader.dataset)

        # Print the training and validation loss
        print(f"Epoch {epoch:3d}/{args.epochs}  train_loss={train_loss:.6f}  val_loss={val_loss:.6f}")

        # Save the best model if the validation loss is lower
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
            # Save the model
            torch.save(model.state_dict(), args.out)
            print(f"  -> saved best model (val_loss={best_val_loss:.6f})")

    # Print logs for the training completion
    print(f"\nTraining complete. Best val_loss={best_val_loss:.6f}")
    print(f"Model saved to {args.out}")


if __name__ == "__main__":
    """
    Main function to train the Behavioural Cloning model.

    Args:
        None

    Returns:
        None
    """
    # Create the argument parser
    parser = argparse.ArgumentParser(description="Train BC model")
    parser.add_argument("--data", required=True, help="Path to processed CSV")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--out", default="bc/bc_model.pth", help="Output model path")
    args = parser.parse_args()
    train(args)
