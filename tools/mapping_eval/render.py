"""PNG previews of saved maps (needs matplotlib: `uv run --project tools --extra plots`)."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from mapping_eval.quality import load_map_strict


def map_image(yaml_path: str | Path) -> tuple[np.ndarray, tuple[float, float, float, float]]:
    """RGB image (row 0 = max y) and its extent (x0, x1, y0, y1) in metres."""
    grid = load_map_strict(yaml_path)
    h, w = grid.shape
    img = np.full((h, w, 3), 0.78)
    img[grid.free] = 1.0
    img[grid.occupied] = 0.0
    x0, y0 = grid.origin_xy
    res = grid.resolution_m
    return np.flipud(img), (x0, x0 + w * res, y0, y0 + h * res)


def montage(
    items: list[tuple[str, Path]], out: Path, columns: int = 4, cell_in: float = 3.2
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = -(-len(items) // columns)
    fig, axes = plt.subplots(
        rows, columns, figsize=(cell_in * columns, cell_in * rows), squeeze=False
    )
    for ax in axes.ravel():
        ax.axis("off")
    for ax, (title, path) in zip(axes.ravel(), items, strict=False):
        img, extent = map_image(path)
        ax.imshow(img, extent=extent, interpolation="nearest")
        ax.set_title(title, fontsize=7)
    fig.tight_layout(pad=0.4)
    fig.savefig(out, dpi=110)
    plt.close(fig)
