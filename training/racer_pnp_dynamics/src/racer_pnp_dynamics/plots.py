"""The identification profile plot: held-out error against the command delay and the two time
constants (and the dead zone), with the fitted value marked. matplotlib is imported here only;
the package does not depend on it (`uv run --with matplotlib python -m racer_pnp_dynamics.cli`).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

PANELS = (
    ("command_delay_s", "command delay (s)", "speed RMSE (m/s)", False),
    ("speed_tau_s", "speed time constant (s)", "speed RMSE (m/s)", True),
    ("steer_tau_s", "steering time constant (s)", "yaw RMSE per 0.1 s (rad)", True),
    ("speed_deadband_mps", "speed dead zone (m/s)", "speed RMSE (m/s)", False),
)


def write_profile_png(
    path: Path, profiles: dict[str, list[dict[str, float]]], fitted: dict[str, float],
    unidentified: dict[str, str], title: str,
) -> None:  # fmt: skip
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(10, 7), constrained_layout=True)
    for ax, (name, xlabel, ylabel, log_x) in zip(axes.ravel(), PANELS, strict=True):
        rows = profiles.get(name, [])
        x = [r["value"] for r in rows]
        y = [r["rmse"] for r in rows]
        ax.plot(x, y, "o-", ms=3)
        if name in fitted:
            ax.axvline(fitted[name], color="C3", ls="--", label=f"shipped {fitted[name]:.3g}")
        if rows:
            best = min(y)
            ax.axhline(best * 1.02, color="0.6", lw=0.8, ls=":", label="2% over the minimum")
        if log_x:
            ax.set_xscale("log")
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        status = "NOT identified: " + unidentified[name] if name in unidentified else "identified"
        ax.set_title(f"{name}\n{status}", fontsize=8)
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)
    fig.suptitle(title)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_from_metrics(path: Path, metrics: dict[str, Any], nominal: dict[str, float], title: str):
    write_profile_png(path, metrics["profiles"], nominal, metrics["unidentified"], title)
