"""Reader of the pre-registered constants of v0.3 (evaluation/prereg/pnp-v0.3.yaml).

The perception block of that file (scale augmentation range, the two folds, the moving-speed
threshold of bags without wheel odometry, the bootstrap) is read here and nowhere retyped. The file
is frozen: `load_v03` refuses a file whose SHA-256 is not the digest written into
docs/notes/preregistration-pnp-v0.3.md, so an edited copy cannot be used by accident.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any

import yaml
from racer_track_perception.vehicle import find_repo_root

# docs/notes/preregistration-pnp-v0.3.md: "Constants file: evaluation/prereg/pnp-v0.3.yaml, SHA-256".
V03_SHA256 = "4415b3719a29cfb08d5f5242ac6cdc5b35fee87e132c6d2939dfa0758808d7a1"
RELATIVE_PATH = Path("evaluation") / "prereg" / "pnp-v0.3.yaml"


class PreregError(ValueError):
    """The constants file is not the pre-registered one, or lacks what is read from it."""


def default_path() -> Path:
    return find_repo_root() / RELATIVE_PATH


def load_v03(path: Path | None = None, expected_sha256: str = V03_SHA256) -> dict[str, Any]:
    """The parsed constants file. Raises PreregError when its SHA-256 is not `expected_sha256`."""
    p = Path(path) if path is not None else default_path()
    data = p.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest != expected_sha256:
        raise PreregError(
            f"{p}: SHA-256 {digest} is not the pre-registered {expected_sha256}; the constants "
            "file must not change"
        )
    doc = yaml.safe_load(data.decode("utf-8"))
    if not isinstance(doc, dict) or "perception" not in doc:
        raise PreregError(f"{p}: no perception block")
    return doc


def scale_augmentation_range(doc: dict[str, Any] | None = None) -> tuple[float, float]:
    """(low, high) of the isotropic scale factor of the real samples (pre-registration 3.1)."""
    lo, hi = (doc or load_v03())["perception"]["scale_augmentation_range"]
    return float(lo), float(hi)


def moving_speed_min_mps(doc: dict[str, Any] | None = None) -> float:
    return float((doc or load_v03())["perception"]["moving_speed_min_mps"])
