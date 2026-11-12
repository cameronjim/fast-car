"""Self-labels on 100 real scans (10 s of the gap follow laps of the 2026-11-12 bag, a corner of
the oval), cut by `cut_fixture --run`."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from racer_pnp_perception import selflabel as sl
from racer_track_perception.vehicle import LiveVehicle

FIXTURE = Path(__file__).parent / "fixtures" / "selflabel_segment_L_2026-11-12.npz"


def load_fixture() -> sl.ScanSet:
    with np.load(FIXTURE) as z:
        ranges = z["ranges_mm"].astype(np.float32) / 1000.0
        ranges[z["ranges_mm"] == 0] = np.inf
        return sl.ScanSet(
            z["stamp_s"],
            ranges,
            float(z["angle_min"]),
            float(z["angle_increment"]),
            float(z["range_min"]),
            float(z["range_max"]),
        )


@pytest.fixture(scope="module")
def real_run(live: LiveVehicle):
    """The scans and the ICP motion between them (about 20 s of ICP)."""
    scans = load_fixture()
    return scans, sl.compute_motion(scans, live)


def test_fixture_is_small() -> None:
    assert FIXTURE.stat().st_size < 300_000


def test_real_scans_motion_and_labels(live: LiveVehicle, real_run) -> None:
    """On real scans the matching works and every label kept satisfies every gate. The yield is
    well under what the simulated oval gives: about a quarter of the 100 scans are kept (the last
    54 have no 4 m ahead, so no driven path), and the others are dropped by the path and
    deficit gates where the car cut the corner or the island end is sparse, not by the matching."""
    scans, series = real_run
    assert series.ok.mean() >= 0.95
    labels = sl.make_labels(scans, series, live, np.ones(len(scans), dtype=bool))
    assert len(labels) >= 20, labels.dropped
    assert sum(labels.dropped.values()) + len(labels) == len(scans)
    assert (labels.valid_count >= sl.MIN_VALID_COUNT).all()
    assert (labels.intrusion <= sl.MAX_INTRUSION).all()
    assert (labels.deficit_p90_m <= sl.MAX_DEFICIT_P90_M).all()
    assert (labels.path_containment >= sl.MIN_PATH_CONTAINMENT).all()
    assert np.isfinite(labels.label).all() and labels.label.shape[1:] == (16, 5)
    # The contract: station 0 within 0.35 m of the car, travelling forward.
    assert (np.hypot(labels.label[:, 0, 0], labels.label[:, 0, 1]) < 0.35).all()
    assert (labels.label[:, 4, 0] > labels.label[:, 0, 0]).all()


def test_real_scans_label_deterministically(live: LiveVehicle, real_run) -> None:
    scans, series = real_run
    cand = np.zeros(len(scans), dtype=bool)
    cand[:30] = True
    a = sl.make_labels(scans, series, live, cand)
    b = sl.make_labels(scans, series, live, cand)
    assert np.array_equal(a.index, b.index) and np.array_equal(a.label, b.label)
