"""L4: the committed real-scan fixtures through the REFINED net backend against a committed golden
(new, brief 32). The goldens of the unrefined net and of the geometric tracer are untouched and
are still checked by test_real_scans.py."""

from __future__ import annotations

import numpy as np
import pytest
from make_refined_golden import GOLDEN, KEYS, run
from racer_track_perception.vehicle import LiveVehicle

TOLERANCE_M = 0.02


@pytest.fixture(scope="module")
def got(live: LiveVehicle) -> dict[str, np.ndarray]:
    return run(live)


def test_golden_is_small() -> None:
    assert GOLDEN.stat().st_size < 100_000


@pytest.mark.parametrize("tag", ["1111", "1113"])
def test_refined_backend_matches_the_golden(got: dict[str, np.ndarray], tag: str) -> None:
    with np.load(GOLDEN) as g:
        want = {k: g[k] for k in g.files}
    assert np.array_equal(got[f"{tag}_valid"], want[f"{tag}_valid"])
    for key in ("x", "y", "hl", "hr"):
        a, b = got[f"{tag}_{key}"], want[f"{tag}_{key}"]
        for i, n in enumerate(want[f"{tag}_valid"]):
            diff = np.abs(a[i, :n] - b[i, :n])
            assert diff.size == 0 or diff.max() <= TOLERANCE_M, (tag, key, i, diff.max())


@pytest.mark.parametrize("tag", ["1111", "1113"])
def test_refined_real_scans_mostly_give_a_track_and_stay_finite(
    got: dict[str, np.ndarray], tag: str
) -> None:
    valid = got[f"{tag}_valid"]
    assert (valid >= 6).mean() > 0.9
    for key in KEYS:
        assert np.isfinite(got[f"{tag}_{key}"]).all()
