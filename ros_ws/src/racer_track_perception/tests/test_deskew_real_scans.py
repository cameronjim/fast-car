"""L4: 30 real corner scans (cut from the train segments R2 and R7a of the 2026-11-13 bag) with the
wheel speed and steering of their time, through the deskew relay, against a committed golden
output, plus fault injection on the stream (dropped odometry, a timestamp that jumps back, frozen
odometry): the scan passes through, nothing crashes and no scan is lost.

The golden is made with stamp_is sweep_start and beam_order index_descending, a TEST
configuration (make_deskew_fixture.py). Tolerance 0.005 m per beam: the golden is the same code on
the same stored stream, so a difference above float noise is a change of behaviour.
"""

from __future__ import annotations

import numpy as np
import pytest
from make_deskew_fixture import GOLDEN, SCANS, dequantise, run_relay

TOLERANCE_M = 0.005
MIN_BEAMS_MOVED_M = 0.02  # a corner scan at about 1 m/s must change by more than this somewhere


@pytest.fixture(scope="module")
def data() -> dict[str, np.ndarray]:
    with np.load(SCANS) as z:
        return {k: z[k] for k in z.files}


def test_fixture_is_small(data) -> None:
    assert SCANS.stat().st_size < 300_000 and GOLDEN.stat().st_size < 300_000
    assert data["scan_q"].shape == (30, 720)


def test_the_relay_matches_the_golden_output(data) -> None:
    got, offset, reason = run_relay(data)
    with np.load(GOLDEN) as g:
        want, want_offset, want_reason = g["ranges"], g["offset"], list(g["reason"])
    assert reason == want_reason
    assert np.allclose(offset, want_offset, atol=1e-9)
    both = np.isfinite(got) & np.isfinite(want)
    assert np.array_equal(np.isfinite(got), np.isfinite(want))
    assert np.abs(got[both] - want[both]).max() <= TOLERANCE_M


def test_the_golden_is_a_deskew_not_a_copy_of_the_input(data) -> None:
    with np.load(GOLDEN) as g:
        want, reason = g["ranges"], list(g["reason"])
    assert not any(reason), "every fixture scan is a moving scan and must be deskewed"
    raw = np.stack([dequantise(q) for q in data["scan_q"]])
    both = np.isfinite(raw) & np.isfinite(want)
    with np.errstate(invalid="ignore"):
        moved = np.abs(raw - want)
    moved[~both] = 0.0
    assert (moved.max(axis=1) > MIN_BEAMS_MOVED_M).all()


def test_output_is_well_formed(data) -> None:
    got, _offset, _reason = run_relay(data)
    assert got.shape == data["scan_q"].shape
    assert not np.isnan(got).any()
    finite = got[np.isfinite(got)]
    assert (finite >= float(data["range_min"])).all() and (finite <= float(data["range_max"])).all()


# -- fault injection ------------------------------------------------------------------------------------


def _faulted(data: dict[str, np.ndarray], kind: str) -> dict[str, np.ndarray]:
    d = {k: np.array(v, copy=True) for k, v in data.items()}
    if kind == "dropped_odometry":
        for key in ("odom_t", "odom_stamp", "odom_v"):
            d[key] = d[key][:0]
    elif kind == "stamp_jump_back":
        d["odom_stamp"] = d["odom_stamp"] - 5.0  # the odometry's clock jumps back 5 s
        d["odom_stamp"][0] = data["odom_stamp"][0]  # the first sample is accepted, then none
    elif kind == "frozen_odometry":
        # a stuck driver resends its first message: same stamp, same speed, arrival still advances
        d["odom_stamp"][:] = d["odom_stamp"][0]
        d["odom_v"][:] = d["odom_v"][0]
    else:  # pragma: no cover
        raise AssertionError(kind)
    return d


@pytest.mark.parametrize("kind", ["dropped_odometry", "stamp_jump_back", "frozen_odometry"])
def test_odometry_faults_pass_every_scan_through_and_lose_none(data, kind) -> None:
    got, offset, reason = run_relay(_faulted(data, kind))
    n = len(data["scan_t"])
    assert got.shape[0] == n, "one output per input"
    assert all(reason), f"{kind}: a scan was deskewed on a faulty stream: {reason}"
    raw = np.stack([dequantise(q) for q in data["scan_q"]])
    assert np.array_equal(got, raw), "a pass-through scan is the input, unchanged"
    assert np.all(offset == 0.0)
    expected = {"dropped_odometry": "no_odom"}.get(kind, "twist_stale")
    assert set(reason) <= {expected, "no_odom"}, set(reason)


def test_a_scan_stream_with_nan_ranges_never_crashes(data) -> None:
    d = {k: np.array(v, copy=True) for k, v in data.items()}
    d["scan_q"][:, ::3] = 0  # a third of every scan is not a return
    got, _offset, _reason = run_relay(d)
    assert got.shape[0] == 30 and not np.isnan(got).any()
