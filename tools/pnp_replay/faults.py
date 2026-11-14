"""Fault injection on a recording: bag mutations for the replay (12-testing.md, layer L4).

Each mutation takes a `Recording` (the dict of numpy arrays of bag.py), a time window
[t0, t1] in bag seconds (ARRIVAL time, the axis of `scan_t` and `odom_t`) and a seed, and
returns a new recording: the input is never changed. Rows outside the window are untouched.
`targets` says which sensor streams a fault hits: "scan", "odom" (wheel odometry), or both.

    nan_ranges            scan: a share (`fraction`) of the beams of every scan in the window set
                          to NaN, inf or 0 (`fill`); odometry: speed NaN
    drop                  rows of the window removed
    stamp_jump_forward    header stamps of the window plus STAMP_JUMP_S, arrival unchanged
    stamp_jump_backward   header stamps of the window minus STAMP_JUMP_S
    out_of_order          adjacent pairs of rows of the window swap places in arrival order
                          (each message keeps its own stamp and payload)
    frozen                the rows of the window repeat the last row before the window, stamp
                          and payload (a stuck driver resending its last message)
    frozen_fresh_stamp    the payload repeats, the stamp keeps advancing (a stuck sensor that
                          still talks, and gets a fresh stamp from the driver)
    track_garbage         the recorded /pnp/local_track of the window replaced (planner only)

Odometry has no header stamp in the recording (the planner stamps wheel odometry with its
ARRIVAL time, `on_odom` in mppi_planner_node and the `O` line of the replay stream), so the
stamp faults write an `odom_stamp` array that the replay stream does not carry: they change what a
driver stamp would say and nothing the planner can see. The planner cannot detect a stuck or
mis-stamped odometry stream: that is measured, not assumed (docs/notes/pnp-replay.md).

Float scan ranges that hold NaN and 0 do not fit the recording's uint16 quarter millimetres, so a
mutated recording carries them as `scan_f` (bag.scan_ranges reads it first).
"""

from __future__ import annotations

import logging
from collections.abc import Iterable
from typing import Any

import numpy as np
from numpy.typing import NDArray

from pnp_replay.bag import Recording, dequantise_ranges, group_of

log = logging.getLogger(__name__)

STAMP_JUMP_S = 5.0
NAN_FRACTIONS = (0.1, 0.5, 1.0)
NAN_FILLS = ("nan", "inf", "zero")
TARGETS = ("scan", "odom")
SCAN_FAULTS = (
    "nan_ranges",
    "drop",
    "stamp_jump_forward",
    "stamp_jump_backward",
    "out_of_order",
    "frozen",
    "frozen_fresh_stamp",
)
TRACK_GARBAGE_KINDS = ("nan", "valid_out_of_range")
# Faults on the local track that the replay stream cannot carry: the `T` line of planner_replay_cli
# has a fixed 16 stations, frame base_link and spacing 0.20 m, so a track of another length, frame
# or spacing cannot be sent to the real planner through it (the node's own validity check, rows
# WrongSize, WrongFrame and WrongSpacing of local_track.cpp, covers them in its gtest).
TRACK_GARBAGE_NOT_INJECTABLE = ("wrong_length", "wrong_frame", "spacing_0.19")


class NotInjectable(ValueError):
    """The fault cannot be expressed in the replay stream (see TRACK_GARBAGE_NOT_INJECTABLE)."""


def _copy(rec: Recording) -> Recording:
    return {k: np.array(v, copy=True) for k, v in rec.items()}


def _check_window(t0: float, t1: float) -> None:
    if not (np.isfinite(t0) and np.isfinite(t1)) or t1 < t0:
        raise ValueError(f"window [{t0}, {t1}] must be finite with t1 >= t0")


def _check_targets(targets: Iterable[str]) -> tuple[str, ...]:
    t = tuple(targets)
    bad = [x for x in t if x not in TARGETS]
    if bad or not t:
        raise ValueError(f"targets must be a non-empty subset of {TARGETS}, got {t}")
    return t


def rows_in(rec: Recording, group: str, t0: float, t1: float) -> NDArray[np.int64]:
    """Indices of the rows of `group` whose arrival time is in [t0, t1]."""
    t = rec[f"{group}_t"]
    return np.nonzero((t >= t0) & (t <= t1))[0].astype(np.int64)


def window_of_scans(rec: Recording, t_start: float, n: int) -> tuple[float, float]:
    """A window that holds exactly `n` scans: from the first scan arriving at or after t_start,
    up to a hair before the arrival of the one after the n-th."""
    t = rec["scan_t"]
    first = int(np.searchsorted(t, t_start))
    if first + n >= len(t):
        raise ValueError(f"fewer than {n + 1} scans after {t_start}")
    return float(t[first]), float((t[first + n - 1] + t[first + n]) / 2.0)


def window_of_odom(rec: Recording, t_start: float, duration_s: float) -> tuple[float, float]:
    """A window that holds the odometry samples spanning `duration_s`: from the first sample at
    or after t_start, round(duration_s / median period) samples, closed halfway to the next one
    (60 ms of a 50 Hz stream is 3 samples, 1.0 s is 50)."""
    t = rec["odom_t"]
    n = max(1, round(duration_s / float(np.median(np.diff(t)))))
    first = int(np.searchsorted(t, t_start))
    if first + n >= len(t):
        raise ValueError(f"fewer than {n + 1} odometry samples after {t_start}")
    return float(t[first]), float((t[first + n - 1] + t[first + n]) / 2.0)


def _payload_keys(rec: Recording, group: str, stamps: bool = True) -> list[str]:
    """The per-row arrays of a group except its arrival time (and, with stamps False, except its
    header stamps)."""
    out = []
    for key, value in rec.items():
        if group_of(key) != group or key == f"{group}_t" or value.ndim < 1:
            continue
        if not stamps and key.endswith("_stamp"):
            continue
        if len(value) == len(rec[f"{group}_t"]):
            out.append(key)
    return out


def _with_odom_stamp(rec: Recording) -> None:
    """Give a (copied) recording an `odom_stamp` array equal to the arrival times."""
    if "odom_stamp" not in rec:
        rec["odom_stamp"] = np.array(rec["odom_t"], copy=True)


def _with_float_scans(rec: Recording) -> None:
    if "scan_f" not in rec:
        rec["scan_f"] = (
            np.stack([dequantise_ranges(q) for q in rec["scan_q"]])
            if len(rec["scan_q"])
            else np.zeros((0, rec["scan_q"].shape[1]), np.float32)
        )


def _drop_rows(rec: Recording, group: str, idx: NDArray[np.int64]) -> None:
    keep = np.ones(len(rec[f"{group}_t"]), dtype=bool)
    keep[idx] = False
    n = len(keep)
    for key in [f"{group}_t", *_payload_keys(rec, group)]:
        if len(rec[key]) == n:
            rec[key] = rec[key][keep]


def _hit_groups(rec: Recording, targets: tuple[str, ...]) -> list[str]:
    return [g for g in targets if f"{g}_t" in rec]


def nan_ranges(
    rec: Recording,
    t0: float,
    t1: float,
    seed: int,
    fraction: float = 0.5,
    fill: str = "nan",
    targets: Iterable[str] = TARGETS,
) -> Recording:
    """Scan: `fraction` of the beams of each scan in the window (a fresh random choice per scan)
    set to NaN, inf or 0. Odometry: the speed of every sample in the window NaN."""
    _check_window(t0, t1)
    if not 0.0 < fraction <= 1.0:
        raise ValueError(f"fraction must be in (0, 1], got {fraction}")
    if fill not in NAN_FILLS:
        raise ValueError(f"fill must be one of {NAN_FILLS}, got {fill!r}")
    tg = _check_targets(targets)
    out = _copy(rec)
    rng = np.random.default_rng(seed)
    if "scan" in tg:
        _with_float_scans(out)
        value = {"nan": np.nan, "inf": np.inf, "zero": 0.0}[fill]
        n_beams = out["scan_f"].shape[1]
        count = round(fraction * n_beams)
        for i in rows_in(out, "scan", t0, t1):
            beams = rng.choice(n_beams, size=count, replace=False)
            out["scan_f"][i, beams] = np.float32(value)
    if "odom" in tg:
        out["odom_v"][rows_in(out, "odom", t0, t1)] = np.nan
    return out


def drop(
    rec: Recording, t0: float, t1: float, seed: int, targets: Iterable[str] = TARGETS
) -> Recording:
    """Remove every row of the targeted streams whose arrival is in the window."""
    _check_window(t0, t1)
    tg = _check_targets(targets)
    out = _copy(rec)
    for g in _hit_groups(out, tg):
        _drop_rows(out, g, rows_in(out, g, t0, t1))
    return out


def _stamp_jump(
    rec: Recording, t0: float, t1: float, targets: Iterable[str], delta_s: float
) -> Recording:
    _check_window(t0, t1)
    tg = _check_targets(targets)
    out = _copy(rec)
    if "scan" in tg:
        out["scan_stamp"][rows_in(out, "scan", t0, t1)] += delta_s
    if "odom" in tg:
        _with_odom_stamp(out)
        out["odom_stamp"][rows_in(out, "odom", t0, t1)] += delta_s
    return out


def stamp_jump_forward(
    rec: Recording, t0: float, t1: float, seed: int, targets: Iterable[str] = TARGETS
) -> Recording:
    """Header stamps of the window plus STAMP_JUMP_S; arrival times unchanged."""
    return _stamp_jump(rec, t0, t1, targets, +STAMP_JUMP_S)


def stamp_jump_backward(
    rec: Recording, t0: float, t1: float, seed: int, targets: Iterable[str] = TARGETS
) -> Recording:
    """Header stamps of the window minus STAMP_JUMP_S; arrival times unchanged."""
    return _stamp_jump(rec, t0, t1, targets, -STAMP_JUMP_S)


def out_of_order(
    rec: Recording, t0: float, t1: float, seed: int, targets: Iterable[str] = TARGETS
) -> Recording:
    """Swap adjacent pairs of the window's messages in arrival order: the 1st and 2nd rows of the
    window trade payload and stamp, so do the 3rd and 4th. An odd last row stays."""
    _check_window(t0, t1)
    tg = _check_targets(targets)
    out = _copy(rec)
    if "odom" in tg:
        _with_odom_stamp(out)
    for g in _hit_groups(out, tg):
        idx = rows_in(out, g, t0, t1)
        pairs = len(idx) // 2
        a, b = idx[0 : 2 * pairs : 2], idx[1 : 2 * pairs : 2]
        for key in _payload_keys(out, g):
            out[key][a], out[key][b] = out[key][b].copy(), out[key][a].copy()
    return out


def _freeze(
    rec: Recording, t0: float, t1: float, targets: Iterable[str], stamps: bool
) -> Recording:
    _check_window(t0, t1)
    tg = _check_targets(targets)
    out = _copy(rec)
    if "odom" in tg:
        _with_odom_stamp(out)
    for g in _hit_groups(out, tg):
        idx = rows_in(out, g, t0, t1)
        if len(idx) == 0:
            continue
        source = idx[0] - 1 if idx[0] > 0 else idx[0]  # the last row before the window
        for key in _payload_keys(out, g, stamps=stamps):
            out[key][idx] = out[key][source]
    return out


def frozen(
    rec: Recording, t0: float, t1: float, seed: int, targets: Iterable[str] = TARGETS
) -> Recording:
    """Every row of the window becomes a copy of the last row before it (payload AND stamp),
    arriving at its own recorded time."""
    return _freeze(rec, t0, t1, targets, stamps=True)


def frozen_fresh_stamp(
    rec: Recording, t0: float, t1: float, seed: int, targets: Iterable[str] = TARGETS
) -> Recording:
    """Every row of the window repeats the payload of the last row before it and keeps its own
    header stamp: a stuck sensor that still talks."""
    return _freeze(rec, t0, t1, targets, stamps=False)


def track_garbage(rec: Recording, t0: float, t1: float, seed: int, kind: str = "nan") -> Recording:
    """The recorded /pnp/local_track messages that arrive in the window replaced by garbage.
    kind "nan": every array NaN and the scalars NaN, valid_count kept (a track that claims 16
    stations and has none); "valid_out_of_range": valid_count 17 (above the 16 stations of the
    contract), the arrays kept. The kinds of TRACK_GARBAGE_NOT_INJECTABLE raise NotInjectable."""
    _check_window(t0, t1)
    if kind in TRACK_GARBAGE_NOT_INJECTABLE:
        raise NotInjectable(
            f"track garbage {kind!r} cannot be sent through the planner_replay_cli stream "
            "(16 stations, frame base_link, spacing 0.20 m are fixed there)"
        )
    if kind not in TRACK_GARBAGE_KINDS:
        raise ValueError(f"kind must be one of {TRACK_GARBAGE_KINDS}, got {kind!r}")
    out = _copy(rec)
    idx = rows_in(out, "trk", t0, t1)
    if kind == "nan":
        for key in ("trk_x", "trk_y", "trk_hl", "trk_hr", "trk_sigma"):
            out[key][idx] = np.nan
        out["trk_residual"][idx] = np.nan
        out["trk_intrusion"][idx] = np.nan
    else:
        out["trk_valid"][idx] = 17
    return out


FAULTS: dict[str, Any] = {
    "nan_ranges": nan_ranges,
    "drop": drop,
    "stamp_jump_forward": stamp_jump_forward,
    "stamp_jump_backward": stamp_jump_backward,
    "out_of_order": out_of_order,
    "frozen": frozen,
    "frozen_fresh_stamp": frozen_fresh_stamp,
}


def apply(
    rec: Recording,
    fault: str,
    t0: float,
    t1: float,
    seed: int,
    targets: Iterable[str] = TARGETS,
    **params: Any,
) -> Recording:
    """Apply a sensor fault by name (`params`: fraction and fill for nan_ranges)."""
    if fault not in FAULTS:
        raise ValueError(f"fault must be one of {tuple(FAULTS)}, got {fault!r}")
    return FAULTS[fault](rec, t0, t1, seed, targets=targets, **params)
