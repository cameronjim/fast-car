"""L1: the windowed read decodes only the heldout windows and never a reserve segment."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from racer_pnp_perception import split_v04, windowed

DOC = yaml.safe_load(
    """
prereg_id: x
split_2026_11_14:
  bags:
    bagC:
      first_scan_log_time_s: 1000.0
      segments:
        - {name: X0, from: 0.0, to: 50.0, perception: reserve}
        - {name: X2b, from: 52.0, to: 80.0, direction: cw, perception: heldout}
        - {name: X3a, from: 90.0, to: 100.0, perception: reserve}
        - {name: X3b, from: 102.0, to: 110.0, direction: ccw, perception: heldout}
"""
)
ORIGIN = 995.0  # the file's first message is 5 s before the first scan


def fake_read_bag(calls: list):
    def read(bag_dir, window_s):
        calls.append(window_s)
        lo, hi = window_s
        scans = []
        for t_file in np.arange(lo - 1.0, hi + 1.0, 0.5):  # chunk edges spill over the window
            log = ORIGIN + t_file
            scans.append(
                SimpleNamespace(
                    log_time_s=log,
                    stamp_s=log,
                    angle_min=-3.0,
                    angle_increment=0.01,
                    range_min=0.05,
                    range_max=12.0,
                    ranges=np.ones(8, dtype=np.float32),
                )
            )
        odom = [SimpleNamespace(stamp_s=ORIGIN + lo, speed_mps=1.0)]
        return SimpleNamespace(scans=scans, odom=odom)

    return read


def test_only_heldout_windows_are_requested_and_none_touches_reserve(tmp_path: Path) -> None:
    split = split_v04.perception_split_from_doc(DOC, "x")
    calls: list = []
    path = windowed.build_windowed_cache(
        tmp_path / "bagC", split, tmp_path / "cache", fake_read_bag(calls), ORIGIN, DOC
    )
    shift = 1000.0 - ORIGIN
    assert calls == [(52.0 + shift, 80.0 + shift), (102.0 + shift, 110.0 + shift)]
    with np.load(path) as z:
        t = z["log_s"]
        assert t.min() >= 52.0 and t.max() <= 110.0
        assert not np.any((t > 80.0) & (t < 102.0))  # nothing between the windows
        for _b, lo, hi in split_v04.reserve_intervals(DOC):
            assert not np.any((t >= lo) & (t <= hi))
        assert z["ranges"].shape[0] == len(t) == len(z["stamp_s"])
        assert str(z["speed_source"]) == "odom"


def test_a_window_touching_reserve_is_refused(tmp_path: Path) -> None:
    doc = yaml.safe_load(yaml.safe_dump(DOC))
    doc["split_2026_11_14"]["bags"]["bagC"]["segments"][2]["from"] = 79.0  # reserve overlaps X2b
    split = split_v04.perception_split_from_doc(DOC, "x")
    with pytest.raises(split_v04.ReserveSegmentRefused):
        windowed.build_windowed_cache(
            tmp_path / "bagC", split, tmp_path / "c", fake_read_bag([]), ORIGIN, doc
        )


def test_missing_odometry_and_unknown_bags_are_refused(tmp_path: Path) -> None:
    split = split_v04.perception_split_from_doc(DOC, "x")

    def no_odom(bag_dir, window_s):
        d = fake_read_bag([])(bag_dir, window_s)
        d.odom = []
        return d

    with pytest.raises(windowed.WindowedReadError, match="odom"):
        windowed.build_windowed_cache(tmp_path / "bagC", split, tmp_path / "c", no_odom, ORIGIN)
    with pytest.raises(windowed.WindowedReadError):
        windowed.build_windowed_cache(tmp_path / "other", split, tmp_path / "c", no_odom, ORIGIN)


def test_scoring_requires_the_windowed_cache(tmp_path: Path) -> None:
    with pytest.raises(windowed.WindowedReadError, match="whole-bag"):
        windowed.require_cache(tmp_path, "bagC")
    (tmp_path / "bagC.scans.npz").write_bytes(b"x")
    assert windowed.require_cache(tmp_path, "bagC").name == "bagC.scans.npz"


def test_the_cache_is_what_load_scanset_reads(tmp_path: Path) -> None:
    from racer_pnp_perception import selflabel as sl

    split = split_v04.perception_split_from_doc(DOC, "x")
    windowed.build_windowed_cache(
        tmp_path / "bagC", split, tmp_path / "cache", fake_read_bag([]), ORIGIN, DOC
    )
    scans, speed_t, _speed = sl.load_scanset(tmp_path / "bagC", tmp_path / "cache")
    assert len(scans) > 0 and len(speed_t) == 2
    assert scans.bag_time_s().min() >= 52.0
