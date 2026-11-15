"""L1/L4: the fused corridor polygon compared with the geometric one on the OUTPUT arrays (outside
review of brief 38: coverage is measured, not argued from the construction)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest
from racer_pnp_perception import corridor
from racer_track_perception import core, fuse
from racer_track_perception import track as tr
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle

TESTS = Path(__file__).resolve().parents[3] / "ros_ws" / "src" / "racer_track_perception" / "tests"
sys.path.insert(0, str(TESTS))
from make_fused_golden import RELEASE_WEIGHTS, SCANS_1111, SCANS_1113, dequantise


def straight(n=12, y=0.0, hl=0.5, hr=0.5) -> tr.LocalTrackData:
    t = tr.LocalTrackData(valid_count=n)
    t.x_m[:] = 0.2 * np.arange(16)
    t.y_m[:] = y
    t.half_width_left_m[:] = hl
    t.half_width_right_m[:] = hr
    t.sigma_m[:n] = 0.05
    t.sigma_m[n:] = tr.INVALID_SIGMA_M
    return t


def test_the_polygon_area_of_a_straight_corridor() -> None:
    r = corridor.compare(straight(), straight())
    assert r["geo_area_m2"] == pytest.approx(2.2 * 1.0, rel=0.02)
    assert r["loss"] == pytest.approx(0.0, abs=1e-6) and r["excess"] == pytest.approx(0.0, abs=1e-6)


def test_a_uniform_shift_inside_the_corridor_loses_nothing_on_a_straight() -> None:
    geo = straight()
    fused = fuse.fuse_track(geo, straight(y=0.1), fuse.FuseConfig(append_far=False))
    r = corridor.compare(geo, fused)
    assert r["loss"] < 0.005 and r["excess"] < 0.005


def test_the_comparison_sees_a_corridor_that_is_really_smaller() -> None:
    r = corridor.compare(straight(), straight(hl=0.25, hr=0.25))
    assert r["loss"] == pytest.approx(0.5, abs=0.02)
    r = corridor.compare(straight(hl=0.25, hr=0.25), straight())
    assert r["excess"] == pytest.approx(1.0, abs=0.05)


def test_the_comparison_sees_a_zigzag_of_centres() -> None:
    """Edges recomputed from the output's own normals move when neighbouring centres jump."""
    geo = straight(hl=0.45, hr=0.45)
    net = straight()
    net.y_m[:] = np.where(np.arange(16) % 2 == 0, 0.12, -0.12)
    fused = fuse.fuse_track(geo, net, fuse.FuseConfig(append_far=False))
    assert corridor.compare(geo, fused)["loss"] > 0.02


def test_no_polygon_gives_nan_and_extra_stations_are_counted() -> None:
    assert math.isnan(corridor.compare(tr.no_track(), straight())["loss"])
    r = corridor.compare(straight(9), fuse.fuse_track(straight(9), straight(16)))
    assert r["extra"] == 7 and r["loss"] < 1e-3


def _real_results(live: LiveVehicle, blend: float) -> list[dict]:
    net = CircConv1dV0.from_npz(RELEASE_WEIGHTS)
    geo_p = core.TrackPerceiver(core.BACKEND_GEOMETRIC, live)
    fus_p = core.TrackPerceiver(
        core.BACKEND_NET, live, net=net, model_id="r", fuse_cfg=fuse.FuseConfig(blend=blend)
    )
    out = []
    for path, quantised in ((SCANS_1111, False), (SCANS_1113, True)):
        with np.load(path) as d:
            ranges = dequantise(d["ranges_q"]) if quantised else d["ranges"]
            meta = (
                float(d["angle_min"]),
                float(d["angle_increment"]),
                float(d["range_min"]),
                float(d["range_max"]),
            )
        out += [corridor.compare(geo_p.process(r, *meta), fus_p.process(r, *meta)) for r in ranges]
    return out


def test_with_blend_zero_the_output_polygon_is_the_tracers_on_real_scans(
    live: LiveVehicle,
) -> None:
    """The measurement itself is sound: a fusion that moves nothing loses and adds nothing."""
    s = corridor.summarise(_real_results(live, 0.0))
    assert s["loss"]["max"] == 0.0 and s["excess"]["max"] == 0.0


@pytest.mark.xfail(
    strict=True,
    reason=(
        "MEASURED, OPEN: on the committed real scans the fused output polygon loses 4 to 8 percent "
        "of the tracer's area on average (up to about 30 percent on one scan) because a consumer "
        "derives edges from the fused polyline's own normals, which rotate when neighbouring "
        "centres move by different amounts. The coverage by construction does not hold on the "
        "output polygons. Owner decision pending (shift smoothing)."
    ),
)
def test_fused_coverage_loss_on_real_scans_is_under_one_percent(live: LiveVehicle) -> None:
    s = corridor.summarise(_real_results(live, 1.0))
    assert s["loss"]["p99"] < 0.01
