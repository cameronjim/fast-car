"""L2 (hypothesis): whatever the scan holds, the tensor is finite and in range, the node core
returns a LocalTrackData with 16-long arrays, and the geometric tracer never raises."""

from __future__ import annotations

import numpy as np
from bundle_fixture import FIXTURE_DIR
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from racer_track_perception import core, geometric, preprocess
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle, load_live_vehicle

LIVE = load_live_vehicle()
NET = CircConv1dV0.from_npz(FIXTURE_DIR / "weights.npz")
PERCEIVERS = [
    core.TrackPerceiver(core.BACKEND_GEOMETRIC, LIVE),
    core.TrackPerceiver(core.BACKEND_NET, LIVE, net=NET, model_id="t"),
]

ANY_FLOAT = st.floats(allow_nan=True, allow_infinity=True, width=32)
RANGES = st.one_of(
    st.lists(ANY_FLOAT, min_size=0, max_size=2000),
    st.lists(st.floats(min_value=-1.0, max_value=15.0, width=32), min_size=0, max_size=2000),
)
ANGLE = st.floats(allow_nan=True, allow_infinity=True, width=32)
SETTINGS = settings(
    max_examples=60,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large],
)


@SETTINGS
@given(ranges=RANGES, a0=ANGLE, da=ANGLE, rmin=ANGLE, rmax=ANGLE)
def test_tensor_is_always_finite_and_in_range(ranges, a0, da, rmin, rmax) -> None:
    t = preprocess.scan_to_tensor(ranges, a0, da, rmin, rmax, LIVE.mount, LIVE.body)
    assert t.shape == (1, 2, 360) and t.dtype == np.float32
    assert np.isfinite(t).all() and t.min() >= 0.0 and t.max() <= 1.0
    p = preprocess.scan_to_points(ranges, a0, da, rmin, rmax, LIVE.mount, LIVE.body)
    assert p.ndim == 2 and p.shape[1] == 2 and np.isfinite(p).all()


@SETTINGS
@given(
    ranges=RANGES,
    a0=st.floats(min_value=-7.0, max_value=7.0),
    da=st.floats(min_value=1e-4, max_value=0.1),
)
def test_core_always_returns_a_16_station_track(ranges, a0, da) -> None:
    for p in PERCEIVERS:
        t = p.process(ranges, a0, da, 0.05, 12.0)
        for a in (t.x_m, t.y_m, t.half_width_left_m, t.half_width_right_m, t.sigma_m):
            assert len(a) == 16 and np.isfinite(a).all()
        assert 0 <= t.valid_count <= 16 and t.arrays_ok()


@SETTINGS
@given(
    pts=st.lists(
        st.tuples(ANY_FLOAT, ANY_FLOAT),
        min_size=0,
        max_size=400,
    )
)
def test_geometric_tracer_never_raises(pts) -> None:
    arr = np.array(pts, dtype=np.float32).reshape(-1, 2)
    t = geometric.trace(arr, LIVE.body.half_width_m, 0.25)
    assert len(t.x_m) == 16 and t.arrays_ok()


def test_hypothesis_fixture_is_the_live_vehicle(live: LiveVehicle) -> None:
    assert live == LIVE
