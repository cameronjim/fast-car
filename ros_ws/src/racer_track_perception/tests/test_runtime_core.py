from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from bundle_fixture import FIXTURE_DIR, tiny_weights
from racer_track_perception import core
from racer_track_perception import track as tr
from racer_track_perception.runtime import CircConv1dV0, ModelFormatError
from racer_track_perception.vehicle import LiveVehicle
from scan_fixture import SCAN, corridor_ranges


def reference_forward(w: dict[str, np.ndarray], x: np.ndarray) -> np.ndarray:
    """A direct (loop) implementation of the same network, independent of the im2col code."""
    h = x[0].astype(np.float64)
    for i in range(int(w["n_conv"])):
        weight = w[f"conv{i}_weight"].astype(np.float64)
        bias = w[f"conv{i}_bias"].astype(np.float64)
        stride, pad = int(w[f"conv{i}_stride"]), int(w[f"conv{i}_pad"])
        c_out, _c_in, k = weight.shape
        padded = np.concatenate([h[:, -pad:], h, h[:, :pad]], axis=1)
        n_out = (h.shape[1] + 2 * pad - k) // stride + 1
        out = np.zeros((c_out, n_out))
        for o in range(c_out):
            for t in range(n_out):
                window = padded[:, t * stride : t * stride + k]
                out[o, t] = np.sum(weight[o] * window) + bias[o]
        h = np.maximum(out, 0.0)
    v = h.reshape(-1)
    n_lin = int(w["n_linear"])
    for i in range(n_lin):
        v = w[f"linear{i}_weight"].astype(np.float64) @ v + w[f"linear{i}_bias"]
        if i < n_lin - 1:
            v = np.maximum(v, 0.0)
    return v.reshape(16, 5)


def test_forward_matches_a_loop_reference() -> None:
    w = tiny_weights(3)
    net = CircConv1dV0(w)
    rng = np.random.default_rng(0)
    for _ in range(5):
        x = rng.uniform(0.0, 1.0, size=(1, 2, 360)).astype(np.float32)
        out = net.forward(x)
        assert out.shape == (16, 5) and out.dtype == np.float32
        assert np.abs(out - reference_forward(w, x)).max() < 1e-4


def test_circular_padding_wraps_the_scan() -> None:
    w = tiny_weights(1)
    net = CircConv1dV0(w)
    rng = np.random.default_rng(1)
    x = rng.uniform(size=(1, 2, 360)).astype(np.float32)
    a = net.forward(x)
    # The first conv layer sees bins 358, 359 as the left neighbours of bin 0: changing bin 359
    # must change the output through the wrap, as it would through any interior bin.
    y = x.copy()
    y[0, 0, 359] += 0.5
    assert np.abs(net.forward(y) - a).max() > 0.0


def test_from_npz_roundtrip(tmp_path: Path) -> None:
    path = tmp_path / "w.npz"
    np.savez(path, **tiny_weights(2))
    net = CircConv1dV0.from_npz(path)
    x = np.random.default_rng(0).uniform(size=(1, 2, 360)).astype(np.float32)
    assert np.array_equal(net.forward(x), CircConv1dV0(tiny_weights(2)).forward(x))
    # The committed fixture loads too.
    CircConv1dV0.from_npz(FIXTURE_DIR / "weights.npz")


def test_forward_rejects_a_wrong_input_shape() -> None:
    with pytest.raises(ValueError, match="input must be"):
        CircConv1dV0(tiny_weights()).forward(np.zeros((1, 2, 359), dtype=np.float32))


def test_format_errors(tmp_path: Path) -> None:
    w = tiny_weights()
    missing = dict(w)
    del missing["conv1_bias"]
    with pytest.raises(ModelFormatError, match="conv1_bias"):
        CircConv1dV0(missing)
    bad_in = dict(w)
    bad_in["conv0_weight"] = np.zeros((4, 3, 5), dtype=np.float32)  # 3 input channels
    with pytest.raises(ModelFormatError):
        CircConv1dV0(bad_in)
    bad_out = dict(w)
    bad_out["linear1_weight"] = np.zeros((40, 16), dtype=np.float32)
    bad_out["linear1_bias"] = np.zeros(40, dtype=np.float32)
    with pytest.raises(ModelFormatError, match="80"):
        CircConv1dV0(bad_out)
    bad_lin = dict(w)
    bad_lin["linear0_weight"] = np.zeros((16, 7), dtype=np.float32)
    with pytest.raises(ModelFormatError):
        CircConv1dV0(bad_lin)
    bad_bias = dict(w)
    bad_bias["conv2_bias"] = np.zeros(3, dtype=np.float32)
    with pytest.raises(ModelFormatError):
        CircConv1dV0(bad_bias)
    none = dict(w)
    none["n_conv"] = np.array(0)
    with pytest.raises(ModelFormatError):
        CircConv1dV0(none)
    with pytest.raises(ModelFormatError, match="cannot read"):
        CircConv1dV0.from_npz(tmp_path / "nope.npz")
    (tmp_path / "junk.npz").write_bytes(b"not an npz")
    with pytest.raises(ModelFormatError):
        CircConv1dV0.from_npz(tmp_path / "junk.npz")
    part = tmp_path / "part.npz"
    np.savez(part, n_conv=np.array(1))
    with pytest.raises(ModelFormatError):
        CircConv1dV0.from_npz(part)


# -- core ---------------------------------------------------------------------------------------


def test_geometric_perceiver_on_a_synthetic_lane(live: LiveVehicle) -> None:
    p = core.TrackPerceiver(core.BACKEND_GEOMETRIC, live)
    assert p.model_id == "geometric-v0"
    t = p.process(corridor_ranges(live), **SCAN)
    assert t.valid_count == 16 and t.arrays_ok()
    assert np.abs(t.y_m).max() < 0.05
    assert t.boundary_residual_m < 0.05 and t.intrusion_fraction < 0.05


def test_net_perceiver_runs_and_gives_a_valid_message(live: LiveVehicle) -> None:
    net = CircConv1dV0.from_npz(FIXTURE_DIR / "weights.npz")
    p = core.TrackPerceiver(core.BACKEND_NET, live, net=net, model_id="trackperc-v0-test")
    assert p.model_id == "trackperc-v0-test"
    t = p.process(corridor_ranges(live), **SCAN)
    assert t.arrays_ok() and len(t.x_m) == 16
    assert t.valid_count == 16  # the fixture bias gives sigma 0.05 everywhere


@pytest.mark.parametrize("backend", [core.BACKEND_GEOMETRIC, core.BACKEND_NET])
def test_bad_scans_give_no_track_not_an_exception(live: LiveVehicle, backend: str) -> None:
    net = CircConv1dV0.from_npz(FIXTURE_DIR / "weights.npz")
    p = core.TrackPerceiver(backend, live, net=net if backend == "net" else None, model_id="x")
    for ranges in ([], [math.nan] * 720, [math.inf] * 720, None, "junk", [1.0, 2.0]):
        t = p.process(ranges, **SCAN)
        assert len(t.x_m) == 16 and t.arrays_ok()
    empty = p.process([math.nan] * 720, **SCAN)
    if backend == "geometric":
        assert empty.valid_count == 0


def test_perceiver_constructor_checks(live: LiveVehicle) -> None:
    with pytest.raises(ValueError, match="backend"):
        core.TrackPerceiver("magic", live)
    with pytest.raises(ValueError, match="net"):
        core.TrackPerceiver(core.BACKEND_NET, live)


def test_process_survives_a_backend_failure(live: LiveVehicle) -> None:
    class Boom:
        def forward(self, _t: np.ndarray) -> np.ndarray:
            raise RuntimeError("boom")

    p = core.TrackPerceiver(core.BACKEND_NET, live, net=Boom())  # type: ignore[arg-type]
    t = p.process(corridor_ranges(live), **SCAN)
    assert t.valid_count == 0 and t.arrays_ok()


def test_net_output_to_track_rules() -> None:
    out = np.zeros((16, 5), dtype=np.float32)
    out[:, 0] = 0.2 * np.arange(16)
    out[:, 2:4] = 0.5
    out[:, 4] = math.log(0.1)
    assert core.net_output_to_track(out, 0.25).valid_count == 16
    out[6:, 4] = math.log(0.5)  # too uncertain from station 6 on
    t = core.net_output_to_track(out, 0.25)
    assert t.valid_count == 6 and np.isfinite(t.x_m).all()
    out[3, 0] = math.nan
    assert core.net_output_to_track(out, 0.25).valid_count == 3
    out[:, 4] = math.nan
    assert core.net_output_to_track(out, 0.25).valid_count == 0
    assert core.net_output_to_track(np.zeros((4, 5)), 0.25).valid_count == 0
    out = np.zeros((16, 5), dtype=np.float32)
    out[:, 4] = 100.0  # absurd log sigma is clamped, then too uncertain
    assert core.net_output_to_track(out, 0.25).valid_count == 0
    out[:, 4] = -100.0  # clamped to a small, valid sigma
    t = core.net_output_to_track(out, 0.25)
    assert t.valid_count == 16 and abs(t.sigma_m[0] - math.exp(-7.0)) < 1e-6
    assert isinstance(t, tr.LocalTrackData)
