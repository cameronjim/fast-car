"""L1: the bundle contract of the scan refinement (`io.arch.type`) and its use in the core."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pytest
from bundle_fixture import tiny_weights, write_bundle
from racer_track_perception import bundle as bd
from racer_track_perception import core, preprocess, refine, selfcheck
from racer_track_perception import errors as er
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle
from scan_fixture import SCAN, corridor_ranges

KIND = "track_perception"


def refine_arch(m: dict[str, Any]) -> None:
    m["io"]["arch"]["type"] = bd.ARCH_TYPE_REFINE_V1


def constant_net(directory: Path, y_offset: float = 0.2) -> Path:
    """Weights of a net that ignores the scan and always answers a straight corridor 0.2 m off:
    stations 0.2 m apart, half widths 0.5 m, sigma 0.05 m."""
    w = tiny_weights()
    for key in list(w):
        if key.endswith("weight"):
            w[key] = np.zeros_like(w[key])
    bias = np.zeros((16, 5), dtype=np.float32)
    bias[:, 0] = 0.2 * np.arange(16)
    bias[:, 1] = y_offset
    bias[:, 2:4] = 0.5
    bias[:, 4] = np.log(0.05)
    w["linear1_bias"] = bias.reshape(-1)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "constant.npz"
    np.savez(path, **w)
    return path


def perceiver(path: Path, live: LiveVehicle) -> core.TrackPerceiver:
    b = bd.load_bundle(path, KIND, live)
    net = CircConv1dV0.from_npz(b.payload_path("weights.npz"))
    return core.TrackPerceiver(
        core.BACKEND_NET, live, net=net, model_id=b.model_id, refine_net=b.refine
    )


def test_exactly_two_arch_types_are_accepted() -> None:
    assert bd.ARCH_TYPES == ("circ_conv1d_v0", "circ_conv1d_v0_refine_v1")


def test_plain_bundle_does_not_refine(tmp_path: Path, live: LiveVehicle) -> None:
    b = bd.load_bundle(write_bundle(tmp_path / "b", live), KIND, live)
    assert b.refine is False


def test_refine_bundle_loads_and_says_so(tmp_path: Path, live: LiveVehicle) -> None:
    b = bd.load_bundle(write_bundle(tmp_path / "b", live, refine_arch), KIND, live)
    assert b.refine is True
    assert b.manifest["io"]["arch"]["type"] == "circ_conv1d_v0_refine_v1"


@pytest.mark.parametrize("arch", ["circ_conv1d_v0_refine_v2", "circ_conv1d_v1", "refine"])
def test_an_unknown_arch_type_is_refused(tmp_path: Path, live: LiveVehicle, arch: str) -> None:
    def change(m: dict[str, Any]) -> None:
        m["io"]["arch"]["type"] = arch

    with pytest.raises(er.IoContractMismatch, match="arch"):
        bd.load_bundle(write_bundle(tmp_path / "b", live, change), KIND, live)


def test_plain_bundle_output_is_bit_identical_to_the_unrefined_path(
    tmp_path: Path, live: LiveVehicle
) -> None:
    """The path of main: preprocess, net forward, `net_output_to_track`, self-check."""
    p = perceiver(write_bundle(tmp_path / "b", live), live)
    ranges = corridor_ranges(live)
    got = p.process(ranges, **SCAN)
    tensor, points = preprocess.scan_to_tensor_and_points(
        ranges,
        SCAN["angle_min"],
        SCAN["angle_increment"],
        SCAN["range_min"],
        SCAN["range_max"],
        live.mount,
        live.body,
    )
    assert p.net is not None
    want = core.net_output_to_track(p.net.forward(tensor), p.sigma_max_m)
    selfcheck.apply_self_check(want, points)
    for name in ("x_m", "y_m", "half_width_left_m", "half_width_right_m", "sigma_m"):
        assert np.array_equal(getattr(got, name), getattr(want, name)), name
    assert got.valid_count == want.valid_count
    assert got.boundary_residual_m == want.boundary_residual_m
    assert got.intrusion_fraction == want.intrusion_fraction


def test_refine_bundle_applies_the_refinement_and_checks_the_refined_track(
    tmp_path: Path, live: LiveVehicle
) -> None:
    weights = constant_net(tmp_path)
    plain = perceiver(write_bundle(tmp_path / "plain", live, weights=weights), live)
    refined = perceiver(
        write_bundle(tmp_path / "refined", live, refine_arch, weights=weights), live
    )
    ranges = corridor_ranges(live)
    a = plain.process(ranges, **SCAN)
    b = refined.process(ranges, **SCAN)
    assert not np.array_equal(a.y_m, b.y_m) or not np.array_equal(
        a.half_width_left_m, b.half_width_left_m
    )
    tensor, points = preprocess.scan_to_tensor_and_points(
        ranges,
        SCAN["angle_min"],
        SCAN["angle_increment"],
        SCAN["range_min"],
        SCAN["range_max"],
        live.mount,
        live.body,
    )
    assert refined.net is not None
    net_track = core.net_output_to_track(refined.net.forward(tensor), refined.sigma_max_m)
    want = refine.refine_track(net_track, points, refine.RefineConfig(sigma_max_m=0.25))
    selfcheck.apply_self_check(want, points)
    for name in ("x_m", "y_m", "half_width_left_m", "half_width_right_m", "sigma_m"):
        assert np.array_equal(getattr(b, name), getattr(want, name)), name
    assert b.valid_count == want.valid_count
    assert np.array_equal(b.intrusion_fraction, want.intrusion_fraction, equal_nan=True)
    assert np.array_equal(b.boundary_residual_m, want.boundary_residual_m, equal_nan=True)
    # The refinement did what it is for: the net's 0.2 m error is gone.
    assert a.valid_count == b.valid_count == 16
    assert np.abs(a.y_m).max() > 0.19 and np.abs(b.y_m).max() < 0.03
    assert b.intrusion_fraction <= a.intrusion_fraction


def test_model_id_is_the_bundles_for_a_refine_bundle(tmp_path: Path, live: LiveVehicle) -> None:
    p = perceiver(write_bundle(tmp_path / "b", live, refine_arch), live)
    assert p.model_id == "trackperc-v0-2026-11-12-tiny"
    assert p.refine_net is True


def test_geometric_backend_never_refines(live: LiveVehicle) -> None:
    p = core.TrackPerceiver(core.BACKEND_GEOMETRIC, live, refine_net=True)
    assert p.refine_net is False
    assert p.model_id == core.GEOMETRIC_MODEL_ID
