"""L1: the bundle contract of `fused-v1` (`io.arch.type` circ_conv1d_v0_fused_v1) and its use in
the core. Plain and `refine_v1` bundles must behave exactly as they did before this arch type."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pytest
from bundle_fixture import write_bundle
from racer_track_perception import bundle as bd
from racer_track_perception import core, fuse, geometric, preprocess, refine, selfcheck
from racer_track_perception import errors as er
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle
from scan_fixture import SCAN, corridor_ranges
from test_refine_bundle import constant_net, refine_arch

KIND = "track_perception"
FIELDS = ("x_m", "y_m", "half_width_left_m", "half_width_right_m", "sigma_m")


def fused_arch(m: dict[str, Any], cfg: dict[str, Any] | None = None) -> None:
    m["io"]["arch"]["type"] = bd.ARCH_TYPE_FUSED_V1
    m["io"]["arch"]["fuse"] = cfg if cfg is not None else fuse.FuseConfig().as_dict()


def perceiver(path: Path, live: LiveVehicle) -> core.TrackPerceiver:
    b = bd.load_bundle(path, KIND, live)
    net = CircConv1dV0.from_npz(b.payload_path("weights.npz"))
    return core.TrackPerceiver(
        core.BACKEND_NET,
        live,
        net=net,
        model_id=b.model_id,
        refine_net=b.refine,
        fuse_cfg=b.fuse_config,
    )


def test_exactly_three_arch_types_are_accepted() -> None:
    assert bd.ARCH_TYPES == (
        "circ_conv1d_v0",
        "circ_conv1d_v0_refine_v1",
        "circ_conv1d_v0_fused_v1",
    )


def test_fused_bundle_loads_and_carries_its_config(tmp_path: Path, live: LiveVehicle) -> None:
    b = bd.load_bundle(write_bundle(tmp_path / "b", live, fused_arch), KIND, live)
    assert b.manifest["io"]["arch"]["type"] == "circ_conv1d_v0_fused_v1"
    assert b.fuse_config == fuse.FuseConfig()
    assert b.refine is False  # the property of the refine_v1 bundle; fusion implies refinement


def test_the_frozen_config_is_read_from_the_manifest(tmp_path: Path, live: LiveVehicle) -> None:
    cfg = fuse.FuseConfig(reach_m=0.3, edge_margin_m=0.1, blend=0.9, append_far=False)

    def mutate(m: dict[str, Any]) -> None:
        fused_arch(m, cfg.as_dict())

    b = bd.load_bundle(write_bundle(tmp_path / "b", live, mutate), KIND, live)
    assert b.fuse_config == cfg


def test_plain_and_refine_bundles_have_no_fuse_config(tmp_path: Path, live: LiveVehicle) -> None:
    assert bd.load_bundle(write_bundle(tmp_path / "a", live), KIND, live).fuse_config is None
    b = bd.load_bundle(write_bundle(tmp_path / "b", live, refine_arch), KIND, live)
    assert b.fuse_config is None and b.refine is True


@pytest.mark.parametrize(
    "arch",
    ["circ_conv1d_v0_fused_v2", "circ_conv1d_v0_fused", "fused-v1", "circ_conv1d_v0_fuse_v1"],
)
def test_an_unknown_arch_type_is_refused(tmp_path: Path, live: LiveVehicle, arch: str) -> None:
    def change(m: dict[str, Any]) -> None:
        fused_arch(m)
        m["io"]["arch"]["type"] = arch

    with pytest.raises(er.IoContractMismatch, match="arch"):
        bd.load_bundle(write_bundle(tmp_path / "b", live, change), KIND, live)


def test_a_loader_that_does_not_know_the_type_refuses_it(
    tmp_path: Path, live: LiveVehicle, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The old loader: the same bundle with the type removed from what the code implements."""
    path = write_bundle(tmp_path / "b", live, fused_arch)
    monkeypatch.setattr(bd, "ARCH_TYPES", (bd.ARCH_TYPE, bd.ARCH_TYPE_REFINE_V1))
    with pytest.raises(er.IoContractMismatch, match="arch"):
        bd.load_bundle(path, KIND, live)


@pytest.mark.parametrize(
    "cfg",
    [
        None,
        {},
        {"reach_m": 0.35, "edge_margin_m": 0.15, "blend": 1.0},  # a key missing
        {**fuse.FuseConfig().as_dict(), "extra": 1},
        {**fuse.FuseConfig().as_dict(), "blend": 2.0},
        {**fuse.FuseConfig().as_dict(), "reach_m": "0.35"},
        {**fuse.FuseConfig().as_dict(), "append_far": "yes"},
    ],
)
def test_a_fused_bundle_without_a_usable_config_is_refused(
    tmp_path: Path, live: LiveVehicle, cfg: dict[str, Any] | None
) -> None:
    def mutate(m: dict[str, Any]) -> None:
        m["io"]["arch"]["type"] = bd.ARCH_TYPE_FUSED_V1
        if cfg is not None:
            m["io"]["arch"]["fuse"] = cfg

    with pytest.raises(er.IoContractMismatch, match="fuse"):
        bd.load_bundle(write_bundle(tmp_path / "b", live, mutate), KIND, live)


def _main_path(p: core.TrackPerceiver, live: LiveVehicle, refined: bool) -> Any:
    """What main did for a net backend: preprocess, forward, refine or not, self-check."""
    ranges = corridor_ranges(live)
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
    track = core.net_output_to_track(p.net.forward(tensor), p.sigma_max_m)
    if refined:
        track = refine.refine_track(track, points, refine.RefineConfig(sigma_max_m=0.25))
    selfcheck.apply_self_check(track, points)
    return track


@pytest.mark.parametrize("variant", ["plain", "refine"])
def test_plain_and_refine_bundles_are_bit_identical_to_main(
    tmp_path: Path, live: LiveVehicle, variant: str
) -> None:
    weights = constant_net(tmp_path)
    mutate = refine_arch if variant == "refine" else None
    p = perceiver(write_bundle(tmp_path / "b", live, mutate, weights=weights), live)
    assert p.fuse_cfg is None
    got = p.process(corridor_ranges(live), **SCAN)
    want = _main_path(p, live, refined=variant == "refine")
    for name in FIELDS:
        assert np.array_equal(getattr(got, name), getattr(want, name)), name
    assert got.valid_count == want.valid_count
    assert np.array_equal(got.intrusion_fraction, want.intrusion_fraction, equal_nan=True)
    assert np.array_equal(got.boundary_residual_m, want.boundary_residual_m, equal_nan=True)
    assert p.fuse_fallbacks == 0


def test_fused_bundle_runs_tracer_net_refinement_and_fusion(
    tmp_path: Path, live: LiveVehicle
) -> None:
    """The constant net is 0.2 m off the lane: the fused track takes the net's centre inside the
    tracer's corridor, and the corridor edges are the tracer's."""
    weights = constant_net(tmp_path)
    p = perceiver(write_bundle(tmp_path / "b", live, fused_arch, weights=weights), live)
    assert p.fuse_cfg == fuse.FuseConfig() and p.refine_net is True
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
    net_track = refine.refine_track(
        core.net_output_to_track(p.net.forward(tensor), p.sigma_max_m),
        points,
        refine.RefineConfig(sigma_max_m=0.25),
    )
    geo = geometric.trace(points, live.body.half_width_m, p.sigma_max_m, p._geo_cfg)
    want = fuse.fuse_track(geo, net_track, fuse.FuseConfig())
    selfcheck.apply_self_check(want, points)
    for name in FIELDS:
        assert np.array_equal(getattr(got, name), getattr(want, name)), name
    assert got.valid_count == want.valid_count >= geo.valid_count
    n = geo.valid_count
    assert (
        np.abs(
            got.y_m[:n] + got.half_width_left_m[:n] - (geo.y_m[:n] + geo.half_width_left_m[:n])
        ).max()
        < 1e-4
    )
    assert p.fuse_fallbacks == 0


def test_a_net_that_gives_no_track_falls_back_to_the_tracer_and_is_counted(
    tmp_path: Path, live: LiveVehicle
) -> None:
    w = np.load(constant_net(tmp_path))
    bad = {k: w[k] for k in w.files}
    bias = bad["linear1_bias"].reshape(16, 5).copy()
    bias[:, 4] = np.nan  # no usable sigma anywhere: the net says "no track"
    bad["linear1_bias"] = bias.reshape(-1)
    path = tmp_path / "bad.npz"
    np.savez(path, **bad)
    p = perceiver(write_bundle(tmp_path / "b", live, fused_arch, weights=path), live)
    got = p.process(corridor_ranges(live), **SCAN)
    assert got.valid_count > 0
    assert p.fuse_fallbacks == 1
    assert p.process(corridor_ranges(live), **SCAN).valid_count == got.valid_count
    assert p.fuse_fallbacks == 2


def test_geometric_backend_never_fuses(live: LiveVehicle) -> None:
    p = core.TrackPerceiver(core.BACKEND_GEOMETRIC, live, fuse_cfg=fuse.FuseConfig())
    assert p.fuse_cfg is None and p.refine_net is False
