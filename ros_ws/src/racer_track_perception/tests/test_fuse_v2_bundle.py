"""L1: the bundle contract of `fused-v2` (`io.arch.type` circ_conv1d_v0_fused_v2) and its use in
the core. The four older paths (plain, refine_v1, fused_v1, geometric) behave as they did on main."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pytest
from bundle_fixture import write_bundle
from racer_track_perception import bundle as bd
from racer_track_perception import core, fuse, fuse_v2, geometric, preprocess, refine, selfcheck
from racer_track_perception import errors as er
from racer_track_perception.vehicle import LiveVehicle
from scan_fixture import SCAN, corridor_ranges
from test_fuse_bundle import FIELDS, KIND, fused_arch, perceiver
from test_refine_bundle import constant_net, refine_arch


def fused_v2_arch(m: dict[str, Any], cfg: dict[str, Any] | None = None) -> None:
    m["io"]["arch"]["type"] = bd.ARCH_TYPE_FUSED_V2
    m["io"]["arch"]["fuse"] = (
        cfg if cfg is not None else fuse_v2.FuseV2Config(edge_end_extension_m=0.1).as_dict()
    )


def test_the_new_arch_type_is_accepted_and_carries_its_config(
    tmp_path: Path, live: LiveVehicle
) -> None:
    b = bd.load_bundle(write_bundle(tmp_path / "b", live, fused_v2_arch), KIND, live)
    assert b.manifest["io"]["arch"]["type"] == "circ_conv1d_v0_fused_v2"
    assert b.fuse_config == fuse_v2.FuseV2Config(edge_end_extension_m=0.1)
    assert isinstance(b.fuse_config, fuse.FuseConfig)  # so a node that passes it on needs no edit
    assert b.refine is False


def test_an_unknown_arch_type_is_still_refused(tmp_path: Path, live: LiveVehicle) -> None:
    def change(m: dict[str, Any]) -> None:
        fused_v2_arch(m)
        m["io"]["arch"]["type"] = "circ_conv1d_v0_fused_v3"

    with pytest.raises(er.IoContractMismatch, match="arch"):
        bd.load_bundle(write_bundle(tmp_path / "b", live, change), KIND, live)


def test_a_loader_that_does_not_know_the_type_refuses_it(
    tmp_path: Path, live: LiveVehicle, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = write_bundle(tmp_path / "b", live, fused_v2_arch)
    monkeypatch.setattr(bd, "ARCH_TYPES", bd.ARCH_TYPES[:3])
    with pytest.raises(er.IoContractMismatch, match="arch"):
        bd.load_bundle(path, KIND, live)


@pytest.mark.parametrize(
    "cfg",
    [
        None,
        {},
        fuse.FuseConfig().as_dict(),  # a v1 config: the extension key is missing
        {**fuse_v2.FuseV2Config().as_dict(), "extra": 1},
        {**fuse_v2.FuseV2Config().as_dict(), "edge_end_extension_m": 0.5},
        {**fuse_v2.FuseV2Config().as_dict(), "edge_end_extension_m": "0.1"},
        {**fuse_v2.FuseV2Config().as_dict(), "blend": 2.0},
    ],
)
def test_a_v2_bundle_without_a_usable_config_is_refused(
    tmp_path: Path, live: LiveVehicle, cfg: dict[str, Any] | None
) -> None:
    def mutate(m: dict[str, Any]) -> None:
        m["io"]["arch"]["type"] = bd.ARCH_TYPE_FUSED_V2
        if cfg is not None:
            m["io"]["arch"]["fuse"] = cfg

    with pytest.raises(er.IoContractMismatch, match="fuse"):
        bd.load_bundle(write_bundle(tmp_path / "b", live, mutate), KIND, live)


def test_a_v1_bundle_with_a_v2_config_is_refused(tmp_path: Path, live: LiveVehicle) -> None:
    def mutate(m: dict[str, Any]) -> None:
        fused_arch(m, fuse_v2.FuseV2Config().as_dict())

    with pytest.raises(er.IoContractMismatch, match="fuse"):
        bd.load_bundle(write_bundle(tmp_path / "b", live, mutate), KIND, live)


def _main_path(p: core.TrackPerceiver, live: LiveVehicle, refined: bool) -> Any:
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
def test_plain_and_refine_paths_are_bit_identical_to_main(
    tmp_path: Path, live: LiveVehicle, variant: str
) -> None:
    weights = constant_net(tmp_path)
    mutate = refine_arch if variant == "refine" else None
    p = perceiver(write_bundle(tmp_path / "b", live, mutate, weights=weights), live)
    got = p.process(corridor_ranges(live), **SCAN)
    want = _main_path(p, live, refined=variant == "refine")
    for name in FIELDS:
        assert np.array_equal(getattr(got, name), getattr(want, name)), name
    assert got.valid_count == want.valid_count


def test_the_v1_path_is_bit_identical_to_main(tmp_path: Path, live: LiveVehicle) -> None:
    weights = constant_net(tmp_path)
    p = perceiver(write_bundle(tmp_path / "b", live, fused_arch, weights=weights), live)
    assert type(p.fuse_cfg) is fuse.FuseConfig
    got = p.process(corridor_ranges(live), **SCAN)
    tensor, points = preprocess.scan_to_tensor_and_points(
        corridor_ranges(live),
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


def test_the_geometric_path_is_untouched(live: LiveVehicle) -> None:
    p = core.TrackPerceiver(core.BACKEND_GEOMETRIC, live, fuse_cfg=fuse_v2.FuseV2Config())
    assert p.fuse_cfg is None and p.refine_net is False


def test_a_v2_bundle_runs_tracer_net_refinement_and_fusion_v2(
    tmp_path: Path, live: LiveVehicle
) -> None:
    weights = constant_net(tmp_path)
    p = perceiver(write_bundle(tmp_path / "b", live, fused_v2_arch, weights=weights), live)
    cfg = fuse_v2.FuseV2Config(edge_end_extension_m=0.1)
    assert p.fuse_cfg == cfg and p.refine_net is True
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
    want = fuse_v2.fuse_track_v2(geo, net_track, cfg)
    selfcheck.apply_self_check(want, points)
    for name in FIELDS:
        assert np.array_equal(getattr(got, name), getattr(want, name)), name
    assert got.valid_count > 0 and p.fuse_fallbacks == 0


def test_a_net_that_gives_no_track_falls_back_to_the_tracer_and_is_counted(
    tmp_path: Path, live: LiveVehicle
) -> None:
    w = np.load(constant_net(tmp_path))
    bad = {k: w[k] for k in w.files}
    bias = bad["linear1_bias"].reshape(16, 5).copy()
    bias[:, 4] = np.nan
    bad["linear1_bias"] = bias.reshape(-1)
    path = tmp_path / "bad.npz"
    np.savez(path, **bad)
    p = perceiver(write_bundle(tmp_path / "b", live, fused_v2_arch, weights=path), live)
    got = p.process(corridor_ranges(live), **SCAN)
    assert got.valid_count > 0 and p.fuse_fallbacks == 1
