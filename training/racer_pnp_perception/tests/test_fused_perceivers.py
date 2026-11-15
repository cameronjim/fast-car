"""L1: the evaluation tools run a `fused-v1` bundle as fused-v1 (and never as a plain net), and the
GP1c perceivers are the four backends of one set of weights."""

from __future__ import annotations

import shutil
from pathlib import Path

import yaml
from racer_pnp_perception import gp1c
from racer_pnp_perception.eval_bags import build_perceivers
from racer_track_perception import fuse
from racer_track_perception.core import BACKEND_GEOMETRIC, BACKEND_NET
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle

RELEASE = (
    Path(__file__).resolve().parents[3]
    / "ros_ws"
    / "src"
    / "racer_track_perception"
    / "test"
    / "fixtures"
    / "bundle_release"
)


def fused_copy(tmp_path: Path) -> Path:
    target = tmp_path / "fused"
    shutil.copytree(RELEASE, target)
    m = yaml.safe_load((target / "manifest.yaml").read_text())
    m["io"]["arch"] = {"type": "circ_conv1d_v0_fused_v1", "fuse": fuse.FuseConfig().as_dict()}
    (target / "manifest.yaml").write_text(yaml.safe_dump(m, sort_keys=False))
    return target


def test_build_perceivers_fuses_a_fused_bundle(tmp_path: Path, live: LiveVehicle) -> None:
    p = build_perceivers(live, fused_copy(tmp_path))
    assert p[BACKEND_NET].fuse_cfg == fuse.FuseConfig()
    assert p[BACKEND_NET].refine_net is True
    assert build_perceivers(live, RELEASE)[BACKEND_NET].fuse_cfg is None


def test_gp1c_perceivers_are_four_backends_of_one_net(live: LiveVehicle) -> None:
    net = CircConv1dV0.from_npz(RELEASE / "weights.npz")
    p = gp1c.perceivers_from_net(net, live, "m", fuse.FuseConfig())
    assert set(p) == {BACKEND_GEOMETRIC, gp1c.FUSED, BACKEND_NET, gp1c.UNREFINED}
    assert p[gp1c.FUSED].fuse_cfg is not None and p[gp1c.FUSED].refine_net
    assert p[BACKEND_NET].fuse_cfg is None and p[BACKEND_NET].refine_net
    assert not p[gp1c.UNREFINED].refine_net and p[gp1c.UNREFINED].fuse_cfg is None
