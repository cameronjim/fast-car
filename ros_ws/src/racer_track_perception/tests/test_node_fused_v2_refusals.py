"""L3: startup refusal of a `fused-v2` bundle without a usable frozen config (rclpy; colcon only)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("racer_pnp_msgs")
pytest.importorskip("racer_tools.vehicle_params_loader")

from bundle_fixture import write_bundle
from racer_track_perception.bundle import ARCH_TYPE_FUSED_V2
from racer_track_perception.fuse import FuseConfig
from racer_track_perception.vehicle import load_live_vehicle
from test_node_refusals import refused, run_node


@pytest.mark.parametrize("cfg", [None, FuseConfig().as_dict()])
def test_a_v2_bundle_without_its_config_is_refused(tmp_path: Path, cfg) -> None:
    def mutate(m: dict[str, Any]) -> None:
        m["io"]["arch"]["type"] = ARCH_TYPE_FUSED_V2
        if cfg is not None:
            m["io"]["arch"]["fuse"] = cfg

    b = write_bundle(tmp_path / "b", load_live_vehicle(), mutate)
    refused(run_node("backend:=net", f"bundle_dir:={b}"), "IoContractMismatch")
