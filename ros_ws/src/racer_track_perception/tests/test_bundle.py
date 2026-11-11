"""One test per refusal class of load_bundle, plus a good bundle."""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any

import pytest
from bundle_fixture import write_bundle
from racer_track_perception import bundle as bd
from racer_track_perception import errors as er
from racer_track_perception.vehicle import LiveVehicle

KIND = "track_perception"


def load(path: Path, live: LiveVehicle, kind: str = KIND) -> bd.Bundle:
    return bd.load_bundle(path, kind, live)


def test_a_good_bundle_loads(tmp_path: Path, live: LiveVehicle) -> None:
    b = load(write_bundle(tmp_path / "b", live), live)
    assert b.model_id == "trackperc-v0-2026-11-12-tiny"
    assert len(b.weights_sha256) == 64
    assert b.payload_path("weights.npz").is_file()


def test_every_refusal_is_a_bundle_refused() -> None:
    for cls in (
        er.ManifestMissing,
        er.ManifestInvalid,
        er.BundleSchemaVersionMismatch,
        er.KindMismatch,
        er.PayloadMissing,
        er.ChecksumMismatch,
        er.VehicleParamsMismatch,
        er.IoContractMismatch,
    ):
        assert issubclass(cls, er.BundleRefused)


def test_manifest_missing(tmp_path: Path, live: LiveVehicle) -> None:
    (tmp_path / "empty").mkdir()
    with pytest.raises(er.ManifestMissing):
        load(tmp_path / "empty", live)
    with pytest.raises(er.ManifestMissing):
        load(tmp_path / "does_not_exist", live)
    bad = tmp_path / "bad"
    bad.mkdir()
    (bad / "manifest.yaml").write_text("key: [unclosed\n  - : :")
    with pytest.raises(er.ManifestMissing):
        load(bad, live)


def test_manifest_invalid_missing_field(tmp_path: Path, live: LiveVehicle) -> None:
    def drop(m: dict[str, Any]) -> None:
        del m["git_sha"]

    with pytest.raises(er.ManifestInvalid, match="git_sha"):
        load(write_bundle(tmp_path / "b", live, drop), live)


def test_manifest_invalid_extra_field(tmp_path: Path, live: LiveVehicle) -> None:
    def extra(m: dict[str, Any]) -> None:
        m["surprise"] = 1

    with pytest.raises(er.ManifestInvalid):
        load(write_bundle(tmp_path / "b", live, extra), live)


def test_manifest_invalid_wrong_type(tmp_path: Path, live: LiveVehicle) -> None:
    def wrong(m: dict[str, Any]) -> None:
        m["git_sha"] = 12345

    with pytest.raises(er.ManifestInvalid):
        load(write_bundle(tmp_path / "b", live, wrong), live)


def test_manifest_invalid_not_a_mapping_and_bad_payload_name(
    tmp_path: Path, live: LiveVehicle
) -> None:
    d = tmp_path / "list"
    d.mkdir()
    (d / "manifest.yaml").write_text("- a\n- b\n")
    with pytest.raises(er.ManifestInvalid):
        load(d, live)

    def traverse(m: dict[str, Any]) -> None:
        m["payload"].append({"file": "../weights.npz", "sha256": "0" * 64})

    with pytest.raises(er.ManifestInvalid):
        load(write_bundle(tmp_path / "b", live, traverse), live)


def test_schema_unreadable_is_a_refusal(tmp_path: Path, live: LiveVehicle) -> None:
    b = write_bundle(tmp_path / "b", live)
    with pytest.raises(er.ManifestInvalid):
        bd.load_bundle(b, KIND, live, schema_path=tmp_path / "no_schema.json")
    (tmp_path / "list.json").write_text("[1]")
    with pytest.raises(er.ManifestInvalid):
        bd.load_bundle(b, KIND, live, schema_path=tmp_path / "list.json")


def test_schema_version_major(tmp_path: Path, live: LiveVehicle) -> None:
    def v2(m: dict[str, Any]) -> None:
        m["bundle_schema_version"] = "2.0.0"

    with pytest.raises(er.BundleSchemaVersionMismatch):
        load(write_bundle(tmp_path / "b", live, v2), live)

    def minor(m: dict[str, Any]) -> None:
        m["bundle_schema_version"] = "1.7.3"

    load(write_bundle(tmp_path / "ok", live, minor), live)  # a minor bump is still readable


def test_kind_mismatch(tmp_path: Path, live: LiveVehicle) -> None:
    good = write_bundle(tmp_path / "good", live)
    with pytest.raises(er.KindMismatch):
        load(good, live, kind="dynamics")

    def other(m: dict[str, Any]) -> None:
        m["kind"] = "lap_memory"

    with pytest.raises(er.KindMismatch):
        load(write_bundle(tmp_path / "lap", live, other), live)


def test_payload_missing(tmp_path: Path, live: LiveVehicle) -> None:
    b = write_bundle(tmp_path / "b", live)
    (b / "weights.npz").unlink()
    with pytest.raises(er.PayloadMissing):
        load(b, live)

    def unlisted(m: dict[str, Any]) -> None:
        m["payload"] = []

    with pytest.raises(er.PayloadMissing):
        load(write_bundle(tmp_path / "u", live, unlisted), live)


def test_checksum_mismatch(tmp_path: Path, live: LiveVehicle) -> None:
    b = write_bundle(tmp_path / "b", live)
    data = (b / "weights.npz").read_bytes()
    (b / "weights.npz").write_bytes(data[:-1] + bytes([data[-1] ^ 1]))
    with pytest.raises(er.ChecksumMismatch):
        load(b, live)


def test_vehicle_params_mismatch_both_fields(tmp_path: Path, live: LiveVehicle) -> None:
    def version(m: dict[str, Any]) -> None:
        m["vehicle_params"]["schema_version"] = "0.13.0"

    with pytest.raises(er.VehicleParamsMismatch, match="schema_version"):
        load(write_bundle(tmp_path / "v", live, version), live)

    def session(m: dict[str, Any]) -> None:
        m["vehicle_params"]["sysid_session_id"] = "sysid-2026-11-12"

    with pytest.raises(er.VehicleParamsMismatch, match="sysid_session_id"):
        load(write_bundle(tmp_path / "s", live, session), live)
    # Exact string match: a live vehicle with a different session id refuses the same bundle.
    other_live = dataclasses.replace(live, sysid_session_id="sysid-other")
    with pytest.raises(er.VehicleParamsMismatch):
        load(write_bundle(tmp_path / "ok", live), other_live)


@pytest.mark.parametrize(
    ("section", "key", "value"),
    [
        ("input", "n_bins", 180),
        ("input", "range_clip_m", 5.0),
        ("input", "body_filter_margin_m", 0.05),
        ("input", "tensor", [1, 2, 180]),
        ("output", "stations", 8),
        ("output", "station_spacing_m", 0.25),
        ("output", "tensor", [8, 5]),
    ],
)
def test_io_contract_mismatch_input_output(
    tmp_path: Path, live: LiveVehicle, section: str, key: str, value: Any
) -> None:
    def change(m: dict[str, Any]) -> None:
        m["io"][section][key] = value

    with pytest.raises(er.IoContractMismatch, match=key):
        load(write_bundle(tmp_path / "b", live, change), live)


@pytest.mark.parametrize("key", ["x_m", "y_m", "yaw_rad"])
def test_io_contract_mismatch_mount(tmp_path: Path, live: LiveVehicle, key: str) -> None:
    def change(m: dict[str, Any]) -> None:
        m["io"]["lidar_mount"][key] += 1e-3

    with pytest.raises(er.IoContractMismatch, match=key):
        load(write_bundle(tmp_path / "b", live, change), live)

    def within(m: dict[str, Any]) -> None:
        m["io"]["lidar_mount"][key] += 5e-7

    load(write_bundle(tmp_path / "ok", live, within), live)  # inside the 1e-6 tolerance


def test_io_contract_mismatch_arch(tmp_path: Path, live: LiveVehicle) -> None:
    def change(m: dict[str, Any]) -> None:
        m["io"]["arch"]["type"] = "resnet"

    with pytest.raises(er.IoContractMismatch, match="arch"):
        load(write_bundle(tmp_path / "b", live, change), live)


def test_io_contract_frame_and_origin_checked_below_the_schema(live: LiveVehicle) -> None:
    # The schema already restricts frame, origin and pooling to one value each; the loader's own
    # comparison is the second line, checked directly.
    io = bd.expected_io(live)
    bd._check_io(io, live)
    io["output"]["frame"] = "map"
    with pytest.raises(er.IoContractMismatch, match="frame"):
        bd._check_io(io, live)
    io = bd.expected_io(live)
    io["input"]["origin"] = "base_link"
    with pytest.raises(er.IoContractMismatch, match="origin"):
        bd._check_io(io, live)
    io = bd.expected_io(live)
    io["input"]["pooling"] = "mean"
    with pytest.raises(er.IoContractMismatch, match="pooling"):
        bd._check_io(io, live)


def test_same_helper() -> None:
    assert bd._same(1, 1.0) and not bd._same(1, 2) and not bd._same(float("nan"), 1.0)
    assert bd._same(True, True) and not bd._same(True, 1)
    assert bd._same("a", "a") and not bd._same("a", "b")
