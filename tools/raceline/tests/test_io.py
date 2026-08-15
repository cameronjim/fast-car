"""L1 tests for raceline.io: provenanced CSV round-trip."""

from __future__ import annotations

import numpy as np
from raceline.io import read_raceline_csv, write_raceline_csv
from raceline.raceline import Raceline


def _tiny_raceline() -> Raceline:
    n = 5
    return Raceline(
        s_m=np.linspace(0.0, 4.0, n),
        x_m=np.linspace(0.0, 4.0, n),
        y_m=np.zeros(n),
        heading_rad=np.zeros(n),
        curvature_1pm=np.zeros(n),
        target_speed_mps=np.full(n, 3.0),
    )


def test_write_then_read_round_trips_arrays(tmp_path):
    original = _tiny_raceline()
    path = tmp_path / "gym_oval" / "raceline.csv"
    write_raceline_csv(
        path,
        original,
        track_id="gym_oval",
        vehicle_params_schema_version="0.1.0",
        vehicle_params_sysid_session_id="none-preliminary",
        generation_params={"track_shape": "stadium", "turn_radius_m": 3.0},
    )
    assert path.exists()

    loaded, _provenance = read_raceline_csv(path)
    np.testing.assert_allclose(loaded.s_m, original.s_m)
    np.testing.assert_allclose(loaded.x_m, original.x_m)
    np.testing.assert_allclose(loaded.y_m, original.y_m)
    np.testing.assert_allclose(loaded.heading_rad, original.heading_rad)
    np.testing.assert_allclose(loaded.curvature_1pm, original.curvature_1pm)
    np.testing.assert_allclose(loaded.target_speed_mps, original.target_speed_mps)


def test_provenance_header_is_parsed_back(tmp_path):
    original = _tiny_raceline()
    path = tmp_path / "raceline.csv"
    write_raceline_csv(
        path,
        original,
        track_id="gym_oval",
        vehicle_params_schema_version="0.1.0",
        vehicle_params_sysid_session_id="none-preliminary",
        generation_params={"track_shape": "stadium", "turn_radius_m": "3.0"},
    )
    _, provenance = read_raceline_csv(path)
    assert provenance.tool_version != ""
    assert provenance.vehicle_params_schema_version == "0.1.0"
    assert provenance.vehicle_params_sysid_session_id == "none-preliminary"
    assert provenance.generation_params["track_shape"] == "stadium"
    assert provenance.generation_params["turn_radius_m"] == "3.0"


def test_written_file_has_hash_commented_header(tmp_path):
    original = _tiny_raceline()
    path = tmp_path / "raceline.csv"
    write_raceline_csv(
        path,
        original,
        track_id="gym_oval",
        vehicle_params_schema_version="0.1.0",
        vehicle_params_sysid_session_id="none-preliminary",
        generation_params={},
    )
    text = path.read_text()
    header_lines = [line for line in text.splitlines() if line.startswith("#")]
    assert len(header_lines) >= 4
    assert any("raceline for track_id=gym_oval" in line for line in header_lines)


def test_written_file_uses_lf_line_endings_only(tmp_path):
    """Regression test: csv.writer's RFC 4180 default is CRLF, which silently corrupts
    the last column of every row (and the header match) for the simple C++ reader
    (ros_ws/src/racer_control/include/racer_control/raceline.hpp), which splits lines on
    '\\n' only via std::getline and does not itself expect a trailing '\\r'. Caught by the
    L5 tracker lap canary's first CI run: tracker_node refused to start because the
    committed raceline's header didn't byte-match. write_raceline_csv must always emit
    plain '\\n' line endings, matching the '#'-commented header lines above it."""
    original = _tiny_raceline()
    path = tmp_path / "raceline.csv"
    write_raceline_csv(
        path,
        original,
        track_id="gym_oval",
        vehicle_params_schema_version="0.1.0",
        vehicle_params_sysid_session_id="none-preliminary",
        generation_params={},
    )
    raw = path.read_bytes()
    assert b"\r" not in raw, "written raceline CSV must use LF-only line endings, found a CR byte"
