"""L1 smoke test for the CLI entry point, writing to a temp directory (never the repo's
own config/tracks/ -- the one committed reference raceline is generated once, by hand, and
checked in; tests must not regenerate or overwrite it as a side effect of running)."""

from __future__ import annotations

from raceline.cli import main
from raceline.io import read_raceline_csv


def test_cli_writes_a_readable_raceline_csv(tmp_path):
    out_dir = tmp_path / "tracks"
    rc = main(
        [
            "--track-id",
            "test_track",
            "--straight-length-m",
            "8.0",
            "--turn-radius-m",
            "3.0",
            "--out-dir",
            str(out_dir),
        ]
    )
    assert rc == 0
    out_path = out_dir / "test_track" / "raceline.csv"
    assert out_path.exists()

    raceline, provenance = read_raceline_csv(out_path)
    assert len(raceline) > 50
    assert provenance.generation_params["track_shape"] == "stadium"


def test_cli_centerline_csv_mode_writes_a_readable_raceline_csv(tmp_path):
    """roadmap milestone 5: --centerline-csv loads an external (e.g. real f1tenth_gym map)
    centerline instead of generating the synthetic stadium shape, through the SAME optimizer
    pipeline (build_raceline_from_centerline is not duplicated or reimplemented)."""
    centerline_path = tmp_path / "source_centerline.csv"
    # A small closed loop (a square), same fixture shape as
    # config/tracks/oschersleben/source_centerline.csv's own format.
    centerline_path.write_text(
        "# x_m, y_m, w_tr_right_m, w_tr_left_m\n"
        "0.0, 0.0, 1.1, 1.1\n"
        "4.0, 0.0, 1.1, 1.1\n"
        "4.0, 4.0, 1.1, 1.1\n"
        "0.0, 4.0, 1.1, 1.1\n"
    )
    out_dir = tmp_path / "tracks"
    rc = main(
        [
            "--track-id",
            "test_external_track",
            "--centerline-csv",
            str(centerline_path),
            "--out-dir",
            str(out_dir),
        ]
    )
    assert rc == 0
    out_path = out_dir / "test_external_track" / "raceline.csv"
    assert out_path.exists()

    raceline, provenance = read_raceline_csv(out_path)
    assert len(raceline) > 50
    assert provenance.generation_params["track_shape"] == "external_centerline"
    assert provenance.generation_params["source_centerline_csv"] == str(centerline_path)
