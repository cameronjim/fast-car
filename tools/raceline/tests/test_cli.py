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
