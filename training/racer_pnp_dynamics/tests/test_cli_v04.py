"""The v0.4 CLI module imports and names its stage 1 commands (the real-data glue is not run in CI)."""

from __future__ import annotations

from racer_pnp_dynamics import cli_v04


def test_the_cli_names_only_stage_one_commands():
    assert set(cli_v04.COMMANDS) == {"truth-train", "truth-variants", "budget", "mb4"}
    assert cli_v04.OUT_DIR == "dyn-v0.4-2026-11-14"
