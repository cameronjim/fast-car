"""The v0.4 CLI module imports and names its stage 1 commands (the real-data glue is not run in CI)."""

from __future__ import annotations

from racer_pnp_dynamics import cli_v04


def test_the_stage_commands_exist_and_the_cache_names_are_per_base():
    from racer_pnp_dynamics import stages_v04

    assert set(stages_v04.STAGE_COMMANDS) == {
        "n4-fit",
        "r4-fit",
        "t4-fit",
        "tl4-fit",
        "freeze-eval",
    }
    assert stages_v04._base_name("dyn-v0.1") == "dynv01" and stages_v04._base_name("N4") == "N4"
    assert stages_v04.MEASUREMENTS == ("M-B3.md", "M-B2.md", "M-B4.md")


def test_the_cli_names_its_commands():
    assert set(cli_v04.COMMANDS) == {
        "truth-train",
        "truth-variants",
        "budget",
        "mb4",
        "n4-fit",
        "r4-fit",
        "t4-fit",
        "tl4-fit",
        "freeze-eval",
    }
    assert cli_v04.OUT_DIR == "dyn-v0.4-2026-11-14"
