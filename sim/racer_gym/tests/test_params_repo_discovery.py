"""Regression test for racer_gym.params's repo-root discovery (milestone 1 PART A item 3,
housekeeping fix): sim/racer_gym located tools/gen_params.py via a fixed
`Path(__file__).resolve().parents[3]` walk-up that only works when `__file__` still points
at racer_gym's real source tree -- broken for a non-editable install, where the package is
copied into some OTHER package's venv site-packages and no repo tree exists above it (found
during roadmap task S.3; training/racer_train worked around it by forcing
`editable = true` on its racer-gym path dependency instead of fixing the lookup).

Fixed by `racer_gym.params.discover_repo_root`, which tries, in order: the
`RACER_REPO_ROOT` env override, walking up from `__file__`, walking up from the current
working directory -- see that function's docstring for the full rationale. This tests it
directly against synthetic directory layouts (dependency-injected `file_hint`/`cwd_hint`/
`env` parameters) rather than actually building and installing a wheel non-editably, which
would be slow, OS-dependent, and no more informative than exercising the same code paths
directly.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from racer_gym.params import RepoLayoutNotFoundError, discover_repo_root


def _make_repo_layout(root: Path) -> None:
    (root / "config").mkdir(parents=True)
    (root / "config" / "vehicle_params.yaml").write_text("placeholder\n")
    (root / "tools").mkdir(parents=True)
    (root / "tools" / "gen_params.py").write_text("# placeholder\n")


def test_editable_install_case_finds_repo_root_via_file_hint(tmp_path):
    """__file__ still points at the real source tree (editable install, or running straight
    out of the checkout) -- the case the old implementation already handled."""
    repo_root = tmp_path / "fast-car"
    _make_repo_layout(repo_root)
    file_hint = repo_root / "sim" / "racer_gym" / "racer_gym" / "params.py"
    file_hint.parent.mkdir(parents=True)
    file_hint.write_text("# placeholder\n")

    unrelated_cwd = tmp_path / "somewhere-else"
    unrelated_cwd.mkdir()

    found = discover_repo_root(file_hint=file_hint, cwd_hint=unrelated_cwd, env={})
    assert found == repo_root


def test_non_editable_install_case_finds_repo_root_via_cwd_hint(tmp_path):
    """THE regression case: __file__ points into some other package's venv site-packages
    (no repo tree above it at all -- a non-editable path-dependency install copies the
    package there), but the process's cwd is still inside the repo checkout, exactly
    claude-docs/12-testing.md's CI pattern (pytest_gate.sh `cd`s into a package directory
    before `uv sync` + `uv run pytest`, and a non-editable `racer-gym` path dependency of a
    sibling package gets installed into THAT package's own venv)."""
    repo_root = tmp_path / "fast-car"
    _make_repo_layout(repo_root)

    site_packages_copy = (
        tmp_path
        / "some-other-package"
        / ".venv"
        / "lib"
        / "site-packages"
        / "racer_gym"
        / "params.py"
    )
    site_packages_copy.parent.mkdir(parents=True)
    site_packages_copy.write_text("# placeholder (simulated non-editable install copy)\n")

    cwd_inside_repo = repo_root / "training" / "racer_train"
    cwd_inside_repo.mkdir(parents=True)

    found = discover_repo_root(file_hint=site_packages_copy, cwd_hint=cwd_inside_repo, env={})
    assert found == repo_root


def test_env_override_wins_even_when_file_and_cwd_hints_would_both_fail(tmp_path):
    repo_root = tmp_path / "fast-car"
    _make_repo_layout(repo_root)

    unrelated_file = tmp_path / "somewhere" / "params.py"
    unrelated_file.parent.mkdir(parents=True)
    unrelated_file.write_text("# placeholder\n")
    unrelated_cwd = tmp_path / "somewhere-else"
    unrelated_cwd.mkdir()

    found = discover_repo_root(
        file_hint=unrelated_file,
        cwd_hint=unrelated_cwd,
        env={"RACER_REPO_ROOT": str(repo_root)},
    )
    assert found == repo_root


def test_raises_clear_error_naming_every_path_tried_when_repo_layout_is_absent(tmp_path):
    """The fully-detached non-editable case: installed outside any checkout, invoked from
    outside any checkout, no override set. This must be a loud, specific refusal
    (claude-docs/06-vehicle-params.md rule 2), not a silent guess or a bare FileNotFoundError
    somewhere downstream."""
    unrelated_file = tmp_path / "somewhere" / "params.py"
    unrelated_file.parent.mkdir(parents=True)
    unrelated_file.write_text("# placeholder\n")
    unrelated_cwd = tmp_path / "somewhere-else"
    unrelated_cwd.mkdir()

    with pytest.raises(RepoLayoutNotFoundError) as exc_info:
        discover_repo_root(file_hint=unrelated_file, cwd_hint=unrelated_cwd, env={})

    message = str(exc_info.value)
    assert "RACER_REPO_ROOT" in message
    assert str(unrelated_file.parent) in message
    assert str(unrelated_cwd) in message


def test_env_override_pointing_at_a_bad_path_is_not_silently_accepted(tmp_path):
    """An override that itself doesn't have the repo layout must not be trusted blindly --
    it falls through to the file/cwd hints (and ultimately the clear error) like any other
    failed candidate."""
    bad_override = tmp_path / "not-a-repo"
    bad_override.mkdir()
    unrelated_file = tmp_path / "somewhere" / "params.py"
    unrelated_file.parent.mkdir(parents=True)
    unrelated_file.write_text("# placeholder\n")
    unrelated_cwd = tmp_path / "somewhere-else"
    unrelated_cwd.mkdir()

    with pytest.raises(RepoLayoutNotFoundError):
        discover_repo_root(
            file_hint=unrelated_file,
            cwd_hint=unrelated_cwd,
            env={"RACER_REPO_ROOT": str(bad_override)},
        )


def test_real_module_import_resolves_to_the_actual_repo_root():
    """Sanity check that the module-level REPO_ROOT (computed via discover_repo_root() with
    real defaults at import time) is in fact this checkout's root, not just that the
    function works in isolation."""
    from racer_gym.params import DEFAULT_PARAMS_PATH, DEFAULT_SCHEMA_PATH, GEN_PARAMS_PATH

    assert DEFAULT_PARAMS_PATH.is_file()
    assert DEFAULT_SCHEMA_PATH.is_file()
    assert GEN_PARAMS_PATH.is_file()
