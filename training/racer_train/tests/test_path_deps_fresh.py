"""The non-editable path dependencies (racer_gym, envelope, racer_policy) installed in this venv
must be byte-for-byte the source in this checkout.

uv copies a non-editable path dependency into site-packages and rebuilds it only when one of its
`[tool.uv] cache-keys` changes. Before those keys covered the source files, an edit to
racer_gym's dynamics left an existing venv running the old copy, and the only symptom was an
unrelated-looking test failure (an episode in test_live_observation_parity ending at step 43)
on one machine while CI, building fresh, passed. This test names the stale file instead.
"""

from __future__ import annotations

import importlib
from pathlib import Path

import pytest
from conftest import REPO_ROOT

# Installed package -> its source package directory in this checkout.
_PATH_DEPS = {
    "racer_gym": REPO_ROOT / "sim" / "racer_gym" / "racer_gym",
    "envelope": REPO_ROOT / "training" / "envelope" / "src" / "envelope",
    "racer_policy": REPO_ROOT / "ros_ws" / "src" / "racer_policy" / "src" / "racer_policy",
}
_SHIPPED_SUFFIXES = (".py", ".json")


def _shipped_files(root: Path) -> dict[str, bytes]:
    return {
        p.relative_to(root).as_posix(): p.read_bytes()
        for p in sorted(root.rglob("*"))
        if p.is_file() and p.suffix in _SHIPPED_SUFFIXES and "__pycache__" not in p.parts
    }


@pytest.mark.parametrize("name", sorted(_PATH_DEPS))
def test_installed_path_dependency_matches_its_source(name: str) -> None:
    source_dir = _PATH_DEPS[name]
    installed_dir = Path(importlib.import_module(name).__file__).resolve().parent
    if installed_dir == source_dir.resolve():
        return  # an editable install imports the source itself
    installed = _shipped_files(installed_dir)
    source = _shipped_files(source_dir)
    assert source, f"no source files found under {source_dir}"
    stale = sorted(k for k in source.keys() & installed.keys() if source[k] != installed[k])
    missing = sorted(source.keys() - installed.keys())
    extra = sorted(installed.keys() - source.keys())
    assert not (stale or missing or extra), (
        f"{name} installed at {installed_dir} is not the source at {source_dir}: "
        f"differs={stale} missing={missing} extra={extra}. Its [tool.uv] cache-keys should "
        "have made uv rebuild it; `uv sync --reinstall-package` fixes this venv."
    )
