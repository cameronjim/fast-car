# Fixture support for tests/test_from_vehicle_params_wiring.py: regenerates the real Python
# vehicle_params binding via tools/gen_params.py (claude-docs/06-vehicle-params.md rule 3 --
# bindings are generated, never hand-written, never committed) and hands the resulting
# VEHICLE_PARAMS instance to tests as a fixture. Nothing here is committed either: it writes
# to pytest's tmp_path and is discarded after the test session.
from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

# tests/conftest.py -> tests/ -> envelope/ -> training/ -> repo root.
REPO_ROOT = Path(__file__).resolve().parents[3]
GEN_PARAMS_SCRIPT = REPO_ROOT / "tools" / "gen_params.py"


@pytest.fixture
def real_vehicle_params(tmp_path: Path) -> Any:
    """Regenerate the Python vehicle_params binding from the committed config and return
    its VEHICLE_PARAMS instance."""
    out_dir = tmp_path / "generated"
    subprocess.run(
        [sys.executable, str(GEN_PARAMS_SCRIPT), "--out-dir", str(out_dir)],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    generated_path = out_dir / "vehicle_params_generated.py"
    source = generated_path.read_text(encoding="utf-8")
    namespace: dict[str, Any] = {}
    # Deliberate: this executes gen_params.py's own freshly-regenerated output (never
    # committed, never touched by anything untrusted) purely so the test doesn't need a
    # second sys.path-manipulation mechanism just to import a tmp_path module.
    exec(compile(source, str(generated_path), "exec"), namespace)  # noqa: S102
    return namespace["VEHICLE_PARAMS"]
