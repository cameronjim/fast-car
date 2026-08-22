"""L1 test: the raceline tool reads vehicle_params through the GENERATED binding, matching
the committed yaml (claude-docs/06-vehicle-params.md rule 3, CLAUDE.md invariant 2)."""

from __future__ import annotations

import yaml
from raceline.params_loader import _TOOLS_DIR, load_vehicle_params


def test_loaded_bindings_match_committed_yaml():
    repo_root = _TOOLS_DIR.parent
    with (repo_root / "config" / "vehicle_params.yaml").open() as f:
        raw = yaml.safe_load(f)

    params = load_vehicle_params()

    assert params.actuation.max_acceleration_mps2 == raw["actuation"]["max_acceleration_mps2"]
    assert params.limits.global_speed_cap_mps == raw["limits"]["global_speed_cap_mps"]
    assert params.meta.schema_version == raw["meta"]["schema_version"]
    assert params.meta.sysid_session_id == raw["meta"]["sysid_session_id"]


def test_load_vehicle_params_never_writes_a_generated_file_into_the_repo():
    repo_root = _TOOLS_DIR.parent
    before = set((repo_root / "config").rglob("*_generated.*"))
    load_vehicle_params()
    after = set((repo_root / "config").rglob("*_generated.*"))
    assert before == after == set()
