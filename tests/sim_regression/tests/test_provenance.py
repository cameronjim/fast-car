"""Unit tests for provenance.build_provenance_meta and its best-effort git-commit lookup."""

from __future__ import annotations

import dataclasses

import racer_gym
from racer_sim_regression import provenance


@dataclasses.dataclass(frozen=True)
class _FakeDynParamsResult:
    fallback_flags: dict[str, bool]


class TestRepoCommit:
    def test_returns_a_commit_sha_in_this_checkout(self):
        commit = provenance._repo_commit()
        assert commit is None or (isinstance(commit, str) and len(commit) == 40)

    def test_returns_none_when_git_is_unavailable(self, monkeypatch):
        def _raise(*_args, **_kwargs):
            raise FileNotFoundError("git not found")

        monkeypatch.setattr(provenance.subprocess, "run", _raise)
        assert provenance._repo_commit() is None


class TestBuildProvenanceMeta:
    def test_includes_expected_keys(self):
        vehicle_params = racer_gym.load_vehicle_params()
        dyn_params_result = _FakeDynParamsResult(fallback_flags={"pacejka_front": True})
        meta = provenance.build_provenance_meta(
            seed=42, dyn_params_result=dyn_params_result, vehicle_params=vehicle_params
        )
        assert meta["seed"] == 42
        assert meta["vehicle_params_fallback_flags"] == {"pacejka_front": True}
        assert meta["vehicle_params_meta"]["schema_version"] == vehicle_params.meta.schema_version
        assert isinstance(meta["racer_gym_version"], str)

    def test_unknown_distribution_falls_back_gracefully(self, monkeypatch):
        from importlib import metadata as importlib_metadata

        def _raise(_name):
            raise importlib_metadata.PackageNotFoundError(_name)

        monkeypatch.setattr(importlib_metadata, "version", _raise)
        assert "unknown" in provenance._racer_gym_version()
