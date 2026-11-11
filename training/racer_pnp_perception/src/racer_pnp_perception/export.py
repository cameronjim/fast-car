"""Write a trained model as a model bundle and its committed pointer.

    uv run --group train python -m racer_pnp_perception.export --run <run dir> \\
        [--model-id trackperc-v0-<date>] [--metrics <json>] [--no-copy]

A bundle (kind track_perception) is `manifest.yaml` + `weights.npz` in
/Users/cameronjim/code/car-data/pnp/models/<model_id>/. The released bundle is also copied to
ros_ws/src/racer_track_perception/test/fixtures/bundle_release/ (about 250 kB, so the sim canary
can drive with it) and a pointer (manifest.yaml, metrics.json, README.md, never the weights) is
written to training/pnp_models/<model_id>/. `finalize` rewrites the metrics and verdict after the
real-bag gate has run.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from numpy.typing import NDArray
from racer_track_perception.bundle import expected_io, load_bundle
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle, find_repo_root, load_live_vehicle

MODELS_DIR = Path("/Users/cameronjim/code/car-data/pnp/models")
SCHEMA_VERSION = "1.0.0"


def git_sha(repo: Path) -> str:
    out = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=repo, capture_output=True, text=True, check=True
    )
    sha = out.stdout.strip()
    if len(sha) != 40:
        raise RuntimeError(f"unexpected git rev-parse output {sha!r}")
    return sha


def default_model_id(today: dt.date | None = None) -> str:
    return f"trackperc-v0-{(today or dt.datetime.now(dt.timezone.utc).date()).isoformat()}"


def build_manifest(
    live: LiveVehicle,
    model_id: str,
    weights_sha256: str,
    config_sha256: str,
    sha: str,
    data_lines: list[str],
    metrics: dict[str, Any],
    created_utc: str | None = None,
) -> dict[str, Any]:
    return {
        "bundle_schema_version": SCHEMA_VERSION,
        "kind": "track_perception",
        "model_id": model_id,
        "created_utc": created_utc
        or dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "git_sha": sha,
        "training_config_sha256": config_sha256,
        "vehicle_params": {
            "schema_version": live.schema_version,
            "sysid_session_id": live.sysid_session_id,
        },
        "payload": [{"file": "weights.npz", "sha256": weights_sha256}],
        "io": expected_io(live),
        "data": data_lines,
        "metrics": metrics,
    }


def write_bundle_dir(
    out: Path,
    arrays: dict[str, NDArray[Any]],
    live: LiveVehicle,
    model_id: str,
    config_sha256: str,
    sha: str,
    data_lines: list[str],
    metrics: dict[str, Any],
) -> Path:
    """Write weights.npz and manifest.yaml into `out` and return it."""
    out.mkdir(parents=True, exist_ok=True)
    np.savez(out / "weights.npz", **arrays)
    weights_sha = hashlib.sha256((out / "weights.npz").read_bytes()).hexdigest()
    manifest = build_manifest(live, model_id, weights_sha, config_sha256, sha, data_lines, metrics)
    write_manifest(out, manifest)
    return out


def write_manifest(bundle_dir: Path, manifest: dict[str, Any]) -> None:
    (bundle_dir / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False, width=100))


def readme_text(model_id: str, metrics: dict[str, Any]) -> str:
    verdict = metrics.get("verdict", "no real-bag verdict yet")
    lines = [
        f"# {model_id}",
        "",
        "Track perception net (`circ_conv1d_v0`), trained in simulation only (labels from the",
        "track geometry; never on the real bags). The weights are not committed: the bundle is in",
        "`/Users/cameronjim/code/car-data/pnp/models/` on the dev machine and copied to the car.",
        "",
        f"Real-bag gate verdict: **{verdict}**.",
        "",
        "Numbers (also in `metrics.json`):",
        "",
        "```json",
        json.dumps(metrics, indent=2, allow_nan=False, default=_json_default),
        "```",
        "",
    ]
    return "\n".join(lines)


def _json_default(value: Any) -> Any:
    if isinstance(value, float) and math.isnan(value):
        return None
    raise TypeError(f"not serialisable: {value!r}")


def _clean(obj: Any) -> Any:
    """NaN and inf to None so the metrics are strict JSON and valid YAML."""
    if isinstance(obj, dict):
        return {k: _clean(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_clean(v) for v in obj]
    if isinstance(obj, (float, np.floating)):
        return None if not np.isfinite(obj) else float(obj)
    return obj


def write_pointer(repo: Path, bundle_dir: Path, metrics: dict[str, Any]) -> Path:
    manifest = yaml.safe_load((bundle_dir / "manifest.yaml").read_text())
    model_id = manifest["model_id"]
    out = repo / "training" / "pnp_models" / model_id
    out.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(bundle_dir / "manifest.yaml", out / "manifest.yaml")
    (out / "metrics.json").write_text(json.dumps(_clean(metrics), indent=2, allow_nan=False))
    (out / "README.md").write_text(readme_text(model_id, _clean(metrics)))
    return out


def copy_release_fixture(repo: Path, bundle_dir: Path) -> Path:
    dest = (
        repo / "ros_ws" / "src" / "racer_track_perception" / "test" / "fixtures" / "bundle_release"
    )
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)
    for name in ("manifest.yaml", "weights.npz"):
        shutil.copyfile(bundle_dir / name, dest / name)
    return dest


def finalize(
    bundle_dir: Path,
    metrics: dict[str, Any],
    repo: Path,
    copy_fixture: bool = True,
    live: LiveVehicle | None = None,
) -> None:
    """Store `metrics` (with the verdict) in the manifest, the pointer and the release fixture."""
    manifest = yaml.safe_load((bundle_dir / "manifest.yaml").read_text())
    manifest["metrics"] = _clean(metrics)
    write_manifest(bundle_dir, manifest)
    write_pointer(repo, bundle_dir, metrics)
    if copy_fixture:
        copy_release_fixture(repo, bundle_dir)
    load_bundle(bundle_dir, "track_perception", live or load_live_vehicle(repo))  # still loads


def export(
    run: Path, model_id: str, metrics: dict[str, Any], data_lines: list[str], repo: Path
) -> Path:
    import torch

    from racer_pnp_perception import model

    net = model.build_model()
    net.load_state_dict(torch.load(run / "model.pt", map_location="cpu"))
    net.eval()
    arrays = model.to_npz_arrays(net)
    # The numpy runtime must reproduce torch before anything is released.
    rng = np.random.default_rng(0)
    x = rng.uniform(0.0, 1.0, size=(8, 2, 360)).astype(np.float32)
    with torch.no_grad():
        want = net(torch.from_numpy(x)).numpy()
    runtime = CircConv1dV0(arrays)
    worst = max(float(np.abs(runtime.forward(x[i : i + 1]) - want[i]).max()) for i in range(8))
    if worst > 1e-4:
        raise RuntimeError(f"numpy runtime differs from torch by {worst}")
    config_sha = hashlib.sha256((run / "config.json").read_bytes()).hexdigest()
    live = load_live_vehicle(repo)
    out = write_bundle_dir(
        MODELS_DIR / model_id,
        arrays,
        live,
        model_id,
        config_sha,
        git_sha(repo),
        data_lines,
        _clean(metrics),
    )
    load_bundle(out, "track_perception", live)
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--model-id", default=default_model_id())
    parser.add_argument("--metrics", type=Path, default=None, help="json file of numbers")
    parser.add_argument("--data", action="append", default=[])
    parser.add_argument("--no-copy", action="store_true")
    args = parser.parse_args(argv)
    repo = find_repo_root()
    metrics = json.loads(args.metrics.read_text()) if args.metrics else {}
    out = export(args.run, args.model_id, metrics, args.data, repo)
    if not args.no_copy:
        write_pointer(repo, out, metrics)
        copy_release_fixture(repo, out)
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
