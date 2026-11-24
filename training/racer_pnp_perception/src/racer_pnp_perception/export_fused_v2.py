"""Export of the `fused-v2` bundle (`trackperc-v2g-<date>`): the frozen v2 weights with
`io.arch.type: circ_conv1d_v0_fused_v2` and the frozen `FuseV2Config` in `io.arch.fuse`.

    uv run --group train python -m racer_pnp_perception.export_fused_v2 \\
        --source /Users/cameronjim/code/car-data/pnp/models/trackperc-v2-2026-11-14 \\
        --fuse-config <json> --frozen-sha <git sha> [--gp1d gp1d.json] [--latency-json ...]

Without `--gp1d` the bundle is a candidate and no pointer is written. With it, the pointer directory
`training/pnp_models/<model_id>/` gets the manifest, `metrics.json` and a README whose first line is
the GP1d verdict. Weights are never committed; only the pointer is.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import shutil
import sys
from pathlib import Path
from typing import Any

import yaml
from racer_track_perception import fuse_v2
from racer_track_perception.bundle import ARCH_TYPE_FUSED_V2, load_bundle
from racer_track_perception.vehicle import find_repo_root, load_live_vehicle

from racer_pnp_perception import gp1d
from racer_pnp_perception.export import MODELS_DIR, _clean, write_manifest
from racer_pnp_perception.export_fused import SOURCE_MODEL_ID


def default_model_id(today: dt.date | None = None) -> str:
    return f"trackperc-v2g-{(today or dt.datetime.now(dt.timezone.utc).date()).isoformat()}"


def export_fused_v2(
    source: Path,
    model_id: str,
    cfg: fuse_v2.FuseV2Config,
    frozen_sha: str,
    extra: dict[str, Any],
    models_dir: Path = MODELS_DIR,
    repo: Path | None = None,
) -> Path:
    """The fused-v2 bundle in `models_dir / model_id`, from the (frozen, not retrained) bundle at
    `source`. The weights file is copied byte for byte; the loader verifies the result."""
    live = load_live_vehicle(repo)
    manifest = yaml.safe_load((source / "manifest.yaml").read_text())
    if manifest["io"]["arch"]["type"] not in ("circ_conv1d_v0_refine_v1", "circ_conv1d_v0"):
        raise RuntimeError("the source bundle is not a plain or refine_v1 bundle")
    if len(frozen_sha) != 40:
        raise RuntimeError(f"frozen sha {frozen_sha!r} is not a full git sha")
    cfg.validate()
    out = models_dir / model_id
    out.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source / "weights.npz", out / "weights.npz")
    weights_sha = hashlib.sha256((out / "weights.npz").read_bytes()).hexdigest()
    manifest["model_id"] = model_id
    manifest["created_utc"] = dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    manifest["git_sha"] = frozen_sha
    manifest["payload"] = [{"file": "weights.npz", "sha256": weights_sha}]
    manifest["io"]["arch"] = {"type": ARCH_TYPE_FUSED_V2, "fuse": cfg.as_dict()}
    manifest["data"] = [
        f"the frozen weights of {SOURCE_MODEL_ID}, no retraining (pre-registration v0.6)",
        (
            "fused-v2: geometric tracer edge polylines, centre from the refined net where within "
            "reach, half widths by ray cast along the output's own normals, the net's far "
            "stations appended; reported as geometric plus net centre, never as the net"
        ),
        f"FuseV2Config {json.dumps(cfg.as_dict())}, frozen at git {frozen_sha}",
    ]
    manifest["metrics"] = _clean({"source_model": SOURCE_MODEL_ID, **extra})
    write_manifest(out, manifest)
    bundle = load_bundle(out, "track_perception", live)
    if bundle.fuse_config != cfg:
        raise RuntimeError("the exported bundle does not carry the frozen FuseV2Config")
    return out


def readme(model_id: str, metrics: dict[str, Any]) -> str:
    g = metrics["gp1d"]
    lines = [
        f"GP1d verdict: {g['outcome']}",
        "",
        f"# {model_id}",
        "",
        "`fused-v2` (`io.arch.type: circ_conv1d_v0_fused_v2`): geometric edges plus net centre. The",
        f"weights are those of `{SOURCE_MODEL_ID}`, not retrained. Weights are not committed",
        "(`/Users/cameronjim/code/car-data/pnp/models/`). Rule:",
        "`docs/notes/preregistration-pnp-v0.6-perception-fused.md`, constants",
        f"`evaluation/prereg/pnp-v0.6-perception-fused.yaml` (SHA-256 `{g.get('prereg_sha256')}`).",
        f"Frozen at git `{g.get('frozen_sha')}`; FuseV2Config `{json.dumps(g.get('fuse_config'))}`.",
        "",
        "Geometric stays the launch default in every case. " + gp1d.ALWAYS_SAID,
        "",
    ]
    for label, s in sorted(g["sets"].items()):
        lines += [f"### Set {label}", "", "| item | result |", "|---|---|"]
        lines += [
            f"| {i['number']}. {i['name']} | {'PASS' if i['passed'] else 'FAIL'} |"
            for i in s["items"]
        ]
        lines += [
            f"| K{i['number']}. {i['name']} | {'PASS' if i['passed'] else 'FAIL'} |"
            for i in s["k_items"]
        ]
        lines.append("")
    lines += ["### Global items", "", "| item | result | note |", "|---|---|---|"]
    lines += [
        f"| {i['number']}. {i['name']} | {'PASS' if i['passed'] else 'FAIL'} | {i['note']} |"
        for i in g["global_items"]
    ]
    lines.append("")
    if "latency_ms" in metrics:
        lines += ["Latency (dev machine, one thread): " + json.dumps(metrics["latency_ms"]), ""]
    return "\n".join(lines) + "\n"


def write_pointer(repo: Path, bundle_dir: Path, metrics: dict[str, Any]) -> Path:
    manifest = yaml.safe_load((bundle_dir / "manifest.yaml").read_text())
    model_id = manifest["model_id"]
    out = repo / "training" / "pnp_models" / model_id
    out.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(bundle_dir / "manifest.yaml", out / "manifest.yaml")
    clean = _clean(metrics)
    (out / "metrics.json").write_text(json.dumps(clean, indent=2, allow_nan=False))
    (out / "README.md").write_text(readme(model_id, clean))
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=MODELS_DIR / SOURCE_MODEL_ID)
    parser.add_argument("--fuse-config", type=Path, required=True)
    parser.add_argument("--frozen-sha", required=True)
    parser.add_argument("--model-id", default=default_model_id())
    parser.add_argument("--gp1d", type=Path, default=None)
    parser.add_argument("--latency-json", type=Path, default=None)
    parser.add_argument("--models-dir", type=Path, default=MODELS_DIR)
    args = parser.parse_args(argv)
    repo = find_repo_root()
    cfg = fuse_v2.FuseV2Config.from_dict(json.loads(args.fuse_config.read_text()))
    extra: dict[str, Any] = {}
    if args.latency_json:
        extra["latency_ms"] = json.loads(args.latency_json.read_text())
    if args.gp1d:
        extra["gp1d"] = json.loads(args.gp1d.read_text())
    out = export_fused_v2(
        args.source, args.model_id, cfg, args.frozen_sha, extra, args.models_dir, repo
    )
    if args.gp1d:
        metrics = yaml.safe_load((out / "manifest.yaml").read_text())["metrics"]
        write_pointer(repo, out, metrics)
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
