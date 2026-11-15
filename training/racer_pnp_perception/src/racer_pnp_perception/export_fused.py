"""Export of the `fused-v1` bundle (`trackperc-v2f-<date>`): the frozen v2 weights with
`io.arch.type: circ_conv1d_v0_fused_v1` and the frozen `FuseConfig` in `io.arch.fuse`.

    uv run --group train python -m racer_pnp_perception.export_fused \\
        --source /Users/cameronjim/code/car-data/pnp/models/trackperc-v2-2026-11-14 \\
        --fuse-config <json> --frozen-sha <git sha> [--gp1c gp1c.json] [--latency-json ...]

Without `--gp1c` the bundle is a candidate and no pointer is written. With it, the pointer
directory `training/pnp_models/<model_id>/` gets the manifest, `metrics.json` and a README whose
first line is the GP1c verdict, with the eight items and the M-A1 table. Weights are never
committed; only the pointer is.
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
from racer_track_perception import fuse
from racer_track_perception.bundle import ARCH_TYPE_FUSED_V1, load_bundle
from racer_track_perception.vehicle import find_repo_root, load_live_vehicle

from racer_pnp_perception.export import MODELS_DIR, _clean, _fmt, write_manifest

SOURCE_MODEL_ID = "trackperc-v2-2026-11-14"


def default_model_id(today: dt.date | None = None) -> str:
    return f"trackperc-v2f-{(today or dt.datetime.now(dt.timezone.utc).date()).isoformat()}"


def export_fused(
    source: Path,
    model_id: str,
    cfg: fuse.FuseConfig,
    frozen_sha: str,
    extra: dict[str, Any],
    models_dir: Path = MODELS_DIR,
    repo: Path | None = None,
) -> Path:
    """The fused bundle in `models_dir / model_id`, from the (frozen, not retrained) bundle at
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
    manifest["io"]["arch"] = {"type": ARCH_TYPE_FUSED_V1, "fuse": cfg.as_dict()}
    manifest["data"] = [
        f"the frozen weights of {SOURCE_MODEL_ID}, no retraining (pre-registration v0.4 3.1)",
        (
            "fused-v1: geometric tracer stations and corridor edges, centre from the refined net "
            "where within reach, half widths recomputed against the tracer's edges, the net's far "
            "stations appended (design 13-plug-and-play.md 16.4); reported as geometric plus net "
            "centre, never as the net"
        ),
        f"FuseConfig {json.dumps(cfg.as_dict())}, frozen at git {frozen_sha}",
    ]
    manifest["metrics"] = _clean({"source_model": SOURCE_MODEL_ID, **extra})
    write_manifest(out, manifest)
    bundle = load_bundle(out, "track_perception", live)
    if bundle.fuse_config != cfg:
        raise RuntimeError("the exported bundle does not carry the frozen FuseConfig")
    return out


def _item_table(result: dict[str, Any]) -> list[str]:
    lines = ["| item | result | note |", "|---|---|---|"]
    for i in result["items"]:
        lines.append(
            f"| {i['number']}. {i['name']} | {'PASS' if i['passed'] else 'FAIL'} | {i['note']} |"
        )
    return lines


def _ma1_table(result: dict[str, Any]) -> list[str]:
    lines = [
        (
            "| set | backend | scans | E_c (m) | V | intrusion | residual median (m) | "
            "deficit p90 (m) | C95 |"
        ),
        "|---|---|---|---|---|---|---|---|---|",
    ]
    m = result["M-A1"]
    for backend in ("geometric", "fused", "net", "net_unrefined"):
        h, a = m["H14"][backend], m["A14"][backend]
        lines.append(
            f"| H14 | {backend} | {h['scans']} | {_fmt(h['e_c_m'])} | {_fmt(h['v'])} | n/a | n/a "
            f"| n/a | {_fmt(h['c95'])} |"
        )
        lines.append(
            f"| A14 | {backend} | {a['scans']} | n/a | n/a | {_fmt(a['intrusion_mean'])} "
            f"| {_fmt(a['residual_median_m'])} | {_fmt(a['deficit_p90_m'])} | {_fmt(a['c95'])} |"
        )
    return lines


def readme(model_id: str, metrics: dict[str, Any]) -> str:
    g = metrics["gp1c"]
    lines = [
        f"GP1c verdict: {g['outcome']}",
        "",
        f"# {model_id}",
        "",
        "`fused-v1` (`io.arch.type: circ_conv1d_v0_fused_v1`): geometric plus net centre. The",
        f"weights are those of `{SOURCE_MODEL_ID}`, not retrained. Weights are not committed",
        "(`/Users/cameronjim/code/car-data/pnp/models/`). Decision rule:",
        "`docs/notes/preregistration-pnp-v0.4.md` section 3.2, constants",
        f"`evaluation/prereg/pnp-v0.4.yaml` (SHA-256 `{g.get('prereg_sha256')}`).",
        f"Frozen at git `{g.get('frozen_sha')}`; FuseConfig `{json.dumps(g.get('fuse_config'))}`.",
        "",
        "Items 3 and 5 hold largely BY CONSTRUCTION (the valid count is at least the tracer's and the",
        "corridor on shared stations is the tracer's). Geometric stays the launch default in every",
        "case. A pass means a candidate backend in the layout family, on a fresh session, and says",
        "nothing about unseen layouts.",
        "",
        *_item_table(g),
        "",
        "### M-A1 and the GP1c numbers, side by side",
        "",
        *_ma1_table(g),
        "",
    ]
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
    parser.add_argument("--gp1c", type=Path, default=None)
    parser.add_argument("--latency-json", type=Path, default=None)
    parser.add_argument("--models-dir", type=Path, default=MODELS_DIR)
    args = parser.parse_args(argv)
    repo = find_repo_root()
    cfg = fuse.FuseConfig.from_dict(json.loads(args.fuse_config.read_text()))
    extra: dict[str, Any] = {}
    if args.latency_json:
        extra["latency_ms"] = json.loads(args.latency_json.read_text())
    if args.gp1c:
        extra["gp1c"] = json.loads(args.gp1c.read_text())
    out = export_fused(
        args.source, args.model_id, cfg, args.frozen_sha, extra, args.models_dir, repo
    )
    if args.gp1c:
        metrics = yaml.safe_load((out / "manifest.yaml").read_text())["metrics"]
        write_pointer(repo, out, metrics)
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
