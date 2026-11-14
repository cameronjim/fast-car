"""Write a trained model as a model bundle and its committed pointer.

    uv run --group train python -m racer_pnp_perception.export --run <run dir> \\
        [--model-id trackperc-v0-<date>] [--metrics <json>] [--no-copy]

A bundle (kind track_perception) is `manifest.yaml` + `weights.npz` in
/Users/cameronjim/code/car-data/pnp/models/<model_id>/. The released bundle is also copied to
ros_ws/src/racer_track_perception/test/fixtures/bundle_release/ (about 250 kB, so the sim canary
can drive with it) and a pointer (manifest.yaml, metrics.json, README.md, never the weights) is
written to training/pnp_models/<model_id>/. `finalize` rewrites the metrics and verdict after the
real-bag gate has run.

    uv run --group train python -m racer_pnp_perception.export --v1 --run <run dir> \\
        --gp1 <gp1.json> [--sim-json <eval_sim.json>] [--latency-json <latency.json>] \\
        [--model-id trackperc-v1-<UTC date>] [--models-dir <dir>] [--copy-release-fixture]

Without --gp1 it writes a candidate bundle only (for the GP1 run, which needs a bundle to load).
With it, it writes the v1 bundle (vehicle_params pinned to the live 0.17.0 / none-preliminary, the training
data listed, the whole gp1.json and the split SHA-256 in `metrics`) and its pointer, whose README
starts with the GP1 verdict. The release fixture is replaced only with `--copy-release-fixture`,
and the code refuses that unless the GP1 outcome is a pass.
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
# Every v0.2 bundle pins exactly this identity (it must also equal the live vehicle_params).
PINNED_VEHICLE_PARAMS = ("0.17.0", "none-preliminary")


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


def compose_metrics(
    sim: dict[str, Any], bags: dict[str, Any], train_history: list[dict[str, Any]] | None = None
) -> dict[str, Any]:
    """The numbers stored in the manifest, the pointer and the README: the held-out simulation
    summary, the real-bag gate (pooled and per bag, both backends) and the verdict."""
    held = {
        name: {k: res[k] for k in ("net_raw", "geometric_valid") if k in res}
        for name, res in sim["held_out"].items()
    }
    out: dict[str, Any] = {
        "verdict": bags.get("verdict", "no real-bag verdict"),
        "sim_held_out": held,
        "sim_held_out_acceptance": sim.get("held_out_acceptance"),
        "sim_in_distribution": {
            k: sim["in_distribution"][k]
            for k in ("net_raw", "geometric_valid")
            if k in sim["in_distribution"]
        },
        "real_bags_gate_pooled": bags.get("gate_pooled"),
        "real_bags": bags.get("bags"),
    }
    if train_history:
        out["final_val"] = train_history[-1]
    return out


def readme_text(model_id: str, metrics: dict[str, Any]) -> str:
    verdict = metrics.get("verdict", "no real-bag verdict yet")
    lines = [
        f"# {model_id}",
        "",
        "Track perception net (`circ_conv1d_v0`), trained in simulation only: labels come from the",
        "track geometry and the real bags were never trained on. The weights are not committed;",
        "the bundle lives in `/Users/cameronjim/code/car-data/pnp/models/` on the dev machine and",
        "is copied to the car.",
        "",
        f"Real-bag gate verdict: **{verdict}**.",
        "",
    ]
    pooled = metrics.get("real_bags_gate_pooled") or {}
    if pooled:
        lines += [
            "Real-bag gate (both 2026-11-11T03-54-11 bags together, moving scans). The net may drive",
            "only if the median boundary residual is at most 0.10 m, the mean intrusion fraction at",
            "most 0.02 and at most 5 percent of scans have valid_count under 6. The thresholds were",
            "not tuned.",
            "",
            "| backend | median residual (m) | p90 residual (m) | mean intrusion | share valid < 6 |",
            "|---|---|---|---|---|",
        ]
        for name, row in pooled.items():
            lines.append(
                f"| {name} | {_fmt(row['boundary_residual_median_m'])} "
                f"| {_fmt(row['boundary_residual_p90_m'])} "
                f"| {_fmt(row['intrusion_fraction_mean'])} "
                f"| {_fmt(row['share_valid_count_under_6'])} |"
            )
        lines.append("")
    held = metrics.get("sim_held_out") or {}
    if held:
        lines += [
            "Simulation, held-out family tracks (net, median error over stations): centreline",
            "0 to 7 / 8 to 15 and half width, in metres.",
            "",
            "| track | centreline 0-7 | centreline 8-15 | half width |",
            "|---|---|---|---|",
        ]
        for name, res in held.items():
            n = res["net_raw"]
            lines.append(
                f"| {name} | {_fmt(n['centreline_err_median_stations_0_7_m'])} "
                f"| {_fmt(n['centreline_err_median_stations_8_15_m'])} "
                f"| {_fmt(n['half_width_err_median_m'])} |"
            )
        lines.append("")
    lines += ["Everything, as stored in the bundle manifest (`metrics.json` is the same):", ""]
    return "\n".join(lines) + "\n"


def _fmt(value: Any) -> str:
    return "n/a" if value is None else f"{value:.3f}"


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
    if worst > 1e-5:
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


# ---------------------------------------------------------------------------------------------
# v1: bundle and pointer for the net trained on sim plus real self-labels
# ---------------------------------------------------------------------------------------------


def default_model_id_v1(today: dt.date | None = None) -> str:
    return f"trackperc-v1-{(today or dt.datetime.now(dt.timezone.utc).date()).isoformat()}"


def check_pinned(live: LiveVehicle) -> None:
    """A v0.2 bundle is only ever written for the pinned vehicle_params identity."""
    if (live.schema_version, live.sysid_session_id) != PINNED_VEHICLE_PARAMS:
        raise RuntimeError(
            f"live vehicle_params is {live.schema_version} / {live.sysid_session_id}, a v0.2 "
            f"bundle pins {PINNED_VEHICLE_PARAMS[0]} / {PINNED_VEHICLE_PARAMS[1]}"
        )


def data_lines_v1(config: dict[str, Any], selection: dict[str, Any] | None) -> list[str]:
    """What the net was trained on, for the manifest `data` field."""
    real = config.get("real") or {}
    lines = [
        (
            f"simulated lanes: {Path(config['train']).name} ({config['epochs']} epochs, batch "
            f"{config['batch_size']}, seed {config['seed']}), validation "
            f"{Path(config['val']).name}; nuisances per racer_pnp_perception.nuisance"
        ),
        (
            "real self-labels (geometry seen later by the same sensor, no command topic read): "
            f"{real.get('train_labels', 0)} train labels, share {config.get('real_fraction')} of "
            f"each batch, mirrored and bin dropout; {real.get('val_labels', 0)} labels held "
            "back as internal validation"
        ),
        "real segments used (role train only): " + ", ".join(real.get("segments_used", [])),
        f"split pnp-v0.2 SHA-256 {real.get('split_sha256', 'none')}",
        "never trained on held-out or cross-layout data; held-out scored once, see metrics.gp1",
    ]
    if config.get("init"):
        lines.append(f"initialised from {config['init']} (sim-only v0 weights)")
    if selection:
        lines.append(
            f"seed {selection['selected_seed']} of {sorted(selection['per_seed'])} chosen by "
            f"{selection['criterion']} (internal validation only)"
        )
    return lines


def readme_v1(model_id: str, metrics: dict[str, Any]) -> str:
    """README of the v1 pointer. The first line is the GP1 verdict."""
    gp1 = metrics["gp1"]
    lines = [
        f"GP1 verdict: {gp1['outcome']}",
        "",
        f"# {model_id}",
        "",
        "Track perception net (`circ_conv1d_v0`) trained on simulated lanes and on self-labels of",
        "the car's own scans (geometry seen later by the same sensor), role `train` of the",
        "pre-registered split only. The weights are not committed; the bundle lives in",
        "`/Users/cameronjim/code/car-data/pnp/models/` on the dev machine. Decision rule:",
        "`docs/notes/preregistration-pnp-v0.2.md` section 2, evaluated once on the held-out sets.",
        "",
        f"Split SHA-256: `{gp1['split_sha256']}`.",
        "",
        "| set | backend | labelled scans | E_c (m) | E_far (m) | V | C95 |",
        "|---|---|---|---|---|---|---|",
    ]
    for name in ("H-run", "H-layout"):
        for backend, v in gp1[name].items():
            lines.append(
                f"| {name} | {backend} | {v['scans']} | {_fmt(v['e_c_m'])} | {_fmt(v['e_far_m'])} "
                f"| {_fmt(v['v'])} | {_fmt(v['c95'])} |"
            )
    lines += [
        "",
        (
            "| set | backend | moving scans | mean intrusion | residual median (m) "
            "| deficit p90 (m) | C95 |"
        ),
        "|---|---|---|---|---|---|---|",
    ]
    for name in ("A-run", "A-layout"):
        for backend, v in gp1[name].items():
            lines.append(
                f"| {name} | {backend} | {v['scans']} | {_fmt(v['intrusion_mean'])} "
                f"| {_fmt(v['residual_median_m'])} | {_fmt(v['deficit_p90_m'])} "
                f"| {_fmt(v['c95'])} |"
            )
    boot = gp1["bootstrap_e_c_ratio_net_over_geometric_h_run"]
    lines += [
        "",
        (
            f"Bootstrap of E_c(net) / E_c(geometric) on H-run: {_fmt(boot['ratio'])}, 95 percent "
            f"interval [{_fmt(boot['lo'])}, {_fmt(boot['hi'])}] ({boot['blocks']} blocks of "
            f"{boot['block_s']} s, {boot['resamples']} resamples, seed {boot['seed']})."
        ),
        "",
        "| item | result | value |",
        "|---|---|---|",
    ]
    for item in gp1["items"]:
        lines.append(
            f"| {item['number']}. {item['name']} | {'pass' if item['passed'] else 'FAIL'} "
            f"| `{json.dumps(item['value'])}` |"
        )
    lines += ["", "| cell | labelled scans | E_c ratio | judged |", "|---|---|---|---|"]
    for c in gp1["cells"]:
        lines.append(
            f"| {c['set']} {c['direction']} {c['speed_bin']} | {c['scans']} | "
            f"{_fmt(c['ratio'])} | {'yes' if c['judged'] else 'no'} |"
        )
    lat = metrics.get("latency_numpy_forward_ms")
    if lat:
        lines += [
            "",
            (
                f"Numpy forward latency on the dev Mac, single thread, {lat['scans']} real "
                f"scans: p50 {_fmt(lat['p50'])} ms, p99 {_fmt(lat['p99'])} ms (a proxy; the "
                "Orin number is taken at the car)."
            ),
        ]
    sim = metrics.get("sim") or {}
    v0 = metrics.get("sim_v0_held_out_net_raw") or {}
    if sim.get("held_out"):
        keys = (
            "centreline_err_median_stations_0_7_m",
            "centreline_err_median_stations_8_15_m",
            "half_width_err_median_m",
        )
        lines += [
            "",
            (
                "Simulation, held-out family tracks, net raw output, median error in metres "
                "(v1 next to v0, no threshold):"
            ),
            "",
            "| track | centreline 0-7 v1 / v0 | centreline 8-15 v1 / v0 | half width v1 / v0 |",
            "|---|---|---|---|",
        ]
        for name, res in sim["held_out"].items():
            cells = [
                f"{_fmt(res['net_raw'][k])} / {_fmt((v0.get(name) or {}).get(k))}" for k in keys
            ]
            lines.append(f"| {name} | " + " | ".join(cells) + " |")
    lines += ["", "Everything, as stored in the bundle manifest (`metrics.json` is the same):", ""]
    return "\n".join(lines) + "\n"


def write_pointer_v1(repo: Path, bundle_dir: Path, metrics: dict[str, Any]) -> Path:
    manifest = yaml.safe_load((bundle_dir / "manifest.yaml").read_text())
    model_id = manifest["model_id"]
    out = repo / "training" / "pnp_models" / model_id
    out.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(bundle_dir / "manifest.yaml", out / "manifest.yaml")
    clean = _clean(metrics)
    (out / "metrics.json").write_text(json.dumps(clean, indent=2, allow_nan=False))
    (out / "README.md").write_text(readme_v1(model_id, clean))
    return out


def export_v1(
    run: Path,
    model_id: str,
    gp1_result: dict[str, Any] | None,
    extra_metrics: dict[str, Any],
    repo: Path,
    selection: dict[str, Any] | None = None,
    models_dir: Path = MODELS_DIR,
) -> Path:
    """The v1 bundle from a training run directory. `metrics` carries the whole gp1 result. With
    no gp1 result the bundle is a candidate (GP1 needs a bundle to load, and runs once on it);
    the final bundle is written again from the same run once GP1 has been decided."""
    import torch

    from racer_pnp_perception import model

    live = load_live_vehicle(repo)
    check_pinned(live)
    net = model.build_model()
    net.load_state_dict(torch.load(run / "model.pt", map_location="cpu"))
    net.eval()
    arrays = model.to_npz_arrays(net)
    rng = np.random.default_rng(0)
    x = rng.uniform(0.0, 1.0, size=(8, 2, 360)).astype(np.float32)
    with torch.no_grad():
        want = net(torch.from_numpy(x)).numpy()
    runtime = CircConv1dV0(arrays)
    worst = max(float(np.abs(runtime.forward(x[i : i + 1]) - want[i]).max()) for i in range(8))
    if worst > 1e-5:
        raise RuntimeError(f"numpy runtime differs from torch by {worst}")
    config = json.loads((run / "config.json").read_text())
    metrics: dict[str, Any] = {"numpy_vs_torch_max_abs_diff": worst, **extra_metrics}
    if gp1_result is None:
        metrics["status"] = "candidate: GP1 not yet decided"
    else:
        metrics = {"gp1": gp1_result, "split_sha256": gp1_result["split_sha256"], **metrics}
    return write_v1_bundle(
        models_dir / model_id,
        arrays,
        live,
        model_id,
        hashlib.sha256((run / "config.json").read_bytes()).hexdigest(),
        git_sha(repo),
        data_lines_v1(config, selection),
        metrics,
    )


def write_v1_bundle(
    out: Path,
    arrays: dict[str, NDArray[Any]],
    live: LiveVehicle,
    model_id: str,
    config_sha256: str,
    sha: str,
    data_lines: list[str],
    metrics: dict[str, Any],
) -> Path:
    """Write the bundle for the pinned identity and check that it loads. Refuses an unpinned
    vehicle_params."""
    check_pinned(live)
    write_bundle_dir(out, arrays, live, model_id, config_sha256, sha, data_lines, _clean(metrics))
    load_bundle(out, "track_perception", live)
    return out


def default_model_id_v2(today: dt.date | None = None) -> str:
    return f"trackperc-v2-{(today or dt.datetime.now(dt.timezone.utc).date()).isoformat()}"


def data_lines_v2(
    config: dict[str, Any], selection: dict[str, Any] | None, recipe_sha: str
) -> list[str]:
    lines = data_lines_v1(config, selection)
    lines.append(
        f"scale augmentation of real samples, factor uniform in {config.get('scale_range')} about "
        f"the LiDAR position, probability {config.get('scale_probability')} (pre-registration "
        "v0.3 section 3.1); the scan refinement runs after the net (io.arch.type "
        "circ_conv1d_v0_refine_v1)"
    )
    lines.append(
        "real self-labels include the two 2026-10 bags (role train in the frozen v0.2 split; "
        "ICP ground speed at least 0.2 m/s is their moving test, they have no /odom/wheel)"
    )
    lines.append(f"recipe frozen at git {recipe_sha} before any fold was trained or scored")
    lines.append(
        f"trained on device {config.get('device', 'cpu')} (torch {config.get('torch')}); the "
        "device is how the recipe was run and not part of it"
    )
    return lines


def _variant_lines(name: str, v: dict[str, Any]) -> list[str]:
    lines = [
        f"### {name}: {v['outcome']}",
        "",
        "| item | result | note |",
        "|---|---|---|",
    ]
    for item in v["items"]:
        lines.append(
            f"| {item['number']}. {item['name']} | {'pass' if item['passed'] else 'FAIL'} "
            f"| {item['note']} |"
        )
    lines += [
        "",
        (
            "| set | backend | scans | E_c (m) | V | intrusion | residual median (m) "
            "| deficit p90 (m) | C95 |"
        ),
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for set_name in ("H-run", "A-run", "A-layout-2026-11-12"):
        for backend, n in v[set_name].items():
            lines.append(
                f"| {set_name} | {backend} | {n['scans']} | {_fmt(n.get('e_c_m'))} | "
                f"{_fmt(n.get('v'))} | {_fmt(n.get('intrusion_mean'))} | "
                f"{_fmt(n.get('residual_median_m'))} | {_fmt(n.get('deficit_p90_m'))} | "
                f"{_fmt(n.get('c95'))} |"
            )
    for fold, fr in v["folds"].items():
        for backend in fr["labelled"]:
            lab, free = fr["labelled"][backend], fr["free"][backend]
            lines.append(
                f"| fold {fold} | {backend} | {lab['scans']} labelled / {free['scans']} moving | "
                f"{_fmt(lab['e_c_m'])} | {_fmt(lab['v'])} | {_fmt(free['intrusion_mean'])} | "
                f"{_fmt(free['residual_median_m'])} | {_fmt(free['deficit_p90_m'])} | "
                f"{_fmt(free['c95'])} |"
            )
    return lines + [""]


def readme_v2(model_id: str, metrics: dict[str, Any]) -> str:
    """README of the v2 pointer. The first line is the GP1b verdict (net plus scan refinement)."""
    g = metrics["gp1b"]
    lines = [
        f"GP1b verdict: {g['refined']['outcome']}",
        "",
        f"# {model_id}",
        "",
        "Net plus scan refinement (`io.arch.type: circ_conv1d_v0_refine_v1`): the v1 network with",
        "scale-augmented real samples, followed by `racer_track_perception.refine`. Weights are not",
        "committed (`/Users/cameronjim/code/car-data/pnp/models/`). Decision rule:",
        "`docs/notes/preregistration-pnp-v0.3.md` section 3.3, constants",
        "`evaluation/prereg/pnp-v0.3.yaml`; thresholds are those of v0.2 section 2.",
        f"Recipe frozen at git `{metrics.get('recipe_sha')}`. Split SHA-256 `{g['split_sha256']}`.",
        f"Moving scans: {g['moving_rule']}; speed source per fold bag: {g['speed_source']}.",
        "",
        "The clearance deficit of the refined output holds BY CONSTRUCTION (per-side clearance cap",
        "in the refinement); the variant without the cap and the unrefined net are printed next to",
        "it. The decision below is the registered one, applied as written.",
        "",
    ]
    lines += _variant_lines("refined (candidate)", g["refined"])
    lines += _variant_lines("refined, without the clearance cap (report only)", g["refined_no_cap"])
    lines += _variant_lines("unrefined (same weights, report only)", g["unrefined"])
    lat = metrics.get("latency_ms")
    if lat:
        lines += [
            f"Latency on the dev machine, {lat['scans']} real scans, preprocess + net + refinement:",
            f"p50 {_fmt(lat['p50'])} ms, p99 {_fmt(lat['p99'])} ms. The Orin budget is p99 under 10 ms",
            f"scan to message; {lat['proxy_note']}",
            "",
        ]
    lines += ["Everything, as stored in the bundle manifest (`metrics.json` is the same):", ""]
    return "\n".join(lines) + "\n"


def export_v2(
    run: Path,
    model_id: str,
    gp1b_result: dict[str, Any] | None,
    extra: dict[str, Any],
    repo: Path,
    recipe_sha: str,
    selection: dict[str, Any] | None = None,
    models_dir: Path = MODELS_DIR,
) -> Path:
    """The v2 bundle (refine arch type) from a training run; a candidate without `gp1b_result`."""
    import torch
    from racer_track_perception.bundle import expected_io_refine

    from racer_pnp_perception import model

    live = load_live_vehicle(repo)
    check_pinned(live)
    net = model.build_model()
    net.load_state_dict(torch.load(run / "model.pt", map_location="cpu"))
    net.eval()
    arrays = model.to_npz_arrays(net)
    x = np.random.default_rng(0).uniform(0.0, 1.0, size=(8, 2, 360)).astype(np.float32)
    with torch.no_grad():
        want = net(torch.from_numpy(x)).numpy()
    runtime = CircConv1dV0(arrays)
    worst = max(float(np.abs(runtime.forward(x[i : i + 1]) - want[i]).max()) for i in range(8))
    if worst > 1e-5:
        raise RuntimeError(f"numpy runtime differs from torch by {worst}")
    config = json.loads((run / "config.json").read_text())
    metrics: dict[str, Any] = {
        "numpy_vs_torch_max_abs_diff": worst,
        "recipe_sha": recipe_sha,
        **extra,
    }
    if gp1b_result is None:
        metrics["status"] = "candidate: GP1b not yet decided"
    else:
        metrics["gp1b"] = gp1b_result
    out = models_dir / model_id
    out.mkdir(parents=True, exist_ok=True)
    np.savez(out / "weights.npz", **arrays)
    manifest = build_manifest(
        live,
        model_id,
        hashlib.sha256((out / "weights.npz").read_bytes()).hexdigest(),
        hashlib.sha256((run / "config.json").read_bytes()).hexdigest(),
        git_sha(repo),
        data_lines_v2(config, selection, recipe_sha),
        _clean(metrics),
    )
    manifest["io"] = expected_io_refine(live)
    write_manifest(out, manifest)
    bundle = load_bundle(out, "track_perception", live)
    if not bundle.refine:
        raise RuntimeError("the v2 bundle does not declare the refinement")
    return out


def write_pointer_v2(repo: Path, bundle_dir: Path, metrics: dict[str, Any]) -> Path:
    manifest = yaml.safe_load((bundle_dir / "manifest.yaml").read_text())
    model_id = manifest["model_id"]
    out = repo / "training" / "pnp_models" / model_id
    out.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(bundle_dir / "manifest.yaml", out / "manifest.yaml")
    clean = _clean(metrics)
    (out / "metrics.json").write_text(json.dumps(clean, indent=2, allow_nan=False))
    (out / "README.md").write_text(readme_v2(model_id, clean))
    return out


def main_v2(args: argparse.Namespace) -> int:
    repo = find_repo_root()
    result = json.loads(args.gp1b.read_text()) if args.gp1b else None
    extra: dict[str, Any] = {}
    if args.latency_json:
        extra["latency_ms"] = json.loads(args.latency_json.read_text())
    selection_path = args.run.parent / "selection.json"
    selection = json.loads(selection_path.read_text()) if selection_path.is_file() else None
    out = export_v2(
        args.run, args.model_id, result, extra, repo, args.recipe_sha, selection, args.models_dir
    )
    if result is None:
        print(out)
        return 0
    metrics = yaml.safe_load((out / "manifest.yaml").read_text())["metrics"]
    write_pointer_v2(repo, out, metrics)
    print(out)
    return 0


def main_v1(args: argparse.Namespace) -> int:
    repo = find_repo_root()
    gp1_result = json.loads(args.gp1.read_text()) if args.gp1 else None
    extra: dict[str, Any] = {}
    if args.sim_json:
        extra["sim"] = json.loads(args.sim_json.read_text())
        v0 = repo / "training" / "pnp_models" / "trackperc-v0-2026-11-11" / "metrics.json"
        if v0.is_file():  # the sim-only v0 net's numbers, for the comparison in the README
            held = json.loads(v0.read_text()).get("sim_held_out", {})
            extra["sim_v0_held_out_net_raw"] = {k: v["net_raw"] for k, v in held.items()}
    if args.latency_json:
        extra["latency_numpy_forward_ms"] = json.loads(args.latency_json.read_text())
    selection_path = args.run.parent / "selection.json"
    selection = json.loads(selection_path.read_text()) if selection_path.is_file() else None
    out = export_v1(args.run, args.model_id, gp1_result, extra, repo, selection, args.models_dir)
    if gp1_result is None:  # a candidate bundle: no pointer, no fixture
        print(out)
        return 0
    metrics = yaml.safe_load((out / "manifest.yaml").read_text())["metrics"]
    write_pointer_v1(repo, out, metrics)
    if args.copy_release_fixture:
        from racer_pnp_perception import gp1 as gp1_module

        if gp1_result["outcome"] != gp1_module.OUTCOME_PASS:
            raise SystemExit("GP1 did not pass: the release fixture is not replaced")
        copy_release_fixture(repo, out)
    print(out)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--finalize", type=Path, default=None, help="bundle dir to finalize")
    parser.add_argument("--sim-json", type=Path, default=None)
    parser.add_argument("--bags-json", type=Path, default=None)
    parser.add_argument("--run", type=Path, default=None)
    parser.add_argument("--model-id", default=default_model_id())
    parser.add_argument("--metrics", type=Path, default=None, help="json file of numbers")
    parser.add_argument("--data", action="append", default=[])
    parser.add_argument("--no-copy", action="store_true")
    parser.add_argument(
        "--v1", action="store_true", help="export the v1 bundle (see the docstring)"
    )
    parser.add_argument("--gp1", type=Path, default=None)
    parser.add_argument("--v2", action="store_true", help="export the v2 bundle (scan refinement)")
    parser.add_argument("--gp1b", type=Path, default=None)
    parser.add_argument("--recipe-sha", default=None, help="git SHA the recipe was frozen at")
    parser.add_argument("--latency-json", type=Path, default=None)
    parser.add_argument("--copy-release-fixture", action="store_true")
    parser.add_argument("--models-dir", type=Path, default=MODELS_DIR)
    args = parser.parse_args(argv)
    if args.v2:
        if args.run is None or args.recipe_sha is None:
            parser.error("--v2 needs --run and --recipe-sha")
        if args.model_id == default_model_id():
            args.model_id = default_model_id_v2()
        return main_v2(args)
    if args.v1:
        if args.run is None:
            parser.error("--v1 needs --run")
        if args.model_id == default_model_id():
            args.model_id = default_model_id_v1()
        return main_v1(args)
    repo = find_repo_root()
    if args.finalize is not None:
        sim = json.loads(args.sim_json.read_text())
        bags = json.loads(args.bags_json.read_text())
        history_path = args.run / "history.json" if args.run else None
        history = json.loads(history_path.read_text()) if history_path else None
        finalize(args.finalize, compose_metrics(sim, bags, history), repo)
        print(f"finalized {args.finalize}")
        return 0
    if args.run is None:
        parser.error("--run is required unless --finalize is given")
    metrics = json.loads(args.metrics.read_text()) if args.metrics else {}
    out = export(args.run, args.model_id, metrics, args.data, repo)
    if not args.no_copy:
        write_pointer(repo, out, metrics)
        copy_release_fixture(repo, out)
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
