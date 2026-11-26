"""pnp.launch.py profiles in the sim harness, and the system measures of a profile run.

`profile_argv` turns a launch profile (pnp.launch.py PROFILES, e.g. `prototype`) into
track_family_runner.py arguments, so a sim run uses the same defaults as
`ros2 launch racer_bringup pnp.launch.py profile:=prototype` on the car. The runner puts these
in front of its own command line, so an argument given there still wins, as on the car.

Two launch arguments have no sim counterpart and are mapped:
  lap_source camera   -> the simulator's lap oracle (`--lap-source truth`, brief 51), which also
                         speaks the camera's freshness keys; it needs `--sim-time`, added here
  lap_counter, lap_rate_hz -> the camera counter's own settings; there is no camera in sim, the
                         oracle stands in for it, so they are recorded but not applied
A key this module does not know raises, so a profile change cannot pass the harness silently.

`ProfileStats` (ROS free) collects, per run: planner solve time and sample count (the sample
budget halves when solves are slow), perception valid station count (V) per frame, and the
perception node's own 1 Hz status. `node_log_summary` counts errors and warnings per node log.
"""

from __future__ import annotations

import ast
import json
import math
import re
from collections import Counter
from pathlib import Path

#: The bundle directory the launch profiles name on the car (inside the stack container).
CAR_MODELS_DIR = "/workspace/data/pnp/models"
#: Profile keys that are the camera lap counter's own settings: no sim counterpart.
NOT_APPLIED_IN_SIM = ("lap_counter", "lap_rate_hz")
#: A frame with fewer valid stations than this gives the planner no usable track (it needs 2).
V_USABLE_MIN = 2


def load_profiles(launch_file: Path) -> dict[str, dict[str, str]]:
    """PROFILES of pnp.launch.py, read without importing it (no ROS needed)."""
    tree = ast.parse(Path(launch_file).read_text(encoding="utf-8"))
    names: dict[str, str] = {}
    for node in tree.body:
        target = None
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target, value = node.targets[0], node.value
        elif isinstance(node, ast.AnnAssign):
            target, value = node.target, node.value
        if not isinstance(target, ast.Name) or value is None:
            continue
        if isinstance(value, ast.Constant) and isinstance(value.value, str):
            names[target.id] = value.value
        elif target.id == "PROFILES":
            return _literal(value, names)
    raise ValueError(f"no PROFILES in {launch_file}")


def _literal(node: ast.AST, names: dict[str, str]):
    if isinstance(node, ast.Dict):
        return {_literal(k, names): _literal(v, names) for k, v in zip(node.keys, node.values)}
    if isinstance(node, ast.Name) and node.id in names:
        return names[node.id]
    return ast.literal_eval(node)


def _bundle(path: str, models_dir: str) -> str:
    if path.startswith(CAR_MODELS_DIR + "/"):
        return str(Path(models_dir) / path[len(CAR_MODELS_DIR) + 1 :])
    return path


def _flag(name: str, value: str) -> str:
    v = value.strip().lower()
    if v not in ("true", "false"):
        raise ValueError(f"profile {name} must be true or false, got {value!r}")
    return f"--{name}" if v == "true" else f"--no-{name}"


def profile_argv(profile: dict[str, str], models_dir: str) -> tuple[list[str], dict]:
    """(runner arguments, notes) for a launch profile. Bundle paths under the car's model
    directory are moved to `models_dir`. Raises on a key it does not know."""
    p = dict(profile)
    argv: list[str] = []
    notes: dict = {"not_applied": {}, "mapped": {}}
    for key in NOT_APPLIED_IN_SIM:
        if key in p:
            notes["not_applied"][key] = p.pop(key)
    simple = {
        "backend": "--backend",
        "v_max_mps": "--v-max-mps",
        "v_lap1_mps": "--v-lap1-mps",
        "dv1_variant": "--dv1-variant",
    }
    for key, opt in simple.items():
        if key in p:
            argv += [opt, p.pop(key)]
    for key in ("perception_bundle", "dynamics_bundle"):
        if key in p:
            argv += ["--" + key.replace("_", "-"), _bundle(p.pop(key), models_dir)]
    for key in ("dynamics_fast_mlp", "dynamics_adaptation", "dynamics_mlp_float32"):
        if key in p:
            argv.append(_flag(key.replace("_", "-"), p.pop(key)))
    if "lap_length_ratio_min" in p or "lap_length_ratio_max" in p:
        argv += [
            "--lap-length-ratio",
            p.pop("lap_length_ratio_min", "0.0"),
            p.pop("lap_length_ratio_max", "0.0"),
        ]
    lap_memory = p.pop("lap_memory", None)
    lap_mode = p.pop("lap_mode", "v0")
    if lap_memory is not None:
        if lap_memory.strip().lower() == "false":
            argv += ["--lap-memory", "none"]
        else:
            argv += ["--lap-memory", {"v0": "v0", "v1": "v1", "observe": "off"}[lap_mode]]
    if "lap_source" in p:
        source = p.pop("lap_source")
        if source == "camera":
            notes["mapped"]["lap_source"] = "camera -> truth (the simulator's lap oracle)"
            argv += ["--lap-source", "truth", "--sim-time"]
        else:
            argv += ["--lap-source", source]
    if p:
        raise ValueError(f"profile keys the sim harness does not know: {sorted(p)}")
    return argv, notes


def expand_profile_argv(argv: list[str], profiles: dict[str, dict[str, str]]) -> tuple[list, dict]:
    """The runner's command line with the chosen `--pnp-profile` in front (explicit arguments,
    which come after, win). Returns (argv, notes); unchanged argv for profile none."""
    name, models_dir = "none", CAR_MODELS_DIR
    for i, a in enumerate(argv):
        if a == "--pnp-profile" and i + 1 < len(argv):
            name = argv[i + 1]
        elif a.startswith("--pnp-profile="):
            name = a.split("=", 1)[1]
        if a == "--pnp-models-dir" and i + 1 < len(argv):
            models_dir = argv[i + 1]
        elif a.startswith("--pnp-models-dir="):
            models_dir = a.split("=", 1)[1]
    if name == "none":
        return list(argv), {}
    if name not in profiles:
        raise ValueError(f"--pnp-profile must be one of {sorted(profiles)}, got {name!r}")
    front, notes = profile_argv(profiles[name], models_dir)
    notes["profile"] = name
    notes["argv"] = front
    return front + list(argv), notes


def _pct(values: list[float], q: float) -> float | None:
    if not values:
        return None
    s = sorted(values)
    k = (len(s) - 1) * q / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


class ProfileStats:
    """Planner solve time and samples, perception V per frame and status, for one run."""

    def __init__(self) -> None:
        self.solve_ms: list[float] = []
        self.samples: list[int] = []
        self.valid: list[int] = []
        self.track_models: Counter = Counter()
        self.planner_models: Counter = Counter()
        self.perception_status: dict = {}
        self.perception_status_count = 0
        self._driving = False

    def on_planner(self, state: int, solve_time_s: float, samples: int, dyn_id: str, perc_id: str):
        if state == 1:
            self._driving = True
        if not self._driving:
            return
        if math.isfinite(solve_time_s) and solve_time_s > 0.0:
            self.solve_ms.append(solve_time_s * 1000.0)
        if samples > 0:
            self.samples.append(int(samples))
        self.planner_models[(dyn_id, perc_id)] += 1

    def on_track(self, valid_count: int, model_id: str) -> None:
        if not self._driving:
            return
        self.valid.append(int(valid_count))
        self.track_models[model_id] += 1

    def on_perception_status(self, data: str) -> None:
        try:
            self.perception_status = json.loads(data)
        except ValueError:
            self.perception_status = {"unparsed": data[:200]}
        self.perception_status_count += 1

    def summary(self) -> dict:
        top = max(self.samples) if self.samples else 0
        cut = [s for s in self.samples if s < top]
        v = self.valid
        runs, longest, run = 0, 0, 0
        for x in v:
            if x < V_USABLE_MIN:
                run += 1
                if run == 1:
                    runs += 1
                longest = max(longest, run)
            else:
                run = 0
        return {
            "solve_ms": {
                "n": len(self.solve_ms),
                "p50": _round(_pct(self.solve_ms, 50)),
                "p99": _round(_pct(self.solve_ms, 99)),
                "max": _round(max(self.solve_ms) if self.solve_ms else None),
                "over_15ms": sum(1 for x in self.solve_ms if x > 15.0),
            },
            "samples": {
                "max": top,
                "min": min(self.samples) if self.samples else 0,
                "ticks_below_max": len(cut),
                "share_below_max": _round(len(cut) / len(self.samples) if self.samples else 0.0),
            },
            "valid_count": {
                "frames": len(v),
                "mean": _round(sum(v) / len(v) if v else None),
                "zero": sum(1 for x in v if x == 0),
                "unusable": sum(1 for x in v if x < V_USABLE_MIN),
                "unusable_runs": runs,
                "unusable_longest_frames": longest,
                "below_8": sum(1 for x in v if x < 8),
                "histogram": {str(k): c for k, c in sorted(Counter(v).items())},
            },
            "track_model_ids": dict(self.track_models),
            "planner_model_ids": {f"{a}|{b}": c for (a, b), c in self.planner_models.items()},
            "perception_status": self.perception_status,
            "perception_status_count": self.perception_status_count,
        }


def _round(x: float | None, nd: int = 4) -> float | None:
    return None if x is None else round(float(x), nd)


_LEVEL = re.compile(r"\[(ERROR|WARN|FATAL)\]")
_SAMPLES = re.compile(r"sample count (\d+) -> (\d+)")


def node_log_summary(work: Path, names: list[str]) -> dict:
    """Per node log in `work`: ERROR / WARN / FATAL line counts, Python tracebacks, the planner's
    sample count changes and the first few error lines."""
    out: dict = {}
    for name in names:
        path = Path(work) / f"{name}.log"
        if not path.exists():
            continue
        counts: Counter = Counter()
        first: list[str] = []
        halvings = 0
        for line in path.read_text(errors="replace").splitlines():
            m = _LEVEL.search(line)
            if m:
                counts[m.group(1)] += 1
                if m.group(1) != "WARN" and len(first) < 5:
                    first.append(line[-300:])
            if "Traceback (most recent call last)" in line:
                counts["traceback"] += 1
            s = _SAMPLES.search(line)
            if s and int(s.group(2)) < int(s.group(1)):
                halvings += 1
        out[name] = {**dict(counts), "sample_cuts": halvings, "first_errors": first}
    return out
