"""The race profile's parameter space and the seeded sampler.

A candidate is a dict of gap_follow_node parameters on top of the lane-2026-10-25 profile. BASE is
what every candidate shares (the physics speed law on, the speed low-pass off because it would
delay the stopping limit), SPACE is what the search varies, each parameter over a short list of
values the owner could defend on the car. Nothing here is a vehicle constant: the values are
tuning parameters (CLAUDE.md invariant 2).

`max_speed_mps` stops at 1.0 m/s on purpose: the floor profile has been driven at 0.64 m/s, the
sim at 0.9, nothing above that, so 1.0 is the ceiling of what this PR proposes to try first.
"""

from __future__ import annotations

import random
from typing import Any

# The recommended profile before this work (docs/notes/track-session-2026-11-11.md): the lane
# profile, lookahead 0.8, outside entry 0.25, top speed 0.64.
BASELINE: dict[str, Any] = {
    "max_speed_mps": 0.64,
    "lane_lookahead_m": 0.8,
    "outside_entry_offset_m": 0.25,
}

BASE: dict[str, Any] = {
    "physics_speed": True,
    # The stopping limit must act at once: a low-pass on the speed target would delay it. The
    # acceleration limiter still bounds what is published.
    "speed_time_constant_s": 0.0,
}

SPACE: dict[str, list[Any]] = {
    "max_speed_mps": [0.64, 0.8, 1.0],
    "a_lat_max_mps2": [1.0, 1.5, 2.0, 3.0],
    "lane_lookahead_m": [0.6, 0.8, 1.0],
    "outside_entry_offset_m": [0.0, 0.15, 0.25],
    "apex_offset_m": [0.0, 0.1, 0.2],
    "track_out_offset_m": [0.0, 0.1, 0.2],
    "side_cap_distance_m": [0.0, 0.1, 0.2],
    "stopping_margin_m": [0.05, 0.1, 0.2],
    "speed_rate_limit_margin_fraction": [0.1, 0.25, 0.5],
}


def with_base(params: dict[str, Any]) -> dict[str, Any]:
    return {**BASE, **params}


def sample_candidates(count: int, seed: int) -> list[dict[str, Any]]:
    """`count` distinct candidates drawn uniformly from SPACE, in a fixed order for a seed. The
    first is always the middle of the space with every optional feature off (the physics law
    alone), so the search always includes that reference point."""
    rng = random.Random(f"race-space-{seed}")
    first = {
        "max_speed_mps": 0.8,
        "a_lat_max_mps2": 2.0,
        "lane_lookahead_m": 0.8,
        "outside_entry_offset_m": 0.25,
        "apex_offset_m": 0.0,
        "track_out_offset_m": 0.0,
        "side_cap_distance_m": 0.0,
        "stopping_margin_m": 0.05,
        "speed_rate_limit_margin_fraction": 0.25,
    }
    seen = {tuple(sorted(first.items()))}
    out = [first]
    size = 1
    for values in SPACE.values():
        size *= len(values)
    while len(out) < min(count, size):
        cand = {key: rng.choice(values) for key, values in SPACE.items()}
        k = tuple(sorted(cand.items()))
        if k in seen:
            continue
        seen.add(k)
        out.append(cand)
    return out


def neighbours(params: dict[str, Any]) -> list[dict[str, Any]]:
    """Every candidate that differs from `params` in one parameter, moved one step along that
    parameter's list (a value not in the list is skipped)."""
    out = []
    for key, values in SPACE.items():
        if params.get(key) not in values:
            continue
        i = values.index(params[key])
        for j in (i - 1, i + 1):
            if 0 <= j < len(values):
                out.append({**params, key: values[j]})
    return out


def candidate_id(params: dict[str, Any]) -> str:
    """A short stable id for a candidate (also a file name): the parameters that differ from
    nothing, in SPACE order, e.g. v0.8_a2.0_la0.8_oe0.25_ap0_to0_sc0_sm0.05_rl0.25."""
    short = {
        "max_speed_mps": "v",
        "a_lat_max_mps2": "a",
        "lane_lookahead_m": "la",
        "outside_entry_offset_m": "oe",
        "apex_offset_m": "ap",
        "track_out_offset_m": "to",
        "side_cap_distance_m": "sc",
        "stopping_margin_m": "sm",
        "speed_rate_limit_margin_fraction": "rl",
    }
    return "_".join(f"{short[key]}{params[key]:g}" for key in SPACE if key in params)
