"""A randomised family of sim tracks for the auto-tune harness.

`sample_track(seed)` maps a seed to one track (kind, lane width, straight length, island size),
deterministically and without touching global random state. Tuning and held-out seeds are two
disjoint ranges, and `held_out_tracks` additionally redraws any seed whose track is identical to
a tuning track, so a held-out result is never a rerun of something the search saw. Every track
runs both directions (the runner's --reverse), and the seed is also the bridge's seed.

The ranges follow what the owner builds (docs/notes/track-session-2026-11-11.md): lanes of about
1 m (0.85 to 1.2), straights of 1 to 3 m, ends from a thin wall island (radius 0, the car's
turning radius does not fit the lane's middle) to round ends, and rounded rectangles. Lane width
and island radius are drawn from short lists of values, not continuously, so two seeds can give
the same track and the de-duplication above has something to do.
"""

from __future__ import annotations

import random
from dataclasses import dataclass

TUNING_SEEDS = tuple(range(1, 9))  # 8 tracks
HELD_OUT_SEEDS = tuple(range(1001, 1013))  # 12 tracks
DIRECTIONS = ("ccw", "cw")

_LANES = (0.85, 0.9, 0.95, 1.0, 1.0, 1.0, 1.05, 1.1, 1.2)
_STADIUM_STRAIGHTS = (1.0, 1.5, 2.0, 2.0, 3.0)
_STADIUM_RADII = (0.0, 0.0, 0.2, 0.4, 0.6)
_RR_STRAIGHTS = (2.0, 3.0)
_RR_WIDTHS = (1.5, 2.0)
_RR_RADII = (0.0, 0.2, 0.4)


@dataclass(frozen=True)
class TrackSpec:
    name: str
    track: str  # the bridge's synthetic track kind
    params: tuple[tuple[str, float], ...]  # sorted (key, value), hashable
    seed: int

    def params_dict(self) -> dict[str, float]:
        return dict(self.params)

    def key(self) -> tuple[str, tuple[tuple[str, float], ...]]:
        """Identity of the geometry, ignoring the seed."""
        return (self.track, self.params)


def sample_track(seed: int) -> TrackSpec:
    rng = random.Random(f"race-track-{seed}")
    lane = rng.choice(_LANES)
    if rng.random() < 0.65:
        straight = rng.choice(_STADIUM_STRAIGHTS)
        radius = rng.choice(_STADIUM_RADII)
        params = {"lane_width_m": lane, "straight_length_m": straight, "island_radius_m": radius}
        name = f"st_w{lane}_s{straight}_r{radius}"
        kind = "stadium"
    else:
        straight = rng.choice(_RR_STRAIGHTS)
        width = rng.choice(_RR_WIDTHS)
        radius = rng.choice(_RR_RADII)
        params = {
            "lane_width_m": lane,
            "straight_length_m": straight,
            "island_width_m": width,
            "island_radius_m": radius,
        }
        name = f"rr_w{lane}_{straight}x{width}_r{radius}"
        kind = "rounded_rect"
    return TrackSpec(name, kind, tuple(sorted(params.items())), seed)


def tuning_tracks() -> list[TrackSpec]:
    """The tracks the search is allowed to see (de-duplicated by geometry, first seed kept)."""
    seen: set = set()
    tracks = []
    for seed in TUNING_SEEDS:
        track = sample_track(seed)
        if track.key() not in seen:
            seen.add(track.key())
            tracks.append(track)
    return tracks


def held_out_tracks(count: int | None = None) -> list[TrackSpec]:
    """Held-out tracks: seeds from HELD_OUT_SEEDS (then further seeds above them when a draw
    repeats a geometry), none of which equals a tuning track or another held-out track."""
    seen = {t.key() for t in [sample_track(s) for s in TUNING_SEEDS]}
    wanted = len(HELD_OUT_SEEDS) if count is None else count
    tracks: list[TrackSpec] = []
    seed = HELD_OUT_SEEDS[0]
    while len(tracks) < wanted:
        track = sample_track(seed)
        seed += 1
        if track.key() in seen:
            continue
        seen.add(track.key())
        tracks.append(track)
    return tracks


# -- The WIDE family (2026-11-12 race bag) ----------------------------------------------------------
# The owner rebuilt the oval for the race: lanes of about 1.3 to 1.5 m, not the 0.85 to 1.2 m above.
# docs/notes/safety-race-retune-2026-11-12.md retunes for it, so it gets its own seed ranges, tuning
# and held-out disjoint like the narrow family. Same bridge track kinds, only wider lanes and a
# little more straight.

WIDE_TUNING_SEEDS = tuple(range(2001, 2009))  # 8 draws, de-duplicated below
WIDE_HELD_OUT_SEEDS = tuple(range(3001, 3017))

_WIDE_LANES = (1.3, 1.4, 1.5)
_WIDE_STRAIGHTS = (1.5, 2.0, 3.0)
_WIDE_RADII = (0.0, 0.2, 0.4, 0.6)


def sample_wide_track(seed: int) -> TrackSpec:
    """A stadium of the wide family for a seed (deterministic, no global random state)."""
    rng = random.Random(f"race-wide-track-{seed}")
    lane = rng.choice(_WIDE_LANES)
    straight = rng.choice(_WIDE_STRAIGHTS)
    radius = rng.choice(_WIDE_RADII)
    params = {"lane_width_m": lane, "straight_length_m": straight, "island_radius_m": radius}
    return TrackSpec(
        f"wst_w{lane}_s{straight}_r{radius}", "stadium", tuple(sorted(params.items())), seed
    )


def wide_tuning_tracks(count: int | None = None) -> list[TrackSpec]:
    """The wide tracks the search may see (de-duplicated by geometry, first seed kept)."""
    seen: set = set()
    out: list[TrackSpec] = []
    for seed in WIDE_TUNING_SEEDS:
        track = sample_wide_track(seed)
        if track.key() not in seen:
            seen.add(track.key())
            out.append(track)
    return out if count is None else out[:count]


def wide_held_out_tracks(count: int | None = None) -> list[TrackSpec]:
    """Wide held-out tracks: none equals a wide tuning track or another held-out track."""
    seen = {sample_wide_track(s).key() for s in WIDE_TUNING_SEEDS}
    wanted = len(WIDE_HELD_OUT_SEEDS) if count is None else count
    out: list[TrackSpec] = []
    seed = WIDE_HELD_OUT_SEEDS[0]
    while len(out) < wanted and seed < WIDE_HELD_OUT_SEEDS[0] + 1000:
        track = sample_wide_track(seed)
        seed += 1
        if track.key() in seen:
            continue
        seen.add(track.key())
        out.append(track)
    return out


def family_tracks(family: str, tuning: bool, count: int | None = None) -> list[TrackSpec]:
    """The tracks of a family ("narrow" is the original, "wide" the 1.3 to 1.5 m lanes)."""
    if family == "wide":
        return wide_tuning_tracks(count) if tuning else wide_held_out_tracks(count)
    if family == "narrow":
        return tuning_tracks() if tuning else held_out_tracks(count)
    raise ValueError(f"unknown track family {family!r}")
