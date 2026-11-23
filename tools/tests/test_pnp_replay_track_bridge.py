"""L4 replay of the one frame track bridge (fix checklist 2026-11-22, item S2) on the real frames.

On 2026-11-22 the planner braked to zero for one to six ticks (BLOCKED, below the speed floor) 11
times in about 53 laps; before 10 of them one /pnp/local_track frame jumped sideways and narrowed,
and the next frame was back to normal. tests/fixtures/pnp_bridge_2026-11-22.npz holds three of
those moments and one clean stretch, cut from bag 2026-11-23T03-50-32_car_teleop
(`python -m pnp_replay.cut_fixture <bag> --lenient --bridge-fixture`).

Replayed open loop through planner_replay_cli with the recorded tracks, the recorded tick times
and the fitted car bundle: without the bridge (`track_bridge=0`) the replay brakes where the car
did; with it the planner stays DRIVING at no less than half the cap, and on the clean stretch the
bridge changes nothing at all. Needs planner_replay_cli (skipped otherwise, a failure in CI where
.github/scripts/pnp_replay_test.sh sets PNP_REPLAY_REQUIRE_CLI=1).
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest
from pnp_replay import cut_fixture, planner_stream

TESTS = Path(__file__).resolve().parent
REPO_ROOT = TESTS.parents[1]
FIXTURE = TESTS / "fixtures" / "pnp_bridge_2026-11-22.npz"
CAR_BUNDLE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-car-2026-11-12"
V_MAX = 1.0  # the observe runs' cap; the D v0 run was at 0.9 by guidance, which a replay lacks
# Open loop (the recorded wheel does not follow the replay) and deterministic (no halving of the
# samples on a slow CI machine).
PARAMS: dict[str, float | bool] = {"stall_hold_s": 600.0, "solve_halve_over_ms": 200.0}
DRIVING, BLOCKED, BELOW_SPEED_FLOOR = 1, 5, 2
# Cut -> the bag time (s) of the car's blip. The outlier frame arrived 0.05 to 0.08 s before it.
BLIPS = {"B6266": 430.35, "B7079": 1078.04, "B7179": 1342.65}
# Cuts where the replay without the bridge brakes as the car did (B6266 replays to 0.48 of the
# cap instead: open loop at 512 samples it stops short of the floor).
REPRODUCED = ("B7079", "B7179")
NEAR_S = 0.3


def _cli() -> planner_stream.Cli:
    try:
        return planner_stream.find_cli()
    except planner_stream.ReplayError as error:
        if os.environ.get("PNP_REPLAY_REQUIRE_CLI"):
            pytest.fail(f"planner_replay_cli is required here but not built: {error}")
        pytest.skip(f"planner_replay_cli not built ({error})")


def replay(name: str, tmp_path: Path, **extra: float | bool) -> dict[str, np.ndarray]:
    cli = _cli()
    rec, a, b = cut_fixture.load_cut(FIXTURE, name)
    lines = planner_stream.stream_lines(rec, a, b, tracks_mode="recorded", tick_times=rec["st_t"])
    tag = f"{name}-{len(list(tmp_path.iterdir()))}"
    stream = tmp_path / f"{tag}.stream"
    planner_stream.write_stream(lines, stream)
    return planner_stream.run_cli(
        cli, CAR_BUNDLE, V_MAX, {**PARAMS, **extra}, stream, tmp_path / f"{tag}.csv"
    )


def near(rows: dict[str, np.ndarray], t: float) -> np.ndarray:
    return (rows["now_s"] > t - NEAR_S) & (rows["now_s"] < t + NEAR_S)


def test_fixture_is_small_and_has_the_cuts() -> None:
    assert FIXTURE.stat().st_size < cut_fixture.BRIDGE_MAX_BYTES
    for name, (t_from, t_to) in cut_fixture.BRIDGE_CUTS.items():
        rec, a, b = cut_fixture.load_cut(FIXTURE, name)
        assert (a, b) == (t_from, t_to)
        assert len(rec["scan_t"]) > 25 and len(rec["trk_t"]) > 25 and len(rec["st_t"]) > 100
    for name, t in BLIPS.items():
        a, b = cut_fixture.BRIDGE_CUTS[name]
        assert a + 1.5 < t < b - NEAR_S  # warm start done, the blip and what follows inside


@pytest.mark.parametrize("name", REPRODUCED)
def test_without_the_bridge_the_replay_brakes_where_the_car_did(name: str, tmp_path: Path) -> None:
    rows = replay(name, tmp_path, track_bridge=False)
    m = near(rows, BLIPS[name])
    blocked = m & (rows["state"] == BLOCKED) & (rows["block_reason"] == BELOW_SPEED_FLOOR)
    assert blocked.any(), "the fixture no longer shows the failure the bridge is for"
    assert rows["cmd_speed_mps"][m].min() == 0.0
    assert not rows["track_bridged"].any()


@pytest.mark.parametrize("name", sorted(BLIPS))
def test_with_the_bridge_the_planner_keeps_driving(name: str, tmp_path: Path) -> None:
    rows = replay(name, tmp_path)
    t = BLIPS[name]
    m = near(rows, t)
    assert m.sum() > 20
    assert (rows["state"][m] == DRIVING).all()
    assert (rows["cmd_speed_mps"][m] >= 0.5 * rows["v_cap_mps"][m]).all()
    # the outlier frame (0.05 to 0.08 s before the blip) is what was bridged
    outlier = (rows["now_s"] > t - 0.15) & (rows["now_s"] < t + 0.02)
    assert rows["track_bridged"][outlier].any()
    # a bridge lasts one track frame: never more than 0.2 s of consecutive bridged ticks
    run, longest, start = 0, 0.0, 0.0
    for now, bridged in zip(rows["now_s"], rows["track_bridged"], strict=True):
        if bridged:
            start = now if run == 0 else start
            run += 1
            longest = max(longest, now - start)
        else:
            run = 0
    assert longest < 0.2


def test_on_a_clean_stretch_the_bridge_changes_nothing(tmp_path: Path) -> None:
    on = replay("CLEAN", tmp_path)
    off = replay("CLEAN", tmp_path, track_bridge=False)
    assert not on["track_bridged"].any()
    assert (on["state"] == DRIVING).sum() > 100
    for key, value in on.items():
        if key == "solve_time_s":
            continue
        if value.dtype.kind == "f":
            np.testing.assert_array_equal(value, off[key], err_msg=key)
        else:
            assert np.array_equal(value, off[key]), key
