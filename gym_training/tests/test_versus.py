# unit tests for the head-to-head wrapper's pure parts: unwrapped s, spawns, the overtake bonus

import math

import gymnasium as gym
import numpy as np
import pytest

from f1rl.envs.versus import (
    EGO_INDEX,
    OPPONENT_INDEX,
    OvertakeBonus,
    VersusConfig,
    advance_s,
    spawn_is_clear,
    spawn_poses,
)

TRACK_LENGTH_M = 343.32


def test_advance_s_is_a_plain_difference_away_from_the_seam():
    assert advance_s(100.0, 104.5, TRACK_LENGTH_M) == pytest.approx(4.5)
    assert advance_s(104.5, 100.0, TRACK_LENGTH_M) == pytest.approx(-4.5)


def test_advance_s_unwraps_a_forward_crossing():
    assert advance_s(TRACK_LENGTH_M - 2.0, 1.0, TRACK_LENGTH_M) == pytest.approx(3.0)


def test_advance_s_unwraps_a_backward_crossing():
    assert advance_s(1.0, TRACK_LENGTH_M - 2.0, TRACK_LENGTH_M) == pytest.approx(-3.0)


def test_accumulated_advance_stays_monotone_across_the_seam():
    samples = [(TRACK_LENGTH_M - 1.0 + 0.5 * i) % TRACK_LENGTH_M for i in range(20)]
    total = sum(advance_s(a, b, TRACK_LENGTH_M) for a, b in zip(samples, samples[1:]))

    assert total == pytest.approx(0.5 * 19)


class CircleTrack:
    """a unit-speed circular reference line, enough to check spawn composition."""

    def __init__(self, radius_m: float = 50.0):
        self.radius_m = radius_m

    def frenet_to_cartesian(self, s, ey, ephi, use_raceline=False):
        theta = s / self.radius_m
        radius = self.radius_m - ey
        yaw = (theta + 0.5 * math.pi + ephi + math.pi) % (2.0 * math.pi) - math.pi
        return radius * math.cos(theta), radius * math.sin(theta), yaw


def test_spawn_puts_the_ego_one_gap_behind_the_opponent():
    track = CircleTrack()
    poses = spawn_poses(track, opponent_s_m=100.0, gap_m=10.0, ego_ey_m=0.0, opponent_ey_m=0.0)

    assert poses.shape == (2, 3)
    arc = track.radius_m * abs(
        (100.0 / track.radius_m) - math.atan2(poses[EGO_INDEX][1], poses[EGO_INDEX][0])
    )
    assert arc == pytest.approx(10.0, abs=1e-6)


def test_both_cars_spawn_facing_the_track_direction():
    track = CircleTrack()
    poses = spawn_poses(track, opponent_s_m=40.0, gap_m=8.0, ego_ey_m=0.3, opponent_ey_m=-0.3)

    for row in poses:
        heading = np.array([math.cos(row[2]), math.sin(row[2])])
        tangent = np.array([-row[1], row[0]])
        tangent /= np.linalg.norm(tangent)
        assert heading @ tangent == pytest.approx(1.0, abs=1e-6)


def test_lateral_jitter_moves_the_cars_off_the_line():
    track = CircleTrack()
    poses = spawn_poses(track, opponent_s_m=0.0, gap_m=5.0, ego_ey_m=0.0, opponent_ey_m=0.4)

    assert np.linalg.norm(poses[OPPONENT_INDEX][:2]) == pytest.approx(track.radius_m - 0.4)


def test_a_spawn_with_room_on_every_car_is_clear():
    assert spawn_is_clear([np.full(12, 3.0), np.full(12, 0.9)], 0.21)


def test_a_car_placed_inside_the_margin_fails_the_spawn_check():
    pinned = np.full(12, 3.0)
    pinned[4] = 0.02

    assert not spawn_is_clear([np.full(12, 3.0), pinned], 0.21)


@pytest.mark.slow
def test_spawn_gap_survives_a_frenet_round_trip_on_spielberg():
    from f1tenth_gym.envs.track import Track

    track = Track.from_track_name("Spielberg")
    length = track.raceline.spline.s_frame_max
    for opponent_s in (5.0, 90.0, 200.0, length - 3.0):
        poses = spawn_poses(track, opponent_s, gap_m=12.0, ego_ey_m=0.0, opponent_ey_m=0.0)
        back = [
            track.cartesian_to_frenet(*row, use_raceline=True, use_s_guess=False)[0] for row in poses
        ]
        assert advance_s(back[EGO_INDEX], back[OPPONENT_INDEX], length) == pytest.approx(
            12.0, abs=0.05
        )


class GapEnv(gym.Env):
    """replays a scripted sequence of ego-minus-opponent gaps."""

    def __init__(self, gaps):
        self.gaps = list(gaps)
        self.step_index = 0

    def reset(self, *, seed=None, options=None):
        self.step_index = 0
        return np.zeros(1, dtype=np.float32), {"gap_m": self.gaps[0], "sim_time": 0.0}

    def step(self, action):
        self.step_index += 1
        gap = self.gaps[min(self.step_index, len(self.gaps) - 1)]
        info = {"gap_m": gap, "sim_time": 0.1 * self.step_index}
        done = self.step_index >= len(self.gaps) - 1
        return np.zeros(1, dtype=np.float32), 0.0, done, False, info


def run_gaps(gaps, bonus=20.0, margin_m=1.6):
    env = OvertakeBonus(GapEnv(gaps), bonus=bonus, margin_m=margin_m)
    env.reset()
    rewards, infos = [], []
    for _ in range(len(gaps) - 1):
        _, reward, _, _, info = env.step(np.zeros(2))
        rewards.append(reward)
        infos.append(info)
    return rewards, infos


def test_the_bonus_fires_once_when_the_lead_clears_the_margin():
    rewards, infos = run_gaps([-5.0, -2.0, 0.5, 1.7, 3.0, 6.0])

    assert rewards == [0.0, 0.0, 20.0, 0.0, 0.0]
    assert [info["overtaken"] for info in infos] == [False, False, True, True, True]


def test_falling_back_behind_does_not_refire_the_bonus():
    rewards, _ = run_gaps([-5.0, 2.0, -3.0, 2.5])

    assert rewards == [20.0, 0.0, 0.0]


def test_a_lead_short_of_the_margin_pays_nothing():
    rewards, infos = run_gaps([-5.0, 0.5, 1.2, 1.59])

    assert rewards == [0.0, 0.0, 0.0]
    assert not any(info["overtaken"] for info in infos)


def test_an_action_repeat_that_skips_the_crossing_still_fires():
    # the level test means a gap that jumps straight past the margin is not missed
    rewards, _ = run_gaps([-5.0, 40.0])

    assert rewards == [20.0]


def test_the_bonus_records_when_the_pass_happened():
    _, infos = run_gaps([-5.0, -1.0, 4.0, 5.0])

    assert infos[1]["overtake_time_sec"] == pytest.approx(0.2)
    assert "overtake_time_sec" not in infos[2]


def test_reset_clears_the_latch():
    env = OvertakeBonus(GapEnv([-5.0, 4.0, 5.0]), bonus=20.0, margin_m=1.6)
    env.reset()
    assert env.step(np.zeros(2))[1] == 20.0
    env.reset()
    assert env.step(np.zeros(2))[1] == 20.0


def test_versus_config_rejects_keys_it_would_otherwise_ignore():
    with pytest.raises(ValueError, match="unknown versus config keys"):
        VersusConfig.from_dict({"gap_min_m": 4.0, "gap_maks_m": 9.0})


def test_versus_config_rejects_an_inverted_spawn_gap():
    with pytest.raises(ValueError, match="gap_min_m <= gap_max_m"):
        VersusConfig(gap_min_m=12.0, gap_max_m=6.0)


def test_versus_config_rejects_an_unknown_spawn_line():
    with pytest.raises(ValueError, match="spawn_line"):
        VersusConfig(spawn_line="apex")
