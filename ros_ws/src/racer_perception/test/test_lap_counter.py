"""L1 tests for racer_perception.lap_counter (no ROS, no OpenCV, numpy only).

Synthetic "driving": a looping panorama (a smooth random texture that wraps around) and a camera
that slides along it. A frame at position p is the 64-pixel-wide window of the panorama starting
at p, so frames one lap apart (p differs by the panorama length) are identical and frames a few
pixels apart are similar, the way two views a few centimetres apart are.
"""

from __future__ import annotations

import json
import math

import numpy as np
import pytest
from racer_perception.lap_counter import (
    STATE_RUNNING,
    STATE_WAITING,
    LapConfig,
    LapCounter,
    blur3,
    make_signature,
    resize_area,
    unit_signature,
)

VIEW_W, VIEW_H = 64, 48
LOOP_PX = 400  # panorama length: one lap is 400 px of camera travel
FPS = 15.0
PX_PER_FRAME = 4  # 60 px/s: a lap takes 400 / 60 = 6.67 s
LAP_S = LOOP_PX / PX_PER_FRAME / FPS


def make_world(seed: int = 0, length: int = LOOP_PX) -> np.ndarray:
    """Wrap-around texture, correlated over about 12 px horizontally and 6 px vertically."""
    rng = np.random.default_rng(seed)
    w = rng.normal(size=(VIEW_H, length)).astype(np.float32)
    kx = np.ones(13, dtype=np.float32) / 13
    ky = np.ones(7, dtype=np.float32) / 7
    wide = np.concatenate([w, w, w], axis=1)
    wide = np.apply_along_axis(lambda r: np.convolve(r, kx, mode="same"), 1, wide)
    wide = np.apply_along_axis(lambda c: np.convolve(c, ky, mode="same"), 0, wide)
    out = wide[:, length : 2 * length]
    out = (out - out.min()) / (out.max() - out.min())
    return (out * 200.0 + 20.0).astype(np.float32)


def view(world: np.ndarray, pos: float, width: int = VIEW_W) -> np.ndarray:
    cols = (round(pos) + np.arange(width)) % world.shape[1]
    return world[:, cols]


def fast_cfg(**kw) -> LapConfig:
    base = {"min_lap_time_s": 3.0, "min_lap_distance_m": 1.0, "start_speed_mps": 0.3}
    base.update(kw)
    return LapConfig(**base)


class Drive:
    """Feeds a LapCounter the way the node does: odometry at 50 Hz, frames at 15 Hz."""

    def __init__(self, counter: LapCounter, world=None, speed_mps: float = 1.0, seed: int = 1):
        self.c = counter
        self.world = make_world() if world is None else world
        self.speed = speed_mps
        self.t = 0.0
        self.pos = 0.0
        self.x = 0.0
        self.rng = np.random.default_rng(seed)
        self.events = []
        self.sims: list[float] = []

    def step(self, dt: float = 1.0 / FPS, moving: bool = True, frame=None):
        """Advance dt seconds: 50 Hz odometry for the interval, then one frame."""
        speed = self.speed if moving else 0.0
        n = max(round(dt * 50), 1)
        for _ in range(n):
            self.t += dt / n
            self.x += speed * dt / n
            self.c.update_odom(self.t, speed, self.x)
        if moving:
            self.pos += PX_PER_FRAME * dt * FPS
        img = view(self.world, self.pos) if frame is None else frame
        event = self.c.update_frame(self.t, img)
        if self.c.similarity is not None:
            self.sims.append(self.c.similarity)
        if event is not None:
            self.events.append(event)
        return event

    def run_laps(self, laps: float, **kw):
        frames = round(laps * LOOP_PX / PX_PER_FRAME)
        for _ in range(frames):
            self.step(**kw)
        return self.events


# --------------------------------------------------------------------------- image maths


def test_resize_area_integer_ratio_is_the_block_mean():
    a = np.arange(8 * 12, dtype=np.float32).reshape(8, 12)
    out = resize_area(a, 3, 2)
    assert out.shape == (2, 3)
    assert out[0, 0] == pytest.approx(a[0:4, 0:4].mean())
    assert out[1, 2] == pytest.approx(a[4:8, 8:12].mean())


def test_resize_area_non_integer_ratio_keeps_shape_and_mean_of_constant():
    out = resize_area(np.full((180, 320), 77, dtype=np.uint8), 64, 48)
    assert out.shape == (48, 64)
    assert np.allclose(out, 77.0)


def test_resize_area_same_size_is_identity_and_smaller_source_is_sampled():
    a = np.arange(6 * 8, dtype=np.float32).reshape(6, 8)
    assert np.array_equal(resize_area(a, 8, 6), a)
    up = resize_area(a, 16, 12)  # source smaller than the target in both directions
    assert up.shape == (12, 16)
    assert up[0, 0] == a[0, 0] and up[-1, -1] == a[-1, -1]


def test_resize_area_rejects_colour_input():
    with pytest.raises(ValueError, match="2-D"):
        resize_area(np.zeros((4, 4, 3)), 2, 2)


def test_blur3_keeps_a_constant_and_spreads_an_impulse():
    assert np.allclose(blur3(np.full((5, 6), 3.0, dtype=np.float32)), 3.0)
    a = np.zeros((5, 5), dtype=np.float32)
    a[2, 2] = 16.0
    b = blur3(a)
    assert b[2, 2] == pytest.approx(4.0)
    assert b[2, 1] == pytest.approx(2.0)
    assert b[1, 1] == pytest.approx(1.0)
    assert b.sum() == pytest.approx(16.0)


def test_unit_signature_is_zero_mean_unit_norm_and_flat_is_none():
    sig = unit_signature(np.random.default_rng(0).normal(size=(10, 10)).astype(np.float32))
    assert float(sig.mean()) == pytest.approx(0.0, abs=1e-6)
    assert float(np.linalg.norm(sig)) == pytest.approx(1.0, abs=1e-6)
    assert unit_signature(np.full((10, 10), 5.0, dtype=np.float32)) is None
    low = np.random.default_rng(0).normal(scale=0.5, size=(10, 10)).astype(np.float32)
    assert unit_signature(low, min_std=2.0) is None
    assert unit_signature(low, min_std=0.0) is not None
    bad = np.full((4, 4), np.nan, dtype=np.float32)
    assert unit_signature(bad) is None


def test_identical_frames_have_similarity_one_and_brightness_does_not_matter():
    cfg = LapConfig()
    w = make_world()
    a = make_signature(view(w, 10), cfg)
    assert float(a @ a) == pytest.approx(1.0, abs=1e-5)
    brighter = make_signature(view(w, 10) * 0.5 + 40.0, cfg)  # gain and offset
    assert float(a @ brighter) == pytest.approx(1.0, abs=1e-4)
    inverted = make_signature(255.0 - view(w, 10), cfg)
    assert float(a @ inverted) == pytest.approx(-1.0, abs=1e-4)


def test_blur_off_compares_the_raw_thumbnail():
    w = make_world()
    raw = make_signature(view(w, 0), LapConfig(blur=False))
    blurred = make_signature(view(w, 0), LapConfig(blur=True))
    assert float(raw @ raw) == pytest.approx(1.0, abs=1e-5)
    assert float(raw @ blurred) < 0.999  # a different vector: the blur did something


def test_similarity_falls_with_distance_and_unrelated_scenes_are_near_zero():
    cfg = LapConfig()
    w = make_world()
    ref = make_signature(view(w, 0), cfg)
    sims = [float(ref @ make_signature(view(w, d), cfg)) for d in (0, 2, 6, 12, 24, 48)]
    assert sims == sorted(sims, reverse=True)
    assert sims[1] > 0.8 > sims[3]
    other = make_signature(view(make_world(seed=99), 0), cfg)
    assert abs(float(ref @ other)) < 0.35


# --------------------------------------------------------------------------- config


@pytest.mark.parametrize(
    "kwargs",
    [
        {"similarity_threshold": 0.5, "rearm_threshold": 0.6},
        {"similarity_threshold": 0.5, "rearm_threshold": 0.5},
        {"similarity_threshold": 1.5},
        {"rearm_threshold": 0.0},
        {"min_lap_time_s": -1.0},
        {"min_lap_distance_m": -0.1},
        {"peak_window_max_s": 0.0},
        {"thumb_width_px": 2},
        {"thumb_height_px": 3},
        {"min_reference_std": -1.0},
        {"similarity_threshold": float("nan")},
        {"min_lap_time_s": float("inf")},
    ],
)
def test_bad_config_is_refused(kwargs):
    with pytest.raises(ValueError):
        LapCounter(LapConfig(**kwargs))


def test_field_names_list_every_tunable():
    names = LapConfig.field_names()
    assert "similarity_threshold" in names and "min_lap_distance_m" in names
    assert len(names) == len(set(names))


# --------------------------------------------------------------------------- starting


def test_waits_for_the_car_to_move_then_takes_the_next_frame_as_the_reference():
    c = LapCounter(fast_cfg())
    d = Drive(c)
    for _ in range(10):
        d.step(moving=False)
    assert c.state == STATE_WAITING and c.similarity is None
    c.update_odom(d.t, 0.29, d.x)  # just under the speed threshold
    d.step(moving=False)
    assert c.state == STATE_WAITING
    c.update_odom(d.t, 0.3, d.x)
    d.step()
    assert c.state == STATE_RUNNING and c.start_time_s == pytest.approx(d.t)
    assert c.similarity == 1.0


def test_start_speed_zero_starts_on_the_first_frame_without_odometry():
    c = LapCounter(LapConfig(start_speed_mps=0.0))
    c.update_frame(0.0, view(make_world(), 0))
    assert c.state == STATE_RUNNING


def test_blank_frame_is_never_a_reference():
    c = LapCounter(LapConfig(start_speed_mps=0.0))
    c.update_frame(0.0, np.zeros((48, 64), dtype=np.uint8))
    assert c.state == STATE_WAITING
    c.update_frame(0.1, np.full((48, 64), 3, dtype=np.uint8))
    assert c.state == STATE_WAITING
    c.update_frame(0.2, view(make_world(), 0))
    assert c.state == STATE_RUNNING


def test_non_finite_time_is_ignored():
    c = LapCounter(LapConfig(start_speed_mps=0.0))
    assert c.update_frame(float("nan"), view(make_world(), 0)) is None
    assert c.state == STATE_WAITING


# --------------------------------------------------------------------------- lap detection


def test_identical_frame_returning_counts_a_lap_with_the_right_time_and_distance():
    d = start_driving()
    d.run_laps(3.2)
    assert [e.lap for e in d.events] == [1, 2, 3]
    for e in d.events:
        assert e.lap_time_s == pytest.approx(LAP_S, abs=2.0 / FPS)
        assert e.peak_similarity > 0.97
        assert e.distance_m == pytest.approx(LAP_S * 1.0, abs=0.15)
    assert d.c.laps == 3
    assert d.c.best_lap_s == min(e.lap_time_s for e in d.events)
    assert d.c.last_lap_s == d.events[-1].lap_time_s
    assert d.events[-1].best_lap_s == d.c.best_lap_s


def start_driving(cfg=None, **kw) -> Drive:
    """A counter started the normal way: odometry says moving, first frame is the reference."""
    c = LapCounter(cfg or fast_cfg())
    d = Drive(c, **kw)
    c.update_odom(0.0, d.speed, 0.0)
    assert c.update_frame(0.0, view(d.world, 0)) is None
    assert c.state == STATE_RUNNING
    return d


def test_no_lap_before_the_loop_is_closed():
    d = start_driving()
    d.run_laps(0.9)
    assert d.events == []
    assert max(d.sims[5:]) < 0.8


def test_noise_does_not_break_detection_and_does_not_invent_laps():
    cfg = fast_cfg()
    d = start_driving(cfg)
    rng = np.random.default_rng(5)
    for _ in range(int(2.2 * LOOP_PX / PX_PER_FRAME)):
        d.pos += PX_PER_FRAME
        d.t += 1.0 / FPS
        d.x += 1.0 / FPS
        d.c.update_odom(d.t, 1.0, d.x)
        noisy = view(d.world, d.pos) + rng.normal(scale=12.0, size=(VIEW_H, VIEW_W))
        event = d.c.update_frame(d.t, np.clip(noisy, 0, 255))
        if event:
            d.events.append(event)
    assert [e.lap for e in d.events] == [1, 2]
    assert all(e.peak_similarity > 0.8 for e in d.events)


def test_lighting_change_between_laps_still_counts():
    d = start_driving()
    for i in range(int(2.2 * LOOP_PX / PX_PER_FRAME)):
        d.pos += PX_PER_FRAME
        d.t += 1.0 / FPS
        d.x += 1.0 / FPS
        d.c.update_odom(d.t, 1.0, d.x)
        lap_index = d.pos / LOOP_PX
        gain = 1.0 - 0.45 * lap_index  # the room gets darker every lap
        img = np.clip(view(d.world, d.pos) * gain + 10.0 * lap_index, 0, 255)
        event = d.c.update_frame(d.t, img)
        if event:
            d.events.append(event)
    assert [e.lap for e in d.events] == [1, 2]


def _drive_with_start_line_edit(edit, laps=2.3, seed=1):
    """Drive `laps` laps; `edit(frame, near_line)` may alter the frame within 12 px of the line."""
    d = start_driving()
    for _ in range(int(laps * LOOP_PX / PX_PER_FRAME)):
        d.pos += PX_PER_FRAME
        d.t += 1.0 / FPS
        d.x += 1.0 / FPS
        d.c.update_odom(d.t, 1.0, d.x)
        phase = d.pos % LOOP_PX
        near = phase > LOOP_PX - 12 or phase < 12
        frame = view(d.world, d.pos)
        event = d.c.update_frame(d.t, edit(frame, near, int(d.pos // LOOP_PX)) if near else frame)
        d.sims.append(d.c.similarity)
        if event:
            d.events.append(event)
    return d


def test_start_line_drifting_a_few_pixels_each_lap_still_counts():
    # The car a little to one side of the line on each pass: the view shifts by a few pixels.
    drift = {1: 2, 2: -3, 3: 2}

    def edit(frame, near, lap):
        return np.roll(frame, drift.get(lap, 0), axis=1)

    d = _drive_with_start_line_edit(edit, laps=2.3)
    assert [e.lap for e in d.events] == [1, 2]
    assert all(0.8 < e.peak_similarity < 1.0 for e in d.events)


def test_a_small_occluder_at_the_line_still_counts_a_big_one_does_not():
    rng = np.random.default_rng(11)
    clutter = rng.uniform(20, 220, size=(VIEW_H, VIEW_W)).astype(np.float32)

    def occlude(fraction):
        cols = int(VIEW_W * fraction)

        def edit(frame, near, lap):
            out = frame.copy()
            if lap >= 1:  # the reference lap (lap 0) is clean
                out[:, :cols] = clutter[:, :cols]
            return out

        return edit

    small = _drive_with_start_line_edit(occlude(0.12), laps=1.3)
    assert small.c.laps == 1
    big = _drive_with_start_line_edit(occlude(0.6), laps=1.3)
    assert big.c.laps == 0
    assert max(big.sims[20:]) < 0.8  # the first frames are still next to the reference


def test_a_different_place_never_counts():
    d = start_driving()
    other = make_world(seed=42)
    for _ in range(int(3 * LOOP_PX / PX_PER_FRAME)):
        d.pos += PX_PER_FRAME
        d.t += 1.0 / FPS
        d.x += 1.0 / FPS
        d.c.update_odom(d.t, 1.0, d.x)
        assert d.c.update_frame(d.t, view(other, d.pos)) is None
    assert d.c.laps == 0


def test_crossing_is_stamped_at_the_peak_not_at_the_threshold():
    d = start_driving()
    d.run_laps(1.3)
    (event,) = d.events
    # The frame exactly one lap on is the one with similarity ~1; it is not the first one over
    # the threshold (that is a couple of frames earlier).
    expected = LOOP_PX / PX_PER_FRAME / FPS
    assert event.crossing_time_s == pytest.approx(expected, abs=1e-6)
    assert event.peak_similarity == pytest.approx(1.0, abs=1e-3)


def test_peak_is_the_max_inside_the_window_even_when_it_is_not_the_last_sample():
    cfg = fast_cfg(min_lap_time_s=0.0, min_lap_distance_m=0.0)
    c = LapCounter(cfg)
    w = make_world()
    c.update_odom(0.0, 1.0, 0.0)
    c.update_frame(0.0, view(w, 0))
    t = 0.0
    out = None
    # drive away, then approach the start line from -8 px to +8 px in 2 px steps
    for pos in range(60, 400, 4):
        t += 0.1
        c.update_odom(t, 1.0, t)
        c.update_frame(t, view(w, pos))
    approach = [392, 396, 400, 404, 408]
    times = {}
    for pos in approach + [440]:
        t += 0.1
        times[pos] = t
        c.update_odom(t, 1.0, t)
        out = c.update_frame(t, view(w, pos)) or out
    assert out is not None
    assert out.crossing_time_s == pytest.approx(times[400])
    assert out.peak_similarity == pytest.approx(1.0, abs=1e-3)


# --------------------------------------------------------------------------- hysteresis


def test_parked_in_front_of_the_start_counts_once_not_every_frame():
    d = start_driving()
    d.run_laps(1.0)
    assert d.events == []
    assert d.c.similarity > 0.99  # stopped exactly on the line: the window is open
    # stop in the bright zone and sit there for many seconds
    for _ in range(200):
        d.step(moving=False)
    assert d.c.laps == 1  # window closed by peak_window_max_s, then disarmed
    assert d.sims[-1] > 0.9


def test_similarity_flickering_around_the_threshold_counts_once():
    cfg = fast_cfg(min_lap_time_s=0.0, min_lap_distance_m=0.0)
    c = LapCounter(cfg)
    w = make_world()
    c.update_odom(0.0, 1.0, 0.0)
    c.update_frame(0.0, view(w, 0))
    t = 0.0
    for pos in range(40, 400, 4):  # drive away: similarity drops under the rearm threshold
        t += 0.1
        c.update_odom(t, 1.0, t)
        c.update_frame(t, view(w, pos))
    events = []
    # approach and wobble around the 0.8 contour (offset 3 to 7 px), then leave
    for pos in (392, 394, 389, 395, 391, 396, 400, 405, 393, 406, 392, 408, 420, 460):
        t += 0.1
        c.update_odom(t, 1.0, t)
        e = c.update_frame(t, view(w, pos))
        if e:
            events.append(e)
    for pos in (395, 400, 404, 398):  # back in the bright zone without ever dropping below rearm
        t += 0.1
        c.update_odom(t, 1.0, t)
        e = c.update_frame(t, view(w, pos))
        if e:
            events.append(e)
    assert c.laps == 1


def test_a_second_lap_needs_the_rearm_dip():
    cfg = fast_cfg(min_lap_time_s=0.0, min_lap_distance_m=0.0)
    c = LapCounter(cfg)
    w = make_world()
    c.update_odom(0.0, 1.0, 0.0)
    c.update_frame(0.0, view(w, 0))
    t = 0.0
    laps_at = []
    # Hover near the start (offset about 14 px: similarity between rearm and threshold, never
    # below rearm) for a long time: the start disarms, so nothing is counted.
    for _ in range(100):
        t += 0.1
        c.update_odom(t, 1.0, t)
        c.update_frame(t, view(w, 6))
    laps_at.append(c.laps)
    assert c.similarity > 0.8 or c.similarity > c.cfg.rearm_threshold
    # Even at the start line itself, never having dipped under rearm => still not armed.
    for _ in range(20):
        t += 0.1
        c.update_odom(t, 1.0, t)
        c.update_frame(t, view(w, 0))
    laps_at.append(c.laps)
    assert laps_at == [0, 0]
    assert c.status()["armed"] is False


# --------------------------------------------------------------------------- gates


def test_a_return_before_min_lap_time_is_ignored_and_the_next_real_one_counts():
    # A 400 px loop at 60 px/s is 6.7 s; with a 10 s minimum the first pass is ignored.
    d = start_driving(fast_cfg(min_lap_time_s=10.0))
    d.run_laps(1.1)
    assert d.events == []
    d.run_laps(1.0)  # the second pass is 13.3 s after the start
    assert [e.lap for e in d.events] == [1]
    assert d.events[0].lap_time_s == pytest.approx(2 * LAP_S, abs=2.0 / FPS)


def test_a_bright_stretch_that_starts_before_the_gate_is_discarded_whole():
    # Gate opens in the middle of the bright stretch (min lap time = 6.7 s +- a few frames).
    d = start_driving(fast_cfg(min_lap_time_s=LAP_S + 0.05))
    d.run_laps(1.0 + 12 / LOOP_PX * PX_PER_FRAME)  # through the bright stretch
    assert d.events == []


def test_a_slow_crawl_never_passes_the_distance_gate():
    # The same views, but the odometry says the car only moved 0.2 m in the lap (a crawl or a
    # frozen wheel-odometry): min_lap_distance_m = 1 m blocks it.
    c = LapCounter(fast_cfg())
    w = make_world()
    c.update_odom(0.0, 0.31, 0.0)
    c.update_frame(0.0, view(w, 0))
    t = 0.0
    for i, pos in enumerate(range(4, 440, 4), start=1):
        t += 0.1
        c.update_odom(t, 0.31, 0.002 * i)
        assert c.update_frame(t, view(w, pos)) is None
    assert c.laps == 0


def test_distance_is_path_length_so_reversing_adds_to_it():
    c = LapCounter(fast_cfg())
    for x in (0.0, 1.0, 2.0, 1.0, 0.0):
        c.update_odom(0.0, 0.5, x)
    assert c.distance_m == pytest.approx(4.0)


def test_distance_without_a_position_integrates_speed_and_ignores_gaps_and_garbage():
    c = LapCounter(fast_cfg())
    c.update_odom(0.0, 2.0)
    c.update_odom(0.5, 2.0)  # 1 m
    c.update_odom(10.0, 2.0)  # a 9.5 s gap counts as at most 0.5 s: 1 m
    c.update_odom(float("nan"), 2.0)
    c.update_odom(10.1, float("inf"))
    c.update_odom(10.2, 1.0, float("nan"))  # NaN position falls back to speed: 0.2 m
    assert c.distance_m == pytest.approx(2.2)
    assert c.odom_seen and c.speed_mps == 1.0


# --------------------------------------------------------------------------- reset


def test_reset_clears_everything_and_the_next_frame_is_the_new_reference():
    d = start_driving()
    d.run_laps(1.2)
    assert d.c.laps == 1
    d.c.reset()
    assert d.c.state == STATE_WAITING
    assert d.c.laps == 0 and d.c.best_lap_s is None and d.c.last_lap_s is None
    assert d.c.similarity is None
    dist_before = d.c.distance_m
    new_world = make_world(seed=7)
    d.world = new_world
    d.step(moving=False)  # even standing still: reset asked for the next frame
    assert d.c.state == STATE_RUNNING
    assert d.c.distance_m == dist_before  # odometry keeps running through a reset
    d.pos = 0.0
    d.run_laps(1.3)
    assert d.c.laps == 1


def test_reset_without_capture_waits_for_motion_again():
    c = LapCounter(fast_cfg())
    c.update_odom(0.0, 1.0, 0.0)
    c.update_frame(0.0, view(make_world(), 0))
    c.reset(capture_next_frame=False)
    c.update_odom(1.0, 0.0, 0.0)
    c.update_frame(1.0, view(make_world(), 0))
    assert c.state == STATE_WAITING
    c.update_odom(2.0, 1.0, 1.0)
    c.update_frame(2.0, view(make_world(), 0))
    assert c.state == STATE_RUNNING


# --------------------------------------------------------------------------- heading cross-check


def test_heading_turns_integrate_yaw_rate_per_lap_and_are_signed():
    d = start_driving()
    omega = 2.0 * math.pi / LAP_S  # one left turn per lap
    t_last = 0.0
    for _ in range(int(2.3 * LOOP_PX / PX_PER_FRAME)):
        d.pos += PX_PER_FRAME
        d.t += 1.0 / FPS
        d.x += 1.0 / FPS
        d.c.update_odom(d.t, 1.0, d.x)
        d.c.update_yaw_rate(d.t, omega)
        e = d.c.update_frame(d.t, view(d.world, d.pos))
        if e:
            d.events.append(e)
        t_last = d.t
    assert len(d.events) == 2
    for e in d.events:
        assert e.heading_turns == pytest.approx(1.0, abs=0.05)
    assert d.c.status()["heading_turns"] == pytest.approx(0.3, abs=0.06)
    assert t_last > 0


def test_clockwise_laps_report_negative_turns():
    d = start_driving()
    omega = -2.0 * math.pi / LAP_S
    for _ in range(int(1.2 * LOOP_PX / PX_PER_FRAME)):
        d.pos += PX_PER_FRAME
        d.t += 1.0 / FPS
        d.x += 1.0 / FPS
        d.c.update_odom(d.t, 1.0, d.x)
        d.c.update_yaw_rate(d.t, omega)
        e = d.c.update_frame(d.t, view(d.world, d.pos))
        if e:
            d.events.append(e)
    assert d.events[0].heading_turns == pytest.approx(-1.0, abs=0.05)


def test_heading_never_counts_or_vetoes_a_lap():
    # Spinning on the spot with no repeat in the camera view: turns pile up, laps stay 0.
    d = start_driving()
    other = make_world(seed=3)
    for i in range(200):
        d.t += 0.1
        d.c.update_odom(d.t, 1.0, d.t)
        d.c.update_yaw_rate(d.t, 3.0)
        assert d.c.update_frame(d.t, view(other, 4 * i)) is None
    assert d.c.laps == 0
    assert d.c.status()["heading_turns"] > 5.0
    # And a lap with NO turning (a straight there and back) still counts.
    d2 = start_driving()
    d2.run_laps(1.2)
    assert d2.c.laps == 1
    assert d2.events[0].heading_turns is None  # no yaw source seen: reported as unknown


def test_yaw_gaps_are_capped_and_garbage_is_ignored():
    c = LapCounter(fast_cfg())
    c.update_yaw_rate(0.0, 1.0)
    c.update_yaw_rate(100.0, 1.0)  # a 100 s gap integrates as 0.5 s
    c.update_yaw_rate(float("nan"), 1.0)
    c.update_yaw_rate(101.0, float("nan"))
    assert c.yaw_rad == pytest.approx(0.5)
    assert c.yaw_seen


# --------------------------------------------------------------------------- reporting


def test_event_and_status_are_json_serialisable_with_the_documented_keys():
    d = start_driving()
    d.c.update_yaw_rate(0.0, 0.0)
    d.run_laps(1.2)
    event = json.loads(json.dumps(d.events[0].to_dict(), allow_nan=False))
    assert set(event) == {
        "lap",
        "lap_time_s",
        "best_lap_s",
        "peak_similarity",
        "crossing_time_s",
        "distance_m",
        "heading_turns",
    }
    status = json.loads(json.dumps(d.c.status(), allow_nan=False))
    assert set(status) == {
        "state",
        "similarity",
        "laps",
        "last_lap_s",
        "best_lap_s",
        "armed",
        "speed_mps",
        "distance_m",
        "heading_turns",
        "odom_seen",
    }
    assert status["state"] == STATE_RUNNING and status["laps"] == 1


def test_status_before_the_start_is_all_empty():
    status = LapCounter().status()
    assert status["state"] == STATE_WAITING
    assert status["similarity"] is None and status["laps"] == 0
    assert status["best_lap_s"] is None and status["heading_turns"] is None
    assert status["distance_m"] == 0.0


def test_best_lap_tracks_the_minimum_across_laps():
    d = start_driving()
    d.run_laps(1.2)  # lap 1 at 1x speed
    # then drive twice as fast: 8 px per frame
    for _ in range(int(2.3 * LOOP_PX / 8)):
        d.pos += 8
        d.t += 1.0 / FPS
        d.x += 2.0 / FPS
        d.c.update_odom(d.t, 2.0, d.x)
        e = d.c.update_frame(d.t, view(d.world, d.pos))
        if e:
            d.events.append(e)
    laps = [e.lap_time_s for e in d.events]
    assert len(laps) >= 3
    assert laps[0] > laps[1]
    assert d.events[-1].best_lap_s == pytest.approx(min(laps))
