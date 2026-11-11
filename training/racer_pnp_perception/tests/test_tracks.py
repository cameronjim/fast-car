from __future__ import annotations

import math

import numpy as np
from racer_pnp_perception import tracks as tk


def test_held_out_are_rejected_within_tolerance() -> None:
    for p in tk.HELD_OUT_ISLANDS.values():
        assert tk.is_held_out(p)
        near = tk.IslandParams(
            p.lane_width_m + 0.04, p.island_length_m - 0.04, p.island_height_m, p.island_radius_m
        )
        assert tk.is_held_out(near)
        far = tk.IslandParams(
            p.lane_width_m + 0.06, p.island_length_m, p.island_height_m, p.island_radius_m
        )
        assert not tk.is_held_out(far)


def test_held_out_values_match_the_spec() -> None:
    s = tk.HELD_OUT_ISLANDS["stadium_l1.0_s2.0_r0.4"]
    assert s.as_tuple() == (1.0, 2.8, 0.8, 0.4)
    held = [p.as_tuple() for p in tk.HELD_OUT_ISLANDS.values()]
    assert (1.0, 3.0, 2.0, 0.4) in held
    assert (1.2, 2.0, 1.6, 0.8) in held


def test_the_generator_never_yields_a_held_out_island() -> None:
    rng = np.random.default_rng(0)
    for _ in range(3000):
        assert not tk.is_held_out(tk.sample_island_params(rng))


def test_generator_redraws_when_the_draw_is_held_out(monkeypatch) -> None:
    held = next(iter(tk.HELD_OUT_ISLANDS.values()))
    first = [held.lane_width_m, held.island_length_m, held.island_height_m, 0.4]
    calls = {"n": 0}

    class FakeRng:
        """Yields exactly the held-out parameters for the first island, then real draws."""

        def __init__(self) -> None:
            self.real = np.random.default_rng(0)
            self.queue = list(first)

        def uniform(self, *args):
            if args == ():
                return 0.0  # radius branch: random radius
            if self.queue and len(args) == 2:
                calls["n"] += 1
                return self.queue.pop(0)
            return self.real.uniform(*args)

    p = tk.sample_island_params(FakeRng())  # type: ignore[arg-type]
    assert calls["n"] == 4  # the held-out island was drawn first ...
    assert not tk.is_held_out(p)  # ... and rejected
    del monkeypatch


def test_island_world_is_a_closed_lane() -> None:
    w = tk.island_world(tk.IslandParams(1.0, 3.0, 2.0, 0.4))
    assert w.centreline.shape[1] == 2 and w.walls.shape[1] == 4
    # Centreline sits midway between the walls: the nearest wall point is half a lane away.
    pts = w.centreline[::17]
    mid = 0.5 * (w.walls[:, :2] + w.walls[:, 2:])
    d = np.hypot(pts[:, None, 0] - mid[None, :, 0], pts[:, None, 1] - mid[None, :, 1]).min(axis=1)
    assert np.all(np.abs(d - 0.5) < 0.06)
    # Counter-clockwise: positive signed area.
    x, y = w.centreline[:, 0], w.centreline[:, 1]
    assert 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y) > 0


def test_held_out_worlds_build() -> None:
    worlds = tk.held_out_worlds()
    assert set(worlds) == set(tk.HELD_OUT_ISLANDS)


def test_fourier_circle_is_accepted_and_has_the_right_walls() -> None:
    fp = tk.FourierParams(1.5, (0.0,) * 4, (0.0,) * 4, 1.0)
    w = tk.fourier_world(fp)
    assert w is not None
    r = np.hypot(w.walls[:, 0], w.walls[:, 1])
    assert abs(r.min() - 1.0) < 0.02 and abs(r.max() - 2.0) < 0.02


def test_fourier_rejects_tight_curvature() -> None:
    # A small loop with a wide lane: the inner wall would fold over (radius < half width + 0.15).
    assert tk.fourier_world(tk.FourierParams(0.5, (0.0,) * 4, (0.0,) * 4, 1.2)) is None
    # A loop with a strong fourth harmonic pinches somewhere.
    amps = (0.0, 0.0, 0.0, 0.25)
    pinched = tk.FourierParams(1.0, amps, (0.0,) * 4, 0.7)
    assert tk.min_curvature_radius(tk.fourier_centreline(1.0, amps, (0.0,) * 4)) < 0.5
    assert tk.fourier_world(pinched) is None


def test_fourier_rejects_touching_lanes() -> None:
    # A peanut: r = 1 + 0.62 cos(2 theta) pinches to 0.38 m, the two lobes' lanes touch.
    amps = (0.0, 0.62, 0.0, 0.0)
    centre = tk.fourier_centreline(1.0, amps, (0.0,) * 4)
    assert not tk.lanes_clear(centre, 0.7)
    assert tk.fourier_world(tk.FourierParams(1.0, amps, (0.0,) * 4, 0.7)) is None


def test_min_curvature_radius_of_a_circle() -> None:
    circle = tk.fourier_centreline(2.0, (0.0,) * 4, (0.0,) * 4)
    assert abs(tk.min_curvature_radius(circle) - 2.0) < 0.01


def test_sampled_worlds_pass_their_own_tests() -> None:
    rng = np.random.default_rng(1)
    for _ in range(10):
        w = tk.sample_fourier_world(rng)
        assert tk.min_curvature_radius(w.centreline) >= w.lane_width_m / 2 + tk.CURVATURE_MARGIN_M
        assert tk.lanes_clear(w.centreline, w.lane_width_m)
    w = tk.sample_world(np.random.default_rng(2), fourier_share=0.0)
    assert w.name == "island"
    assert math.isfinite(w.bounds()[0])


def test_resample_closed_spacing() -> None:
    sq = np.array([[0, 0], [2, 0], [2, 2], [0, 2]], dtype=float)
    r = tk.resample_closed(sq, 0.1)
    d = np.hypot(*(np.roll(r, -1, axis=0) - r).T)
    assert len(r) == 80 and np.all(d <= 0.1 + 1e-9)
