# minimum-curvature raceline generation from a track centerline and its measured widths

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .clearance import ClearanceMap
from .raceline_io import (
    centerline_path,
    generated_raceline_path,
    read_centerline_csv,
    write_raceline_csv,
)

CAR_HALF_WIDTH_M = 0.155
STEER_MAX_RAD = 0.4189
WHEELBASE_M = 0.3302
KAPPA_MAX_RADPM = math.tan(STEER_MAX_RAD) / WHEELBASE_M
VEHICLE_MASS_KG = 3.74
A_MAX_MPS2 = 9.51
V_SWITCH_MPS = 7.319
MU = 1.0489
GRAVITY_MPS2 = 9.81
DEFAULT_MARGIN_M = 0.15
DEFAULT_V_MAX_MPS = 8.0
# a wall search only ever has to reach past the widest corridor these maps declare
WALL_SEARCH_M = 3.0


@dataclass(frozen=True)
class RacelineSpec:
    """everything the optimizer needs beyond the track itself."""

    margin_m: float = DEFAULT_MARGIN_M
    half_width_m: float = CAR_HALF_WIDTH_M
    min_half_width_m: float = 0.0
    kappa_max_radpm: float = KAPPA_MAX_RADPM
    stepsize_prep_m: float = 0.4
    stepsize_reg_m: float = 1.5
    stepsize_interp_m: float = 0.2
    smooth_window: int = 3
    v_max_mps: float = DEFAULT_V_MAX_MPS
    a_max_mps2: float = A_MAX_MPS2
    v_switch_mps: float = V_SWITCH_MPS
    mu: float = MU
    mass_kg: float = VEHICLE_MASS_KG
    # the tyre never quite delivers mu * g, so this trims the cornering budget the profile plans on
    grip_usage: float = 1.0
    filt_window: int = 5
    relinearize_passes: int = 5
    trust_region_m: float = 0.15
    safety_passes: int = 8
    tighten_step_m: float = 0.02

    def __post_init__(self) -> None:
        if self.margin_m < 0.0:
            raise ValueError(f"margin_m must be >= 0, got {self.margin_m}")
        if self.half_width_m <= 0.0:
            raise ValueError(f"half_width_m must be > 0, got {self.half_width_m}")
        if self.v_max_mps <= 0.0 or self.a_max_mps2 <= 0.0:
            raise ValueError("v_max_mps and a_max_mps2 must both be > 0")
        if not 0.0 < self.grip_usage <= 1.0:
            raise ValueError(f"grip_usage must be in (0, 1], got {self.grip_usage}")

    @property
    def keep_out_m(self) -> float:
        """how far the corridor is pulled in on each side before the optimizer sees it."""
        return self.half_width_m + self.margin_m

    @property
    def required_clearance_m(self) -> float:
        """clearance the finished line must hold, half the margin below what was reserved."""
        return self.half_width_m + 0.5 * self.margin_m


@dataclass(frozen=True)
class RacelineReport:
    """what the generated line measured against the map and the steering limit."""

    map_name: str
    points: int
    length_m: float
    closing_gap_m: float
    min_clearance_m: float
    required_clearance_m: float
    max_kappa_radpm: float
    kappa_limit_radpm: float
    v_min_mps: float
    v_max_mps: float
    lap_time_sec: float

    @property
    def clears_walls(self) -> bool:
        return self.min_clearance_m >= self.required_clearance_m

    @property
    def steerable(self) -> bool:
        return self.max_kappa_radpm <= self.kappa_limit_radpm

    def summary(self) -> str:
        walls = "ok" if self.clears_walls else "TOO CLOSE"
        steering = "ok" if self.steerable else "TOO TIGHT"
        return (
            f"{self.map_name}: {self.points} points over {self.length_m:.1f} m "
            f"(closing gap {self.closing_gap_m:.3f} m), "
            f"min clearance {self.min_clearance_m:.3f} m (need {self.required_clearance_m:.3f}, {walls}), "
            f"max |kappa| {self.max_kappa_radpm:.3f} rad/m (limit {self.kappa_limit_radpm:.3f}, {steering}), "
            f"speed {self.v_min_mps:.2f}..{self.v_max_mps:.2f} m/s, "
            f"ideal lap {self.lap_time_sec:.2f} s"
        )


def cap_track_widths(reftrack: np.ndarray, right_m, left_m) -> np.ndarray:
    """clip a declared corridor down to the room actually measured on the occupancy map."""
    capped = _as_reftrack(reftrack)
    capped[:, 2] = np.minimum(capped[:, 2], np.asarray(right_m, dtype=np.float64).reshape(-1))
    capped[:, 3] = np.minimum(capped[:, 3], np.asarray(left_m, dtype=np.float64).reshape(-1))
    return capped


def shrink_track_widths(reftrack: np.ndarray, keep_out_m: float, min_half_width_m: float = 0.0) -> np.ndarray:
    """pull both track edges in by keep_out_m so any line inside the result already fits the car."""
    if keep_out_m < 0.0:
        raise ValueError(f"keep_out_m must be >= 0, got {keep_out_m}")
    if min_half_width_m < 0.0:
        raise ValueError(f"min_half_width_m must be >= 0, got {min_half_width_m}")
    shrunk = _as_reftrack(reftrack)
    shrunk[:, 2:4] = np.maximum(shrunk[:, 2:4] - keep_out_m, min_half_width_m)
    return shrunk


def normals_fold(reftrack: np.ndarray, normals: np.ndarray) -> np.ndarray:
    """per point, whether its normal meets the next one inside the corridor, folding the offset frame."""
    track = _as_reftrack(reftrack)
    normals = np.asarray(normals, dtype=np.float64).reshape(-1, 2)
    if normals.shape[0] != track.shape[0]:
        raise ValueError(f"need one normal per point, got {normals.shape[0]} and {track.shape[0]}")
    step = np.roll(track[:, :2], -1, axis=0) - track[:, :2]
    ahead = np.roll(normals, -1, axis=0)
    # solve p + t * n == p_next + u * n_next for the pair of normal lines
    det = ahead[:, 0] * normals[:, 1] - ahead[:, 1] * normals[:, 0]
    parallel = np.isclose(det, 0.0)
    safe_det = np.where(parallel, 1.0, det)
    t = (ahead[:, 0] * step[:, 1] - ahead[:, 1] * step[:, 0]) / safe_det
    u = (normals[:, 0] * step[:, 1] - normals[:, 1] * step[:, 0]) / safe_det
    reach = float(track[:, 2:4].max())
    return ~parallel & (t * u > 0.0) & (np.minimum(np.abs(t), np.abs(u)) < reach)


def acceleration_limits(spec: RacelineSpec, points: int = 21) -> tuple[np.ndarray, np.ndarray]:
    """the tph ggv diagram and motor limit table for the f1tenth single-track vehicle."""
    speeds = np.linspace(0.0, spec.v_max_mps, points)
    # the sim's motor holds a_max up to v_switch and falls off as v_switch / v above it
    motor_mps2 = spec.a_max_mps2 * np.minimum(1.0, spec.v_switch_mps / np.maximum(speeds, 1e-6))
    lateral_mps2 = np.full_like(speeds, spec.grip_usage * spec.mu * GRAVITY_MPS2)
    ggv = np.column_stack((speeds, motor_mps2, lateral_mps2))
    ax_max_machines = np.column_stack((speeds, motor_mps2))
    return ggv, ax_max_machines


def drivable_corridor(
    reftrack: np.ndarray, normals: np.ndarray, clearance: ClearanceMap, keep_out_m: float
) -> tuple[np.ndarray, np.ndarray]:
    """how far the car centre may sit either side of the reference line and still clear the walls."""
    normals = np.asarray(normals, dtype=np.float64).reshape(-1, 2)
    # the band is measured by perpendicular clearance, not distance along the normal, because an
    # oblique wall is closer than it looks; both bounds go negative where the line must move over
    upper, lower = clearance.lateral_bounds(
        reftrack[:, 0], reftrack[:, 1], normals, WALL_SEARCH_M, keep_out_m
    )
    return upper, -lower


def prepare_reference_line(reftrack: np.ndarray, spec: RacelineSpec) -> np.ndarray:
    """evenly spaced, moving-average smoothed reference track, still an open loop."""
    import trajectory_planning_helpers as tph

    dense = _resample_closed(_as_reftrack(reftrack), spec.stepsize_prep_m)
    if spec.smooth_window > 1:
        # map-extracted centerlines are pixel-quantised, and that noise wrecks the spline normals
        for axis in (0, 1):
            dense[:, axis] = tph.conv_filt.conv_filt(
                signal=dense[:, axis], filt_window=spec.smooth_window, closed=True
            )
    return _resample_closed(dense, spec.stepsize_reg_m)


def solve_min_curvature(
    reftrack: np.ndarray,
    clearance: ClearanceMap,
    spec: RacelineSpec,
    keep_out_m: float,
    kappa_bound_radpm: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """min-curvature qp relinearized a few times, as refline, normals and lateral shift alpha."""
    track = _as_reftrack(reftrack)
    for _ in range(spec.relinearize_passes):
        solution = _solve_once(track, clearance, spec, keep_out_m, kappa_bound_radpm)
        # the curvature model is linear about the reference line, so it is only honest nearby;
        # the line creeps onto each solution instead of jumping, which would land it on a wall
        track = _relinearize(track, *solution, spec)
    return _solve_once(track, clearance, spec, keep_out_m, kappa_bound_radpm)


def _solve_once(
    reftrack: np.ndarray,
    clearance: ClearanceMap,
    spec: RacelineSpec,
    keep_out_m: float,
    kappa_bound_radpm: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """one qp on the corridor left around this reference line after keep_out_m."""
    import trajectory_planning_helpers as tph

    track = _as_reftrack(reftrack)
    closed_path = np.vstack((track[:, :2], track[0, :2]))
    _, _, spline_matrix, normals = tph.calc_splines.calc_splines(path=closed_path)
    # alpha is positive along the normal, which is the right-hand side, matching w_tr_right
    usable = cap_track_widths(
        shrink_track_widths(track, keep_out_m, spec.min_half_width_m),
        *drivable_corridor(track, normals, clearance, keep_out_m),
    )
    folded = normals_fold(usable, normals)
    if folded.any():
        raise RuntimeError(
            f"reference line normals cross at {int(folded.sum())} of {folded.size} points, so the "
            "corridor is ambiguous; raise stepsize_reg_m or smooth_window"
        )
    try:
        alpha, _ = tph.opt_min_curv.opt_min_curv(
            reftrack=usable,
            normvectors=normals,
            A=spline_matrix,
            kappa_bound=kappa_bound_radpm,
            # the corridor was already shrunk by the car's half width, so the solver adds nothing
            w_veh=0.0,
            closed=True,
        )
    except (RuntimeError, ValueError) as ex:
        raise RuntimeError(
            f"no line fits a {keep_out_m:.3f} m keep-out with |kappa| <= "
            f"{kappa_bound_radpm:.3f} rad/m on this track: {ex}"
        ) from ex
    return usable, normals, alpha


def _relinearize(
    track: np.ndarray, usable: np.ndarray, normals: np.ndarray, alpha: np.ndarray, spec: RacelineSpec
) -> np.ndarray:
    """re-space the reference track a trust region towards the solution, carrying its edges along."""
    import trajectory_planning_helpers as tph

    step = np.clip(alpha, -spec.trust_region_m, spec.trust_region_m)
    moved, _, _, _, spline_inds, t_values = tph.create_raceline.create_raceline(
        refline=usable[:, :2],
        normvectors=normals,
        alpha=step,
        stepsize_interp=spec.stepsize_reg_m,
    )[:6]
    # the declared corridor is nailed to the map, so moving the line by step shifts both its edges
    edges = track[:, 2:4] - np.column_stack((step, -step))
    interpolated = tph.interp_track_widths.interp_track_widths(
        w_track=edges, spline_inds=spline_inds, t_values=t_values, incl_last_point=False
    )
    return np.column_stack((moved, interpolated))


def generate_raceline(
    reftrack: np.ndarray, clearance: ClearanceMap, spec: RacelineSpec, map_name: str = ""
) -> tuple[dict, RacelineReport]:
    """min-curvature raceline, re-solved with a tighter corridor until it is drivable as written."""
    smoothed = prepare_reference_line(reftrack, spec)
    keep_out_m = spec.keep_out_m
    kappa_bound_radpm = spec.kappa_max_radpm
    for _ in range(spec.safety_passes):
        solution = solve_min_curvature(smoothed, clearance, spec, keep_out_m, kappa_bound_radpm)
        raceline = _interpolate_raceline(*solution, spec)
        report = validate_raceline(raceline, clearance, spec, map_name)
        if report.clears_walls and report.steerable:
            return raceline, report
        # the qp only sees the corridor at its own points; the spline drawn through them bulges
        # past it in between, and the only way back is to hand that bulge to the corridor
        keep_out_m += max(report.required_clearance_m - report.min_clearance_m, 0.0)
        keep_out_m += spec.tighten_step_m
        if not report.steerable:
            kappa_bound_radpm *= spec.kappa_max_radpm / report.max_kappa_radpm
    return raceline, report


def _interpolate_raceline(
    reftrack: np.ndarray, normals: np.ndarray, alpha: np.ndarray, spec: RacelineSpec
) -> dict:
    """the seven raceline columns of the offset line, resampled and given a speed profile."""
    import trajectory_planning_helpers as tph

    (
        raceline_xy,
        _,
        coeffs_x,
        coeffs_y,
        spline_inds,
        t_values,
        s_points,
        _,
        el_lengths_cl,
    ) = tph.create_raceline.create_raceline(
        refline=reftrack[:, :2],
        normvectors=normals,
        alpha=alpha,
        stepsize_interp=spec.stepsize_interp_m,
    )
    psi, kappa = tph.calc_head_curv_an.calc_head_curv_an(
        coeffs_x=coeffs_x, coeffs_y=coeffs_y, ind_spls=spline_inds, t_spls=t_values
    )

    ggv, ax_max_machines = acceleration_limits(spec)
    vx = tph.calc_vel_profile.calc_vel_profile(
        ax_max_machines=ax_max_machines,
        kappa=kappa,
        el_lengths=el_lengths_cl,
        closed=True,
        drag_coeff=0.0,
        m_veh=spec.mass_kg,
        ggv=ggv,
        v_max=spec.v_max_mps,
        dyn_model_exp=1.0,
        filt_window=spec.filt_window,
    )
    ax = tph.calc_ax_profile.calc_ax_profile(
        vx_profile=np.append(vx, vx[0]), el_lengths=el_lengths_cl, eq_length_output=False
    )

    return {
        "s_m": s_points,
        "x_m": raceline_xy[:, 0],
        "y_m": raceline_xy[:, 1],
        "psi_rad": tph.normalize_psi.normalize_psi(psi),
        "kappa_radpm": kappa,
        "vx_mps": vx,
        "ax_mps2": ax,
    }


def validate_raceline(
    raceline: dict, clearance: ClearanceMap, spec: RacelineSpec, map_name: str = ""
) -> RacelineReport:
    """measure the finished line against the walls, the steering limit and its own speed profile."""
    xs, ys = raceline["x_m"], raceline["y_m"]
    kappa, vx = raceline["kappa_radpm"], raceline["vx_mps"]
    closed_x, closed_y = np.append(xs, xs[0]), np.append(ys, ys[0])
    el_lengths = np.hypot(np.diff(closed_x), np.diff(closed_y))
    return RacelineReport(
        map_name=map_name,
        points=int(xs.size),
        length_m=float(el_lengths.sum()),
        closing_gap_m=float(el_lengths[-1]),
        min_clearance_m=float(clearance.distance_at(xs, ys).min()),
        required_clearance_m=spec.required_clearance_m,
        max_kappa_radpm=float(np.abs(kappa).max()),
        kappa_limit_radpm=spec.kappa_max_radpm,
        v_min_mps=float(vx.min()),
        v_max_mps=float(vx.max()),
        lap_time_sec=float(np.sum(el_lengths / np.maximum(vx, 1e-6))),
    )


def generate_for_map(map_name: str, spec: RacelineSpec) -> tuple[dict, RacelineReport, Path]:
    """generate and validate a raceline for a map the gym can load, with where it belongs."""
    from f1tenth_gym.envs.track import Track

    track = Track.from_track_name(map_name)
    clearance = ClearanceMap.from_track(track)
    reftrack = read_centerline_csv(centerline_path(track))
    raceline, report = generate_raceline(reftrack, clearance, spec, map_name)
    return raceline, report, generated_raceline_path(track)


def _resample_closed(reftrack: np.ndarray, stepsize_m: float) -> np.ndarray:
    """re-space an open reference track around its closed arc length, dropping the repeat point."""
    import trajectory_planning_helpers as tph

    track = _as_reftrack(reftrack)
    return tph.interp_track.interp_track(track=np.vstack((track, track[0])), stepsize=stepsize_m)[:-1]


def _as_reftrack(reftrack) -> np.ndarray:
    """a writable [x, y, w_tr_right, w_tr_left] copy, rejecting anything narrower."""
    track = np.array(reftrack, dtype=np.float64, copy=True)
    if track.ndim != 2 or track.shape[1] < 4:
        raise ValueError(f"reference track must be (n, 4+) [x, y, w_right, w_left], got {track.shape}")
    return track[:, :4]


def _spec_from_args(args) -> RacelineSpec:
    """spec with only the flags the caller actually passed applied."""
    overrides = {
        "margin_m": args.margin,
        "half_width_m": args.half_width,
        "v_max_mps": args.v_max,
        "grip_usage": args.grip_usage,
        "stepsize_reg_m": args.stepsize_reg,
        "stepsize_interp_m": args.stepsize_interp,
        "smooth_window": args.smooth_window,
    }
    return RacelineSpec(**{name: value for name, value in overrides.items() if value is not None})


def main() -> None:
    parser = argparse.ArgumentParser(description="generate a minimum-curvature raceline for a map")
    parser.add_argument("--map", default="Spielberg")
    parser.add_argument("--margin", type=float, default=None, help="safety margin beyond the car's half width, m")
    parser.add_argument("--half-width", type=float, default=None)
    parser.add_argument("--v-max", type=float, default=None)
    parser.add_argument("--grip-usage", type=float, default=None)
    parser.add_argument("--stepsize-reg", type=float, default=None)
    parser.add_argument("--stepsize-interp", type=float, default=None)
    parser.add_argument("--smooth-window", type=int, default=None, help="moving average window, odd")
    parser.add_argument("--out", default=None, help="write here instead of beside the shipped raceline")
    args = parser.parse_args()

    # loading a track pulls in the gym's renderer, which wants a display even headless
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    spec = _spec_from_args(args)
    raceline, report, default_path = generate_for_map(args.map, spec)

    print(report.summary())
    if not report.clears_walls:
        raise SystemExit(
            f"{args.map}: min clearance {report.min_clearance_m:.3f} m is under the required "
            f"{report.required_clearance_m:.3f} m, so nothing was written; raise --margin"
        )
    if not report.steerable:
        raise SystemExit(
            f"{args.map}: max |kappa| {report.max_kappa_radpm:.3f} rad/m exceeds the steering "
            f"limit {report.kappa_limit_radpm:.3f} rad/m, so nothing was written"
        )
    path = write_raceline_csv(
        Path(args.out) if args.out else default_path,
        raceline,
        comment=f"min curvature, margin {spec.margin_m} m, car half width {spec.half_width_m} m",
    )
    print(f"written to {path}")


if __name__ == "__main__":
    main()
