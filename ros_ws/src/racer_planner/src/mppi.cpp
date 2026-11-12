#include "racer_planner/mppi.hpp"

#include <algorithm>
#include <cmath>

namespace racer_planner {

namespace {
// Rotates the unit vector (c, s) by the angle d: the step's yaw increment is small (under 0.1 rad
// at the vehicle's limits), so a 7th order series is exact to about 1e-12 and replaces two
// libm calls per step. Anything larger, or not finite, recomputes from the yaw itself.
inline void rotate_cs(double yaw_new, double d, double* c, double* s) {
  if (!(std::fabs(d) < 0.5)) {
    *c = std::cos(yaw_new);
    *s = std::sin(yaw_new);
    return;
  }
  const double d2 = d * d;
  const double cd = 1.0 - d2 * (0.5 - d2 * (1.0 / 24.0 - d2 * (1.0 / 720.0)));
  const double sd = d * (1.0 - d2 * (1.0 / 6.0 - d2 * (1.0 / 120.0 - d2 * (1.0 / 5040.0))));
  const double nc = *c * cd - *s * sd;
  const double ns = *s * cd + *c * sd;
  const double k = 1.5 - 0.5 * (nc * nc + ns * ns);  // one Newton step back onto the unit circle
  *c = nc * k;
  *s = ns * k;
}
}  // namespace

void smooth_steering_3tap(Control* seq, int n) {
  if (n < 1) return;
  double prev = seq[0].steering_cmd;
  for (int t = 0; t < n; ++t) {
    const double cur = seq[t].steering_cmd;
    const double next = seq[t + 1 < n ? t + 1 : n - 1].steering_cmd;
    seq[t].steering_cmd = 0.25 * prev + 0.5 * cur + 0.25 * next;
    prev = cur;
  }
}

MppiSolver::MppiSolver(const MppiParams& params, const DynamicsModel& model,
                       const VehicleGeometry& geometry, int max_samples)
    : p_(params),
      model_(model),
      footprint_(geometry, params.corner_radius_m),
      half_width_m_(geometry.width_m * 0.5),
      wheelbase_m_(geometry.wheelbase_m),
      stop_decel_mps2_(geometry.stop_deceleration_mps2),
      steer_min_(geometry.steering_min_rad),
      steer_max_(geometry.steering_max_rad),
      horizon_(std::max(2, params.horizon_steps)),
      capacity_(std::min(std::max(max_samples, kMinSamples), kMaxSamples)),
      samples_(std::min(std::max(params.samples, kMinSamples), capacity_)),
      noise_(static_cast<std::size_t>(capacity_) * static_cast<std::size_t>(horizon_) * 2),
      u_(static_cast<std::size_t>(horizon_)),
      v_(static_cast<std::size_t>(capacity_) * static_cast<std::size_t>(horizon_)),
      cost_(static_cast<std::size_t>(capacity_)),
      scratch_(static_cast<std::size_t>(capacity_)),
      hit_(static_cast<std::size_t>(capacity_)),
      cls_(static_cast<std::size_t>(capacity_)),
      u_new_(static_cast<std::size_t>(horizon_)),
      plan_(static_cast<std::size_t>(horizon_) + 1) {
  GaussianRng rng(params.seed);
  for (double& n : noise_) n = rng.normal();
}

void MppiSolver::set_samples(int n) { samples_ = std::min(std::max(n, kMinSamples), capacity_); }

void MppiSolver::reset() {
  first_ = true;
  std::fill(u_.begin(), u_.end(), Control{});
}

MppiSolver::Ctx MppiSolver::make_ctx(const SolveInput& in) const {
  Ctx ctx;
  const double c = std::isfinite(in.confidence) ? clamp_double(in.confidence, 0.0, 1.0) : 0.0;
  ctx.use_track = c > 0.0 && in.track != nullptr && in.track->valid();
  ctx.c = ctx.use_track ? c : 0.0;
  ctx.use_free = ctx.c < 1.0 && in.freespace != nullptr && in.freespace->valid();
  if (ctx.use_track) {
    ctx.hint0 = in.track->locate(in.x0.x, in.x0.y);
    int h = ctx.hint0;
    ctx.s_start = in.track->project(in.x0.x, in.x0.y, &h).s;
  }
  if (ctx.use_free) {
    ctx.phi_start = in.freespace->phi(in.x0.x + wheelbase_m_ * std::cos(in.x0.yaw),
                                      in.x0.y + wheelbase_m_ * std::sin(in.x0.yaw));
  }
  return ctx;
}

double MppiSolver::evaluate(const SolveInput& in, const Ctx& ctx, const Control* u,
                            CostBreakdown* bd, Pose2* poses) const {
  const double dt = model_.dt();
  const DistanceField& field = *in.field;
  State s = in.x0;
  double cs = std::cos(s.yaw);
  double sn = std::sin(s.yaw);
  int hint = ctx.hint0;
  double j_clear = 0.0, j_bound = 0.0, j_lat = 0.0, j_du = 0.0, j_sr = 0.0;
  double s_end = ctx.s_start;
  double min_clear = 1e9;
  bool collided = false;
  int since_projection = 0;
  Pose2 last{s.x, s.y, s.yaw};  // pose at the last scored step: where the progress is measured
  Control prev = in.prev_cmd;
  if (poses != nullptr) poses[0] = last;
  for (int t = 0; t < horizon_; ++t) {
    const Control& ut = u[t];
    if (!collided) {
      const double dsp = ut.speed_cmd - prev.speed_cmd;
      const double dst = ut.steering_cmd - prev.steering_cmd;
      j_du += dsp * dsp;
      j_sr += dst * dst / dt;  // ((delta[t] - delta[t-1]) / dt)^2 * dt
    }
    prev = ut;
    const double v_before = s.v;
    const double yaw_before = s.yaw;
    double yaw_rate = 0.0;
    s = model_.step_trig(s, ut, cs, sn, &yaw_rate);
    rotate_cs(s.yaw, s.yaw - yaw_before, &cs, &sn);
    if (poses != nullptr) poses[t + 1] = Pose2{s.x, s.y, s.yaw};
    if (collided) {
      if (poses == nullptr) break;
      const double c = footprint_.clearance(field, s.x, s.y, cs, sn);
      min_clear = std::min(min_clear, c);
      continue;
    }
    const double clearance = footprint_.clearance(field, s.x, s.y, cs, sn);
    min_clear = std::min(min_clear, clearance);
    last = Pose2{s.x, s.y, s.yaw};
    ++since_projection;
    // The track is projected on every second step (and on the last), the boundary term of a
    // projected step covers the steps since the previous projection.
    const bool project = ctx.use_track && ((t & 1) == 1 || t == horizon_ - 1);
    Projection pr;
    if (project) {
      pr = in.track->project(s.x, s.y, &hint);
      s_end = pr.s;
    }
    if (clearance < p_.d_hard_m) {
      collided = true;  // progress is frozen at this step, the rest of the rollout is not scored
      if (poses == nullptr) break;
      continue;
    }
    const double d_soft = p_.d_soft_base_m + p_.t_headway_s * s.v;
    if (clearance < d_soft) {
      const double e = d_soft - clearance;
      j_clear += e * e * dt;
    }
    if (project) {
      const double side = pr.d >= 0.0 ? pr.half_left : pr.half_right;
      const double excess = std::fabs(pr.d) - (side - half_width_m_ - p_.m_bound_m);
      if (excess > 0.0) j_bound += excess * excess * dt * since_projection;
      since_projection = 0;
    }
    const double lat = std::fabs(v_before * yaw_rate);
    if (lat > p_.a_lat_max_mps2) {
      const double e = lat - p_.a_lat_max_mps2;
      j_lat += e * e * dt;
    }
  }
  const double c = ctx.c;
  const double progress_track = ctx.use_track ? s_end - ctx.s_start : 0.0;
  double phi_end = 0.0;
  double progress_free = 0.0;
  if (ctx.use_free) {
    phi_end = in.freespace->phi(last.x + wheelbase_m_ * std::cos(last.yaw),
                                last.y + wheelbase_m_ * std::sin(last.yaw));
    progress_free = ctx.phi_start - phi_end;
  }
  const double progress = c * progress_track + (1.0 - c) * progress_free;
  CostBreakdown local;
  CostBreakdown& o = bd != nullptr ? *bd : local;
  o.progress = -p_.w_progress * progress;
  o.collision = collided ? p_.c_collision : 0.0;
  o.clearance = p_.w_clear * j_clear;
  o.boundary = c * p_.w_bound * j_bound;
  o.lateral = p_.w_lat * j_lat;
  o.smoothness = p_.w_du * j_du;
  o.steer_rate = p_.w_steer_rate * j_sr;
  double terminal = 0.0;
  if (!collided) {
    const double sight = (ctx.use_track ? c * (in.track->s_valid_end() - s_end) : 0.0) +
                         (ctx.use_free ? (1.0 - c) * phi_end : 0.0);
    const double need = s.v * s.v / (2.0 * stop_decel_mps2_) - sight;
    if (need > 0.0) terminal = p_.w_term * need * need;
  }
  o.terminal = terminal;
  o.total = o.progress + o.collision + o.clearance + o.boundary + o.lateral + o.smoothness +
            o.steer_rate + o.terminal;
  o.s_end_m = s_end;
  o.progress_m = progress;
  o.min_clearance_m = min_clear;
  o.v_end_mps = s.v;
  o.collided = collided;
  return o.total;
}

double MppiSolver::evaluate_sequence(const SolveInput& in, const Control* seq,
                                     CostBreakdown* out) const {
  return evaluate(in, make_ctx(in), seq, out, nullptr);
}

void MppiSolver::build_samples(const SolveInput&, double v_cap) {
  const int T = horizon_;
  const int N = samples_;
  const double floor_mps = p_.speed_floor_mps;
  auto snap = [&](double v) { return snap_speed(v, floor_mps, v_cap); };
  // 0: the nominal, 1: the nominal steering at speed 0.
  {
    Control* v0 = &v_[0];
    Control* v1 = &v_[static_cast<std::size_t>(T)];
    for (int t = 0; t < T; ++t) {
      v0[t] = Control{snap(u_[t].speed_cmd), u_[t].steering_cmd};
      v1[t] = Control{0.0, u_[t].steering_cmd};
    }
  }
  // 2 to 28: constant command arcs, 9 steering values from the lower to the upper limit, speed at
  // the floor, the cap and their mean.
  const double speeds[kArcSpeedValues] = {snap(floor_mps), snap(v_cap),
                                          snap(0.5 * (floor_mps + v_cap))};
  for (int a = 0; a < kArcSteeringValues * kArcSpeedValues; ++a) {
    const int is = a / kArcSpeedValues;
    const double steer =
        steer_min_ + (steer_max_ - steer_min_) * static_cast<double>(is) / (kArcSteeringValues - 1);
    Control* vk = &v_[static_cast<std::size_t>(2 + a) * T];
    const Control c{speeds[a % kArcSpeedValues], steer};
    for (int t = 0; t < T; ++t) vk[t] = c;
  }
  // The rest: AR(1) noise around the nominal from the frozen table, the second half of the noise
  // samples being the negation of the first (antithetic).
  const double inno = std::sqrt(std::max(0.0, 1.0 - p_.beta * p_.beta));
  const int M = N - kStructuredSamples;
  const int h = M / 2;
  for (int j = 0; j < M; ++j) {
    int set = j;
    double sign = 1.0;
    if (j >= h && j < 2 * h) {
      set = j - h;
      sign = -1.0;
    }
    const double* nz = &noise_[static_cast<std::size_t>(set) * T * 2];
    Control* vk = &v_[static_cast<std::size_t>(kStructuredSamples + j) * T];
    double e_s = 0.0, e_d = 0.0;
    for (int t = 0; t < T; ++t) {
      const double n1 = sign * nz[2 * t];
      const double n2 = sign * nz[2 * t + 1];
      if (t == 0) {
        e_s = n1;
        e_d = n2;
      } else {
        e_s = p_.beta * e_s + inno * n1;
        e_d = p_.beta * e_d + inno * n2;
      }
      vk[t].speed_cmd = snap(u_[t].speed_cmd + p_.sigma_speed_mps * e_s);
      vk[t].steering_cmd =
          clamp_double(u_[t].steering_cmd + p_.sigma_steering_rad * e_d, steer_min_, steer_max_);
    }
  }
}

SolveOutput MppiSolver::solve(const SolveInput& in) {
  const int T = horizon_;
  const int N = samples_;
  const double v_cap = std::max(0.0, in.v_cap_mps);
  const Ctx ctx = make_ctx(in);
  if (first_) {
    for (auto& c : u_) c = Control{clamp_double(in.x0.v, 0.0, v_cap), 0.0};
    first_ = false;
  }
  for (auto& c : u_) {
    c.speed_cmd = clamp_double(c.speed_cmd, 0.0, v_cap);
    c.steering_cmd = clamp_double(c.steering_cmd, steer_min_, steer_max_);
  }
  build_samples(in, v_cap);

  double j_min = 1e300;
  int best = 0;
  int hits = 0;
  int clean = 0;
  for (int k = 0; k < N; ++k) {
    const Control* vk = &v_[static_cast<std::size_t>(k) * T];
    CostBreakdown bd;
    double j = evaluate(in, ctx, vk, &bd, nullptr);
    if (!std::isfinite(j)) {
      j = 1e9 + p_.c_collision;
      bd.collided = true;
    }
    cost_[k] = j;
    hit_[k] = bd.collided ? 1 : 0;
    hits += bd.collided ? 1 : 0;
    if (!bd.collided) scratch_[clean++] = j;
    if (j < j_min) {
      j_min = j;
      best = k;
    }
  }
  SolveOutput out;
  out.samples = N;
  out.best_cost = j_min;
  out.collision_fraction = static_cast<double>(hits) / N;
  out.all_collide = hits == N;
  if (out.all_collide) {
    const Control* vb = &v_[static_cast<std::size_t>(best) * T];
    for (int t = 0; t < T; ++t) u_new_[t] = Control{0.0, vb[t].steering_cmd};
  } else {
    // The samples fall in two classes: those that move (mean speed command over the first
    // plan_window_s at or above the floor) and those that stand. A weighted mean across the two
    // is neither (a car commanded half a floor speed does not move), so the mean is taken inside
    // the class of the best sample: the plan either moves or it does not.
    const int win =
        std::min(T, std::max(1, static_cast<int>(std::lround(p_.plan_window_s / model_.dt()))));
    const double thr = std::max(p_.speed_floor_mps, 1e-3);
    for (int k = 0; k < N; ++k) {
      const Control* vk = &v_[static_cast<std::size_t>(k) * T];
      double sum = 0.0;
      for (int t = 0; t < win; ++t) sum += vk[t].speed_cmd;
      cls_[k] = sum / win >= thr ? 1 : 0;
    }
    const unsigned char best_class = cls_[best];
    int in_class = 0;
    double j_min_c = 1e300;
    for (int k = 0; k < N; ++k) {
      if (hit_[k] || cls_[k] != best_class) continue;
      scratch_[in_class++] = cost_[k];
      j_min_c = std::min(j_min_c, cost_[k]);
    }
    // Robust temperature: the spread of the class sets the scale, lambda is a fraction of
    // (median - minimum) with a floor so a flat cost field does not give an arbitrary weighting.
    std::nth_element(scratch_.begin(), scratch_.begin() + in_class / 2,
                     scratch_.begin() + in_class);
    const double j_med = scratch_[static_cast<std::size_t>(in_class / 2)];
    const double scale = p_.lambda * std::max(j_med - j_min_c, p_.cost_scale_floor);
    for (auto& c : u_new_) c = Control{};
    double wsum = 0.0;
    for (int k = 0; k < N; ++k) {
      if (hit_[k] || cls_[k] != best_class) continue;
      const double w = std::exp(-(cost_[k] - j_min_c) / scale);
      wsum += w;
      const Control* vk = &v_[static_cast<std::size_t>(k) * T];
      for (int t = 0; t < T; ++t) {
        u_new_[t].speed_cmd += w * vk[t].speed_cmd;
        u_new_[t].steering_cmd += w * vk[t].steering_cmd;
      }
    }
    for (auto& c : u_new_) {
      c.speed_cmd /= wsum;
      c.steering_cmd /= wsum;
    }
    smooth_steering_3tap(u_new_.data(), T);
  }
  out.cmd = u_new_[0];
  {
    const int n =
        std::min(T, std::max(1, static_cast<int>(std::lround(p_.plan_window_s / model_.dt()))));
    double sum = 0.0;
    for (int t = 0; t < n; ++t) sum += u_new_[t].speed_cmd;
    out.plan_speed_mps = sum / n;
  }
  // The weighted mean rollout, for the plan topic and the status numbers.
  CostBreakdown pbd;
  evaluate(in, ctx, u_new_.data(), &pbd, plan_.data());
  out.plan_progress_m = pbd.progress_m;
  out.plan_min_clearance_m = pbd.min_clearance_m;
  // Warm start: advance the sequence by tick_s / dt of a model step with linear interpolation,
  // the last value held.
  const double f = clamp_double(p_.tick_s / model_.dt(), 0.0, 1.0);
  for (int t = 0; t + 1 < T; ++t) {
    u_[t].speed_cmd = (1.0 - f) * u_new_[t].speed_cmd + f * u_new_[t + 1].speed_cmd;
    u_[t].steering_cmd = (1.0 - f) * u_new_[t].steering_cmd + f * u_new_[t + 1].steering_cmd;
  }
  u_[T - 1] = u_new_[T - 1];
  return out;
}

}  // namespace racer_planner
