// Brief 43: a contract 1 bundle must behave bit-identically to main. These tests pin that.
//
// The numbers in the kStep*, kScene* and kEgo* tables were recorded from the code on main
// (commit cdd1e56, before step contract 2 existed) with RACER_PLANNER_RECORD=1, as exact hex
// floats, on aarch64 (GCC 11, -O2, the ros-dev image). On aarch64 they are compared exactly;
// another architecture contracts multiply-adds differently and is compared to 1e-9 (the same rule
// as MppiSolver.WithoutTheHooksTheSolverIsBitIdenticalToTheOneBeforeThem). The reference step in
// this file is a verbatim copy of main's DynamicsModel::step_trig, and is compared bit for bit on
// every architecture over random states.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "gate_test_support.hpp"
#include "racer_planner/bundle_loader.hpp"
#include "racer_planner/distance_field.hpp"
#include "racer_planner/dynamics_model.hpp"
#include "racer_planner/ego_motion.hpp"
#include "racer_planner/lap_profile.hpp"
#include "racer_planner/local_track.hpp"
#include "racer_planner/mppi.hpp"
#include "test_support.hpp"

using namespace racer_planner;

namespace {

bool recording() {
  const char* e = std::getenv("RACER_PLANNER_RECORD");
  return e != nullptr && std::string(e) == "1";
}

void expect_same(double got, double want, const std::string& what) {
#if defined(__aarch64__)
  EXPECT_EQ(got, want) << what;
#else
  EXPECT_NEAR(got, want, 1e-9 * std::max(1.0, std::fabs(want))) << what;
#endif
}

void print_hex_row(const char* tag, int idx, const std::vector<double>& row) {
  std::printf("RECORD %s %d", tag, idx);
  for (double v : row) std::printf(" %a", v);
  std::printf("\n");
}

// -- the reference step: main's DynamicsModel::step_trig, copied
// -----------------------------------

struct RefStep {
  const DynamicsParams& params;
  const VehicleGeometry& geometry;
  const DynamicsModel& model;  // for residual() only
  double dt;

  State operator()(const State& s, const Control& u) const {
    const NominalParams& n = params.nominal;
    const double a_d = 1.0 - std::exp(-dt / n.steer_tau_s);
    const double a_v = 1.0 - std::exp(-dt / n.speed_tau_s);
    const double rate_dt = geometry.steering_max_rate_rad_per_s * dt;
    const double accel_dt = n.accel_max_mps2 * dt;
    const double decel_dt = n.decel_max_mps2 * dt;
    const double g = u.steering_cmd >= 0.0 ? n.steer_gain_left : n.steer_gain_right;
    const double delta_target = clamp_double(g * u.steering_cmd + n.steer_offset_rad,
                                             geometry.steering_min_rad, geometry.steering_max_rad);
    const double d_delta = clamp_double(a_d * (delta_target - s.delta), -rate_dt, rate_dt);
    const double target =
        std::fabs(u.speed_cmd) < n.speed_deadband_mps ? 0.0 : n.speed_gain * u.speed_cmd;
    double dv = clamp_double(a_v * (target - s.v), -decel_dt, accel_dt);
    double yaw_rate =
        s.v * std::tan(s.delta) / geometry.wheelbase_m / (1.0 + n.understeer_s2_per_m2 * s.v * s.v);
    if (params.residual.enabled) {
      double r[kMlpOut];
      model.residual(s.v, s.delta, u.speed_cmd, u.steering_cmd, r);
      dv += r[0] * dt;
      yaw_rate += r[1];
    }
    State out;
    out.x = s.x + s.v * std::cos(s.yaw) * dt;
    out.y = s.y + s.v * std::sin(s.yaw) * dt;
    out.yaw = s.yaw + yaw_rate * dt;
    out.v = s.v + dv;
    if (out.v < 0.0) out.v = 0.0;
    out.delta = s.delta + d_delta;
    return out;
  }
};

struct Contract1Model {
  VehicleGeometry g = test::fixture_geometry();
  DynamicsParams params;
  Contract1Model(bool residual) {
    const auto r = load_dynamics_bundle(test::fixture_dir(), g, 0.5);
    EXPECT_TRUE(r.ok()) << (r.refusal ? r.refusal->message : "");
    params = r.bundle->params;
    if (!residual) {
      params.residual.enabled = false;
    }
  }
};

std::array<double, 5> as_array(const State& s) { return {s.x, s.y, s.yaw, s.v, s.delta}; }

bool same_bits(double a, double b) {
  std::uint64_t x, y;
  std::memcpy(&x, &a, 8);
  std::memcpy(&y, &b, 8);
  return x == y;
}

}  // namespace

TEST(Contract1Identity, StepEqualsMainsStepBitForBitOnRandomStates) {
  for (const bool residual : {true, false}) {
    Contract1Model m(residual);
    const DynamicsModel model(m.params, m.g, m.params.dt_s);
    const DynamicsModel model_tick(m.params, m.g, 0.02);
    const RefStep ref{m.params, m.g, model, m.params.dt_s};
    const RefStep ref_tick{m.params, m.g, model_tick, 0.02};
    std::mt19937_64 rng(77 + residual);
    auto u = [&](double lo, double hi) {
      return std::uniform_real_distribution<double>(lo, hi)(rng);
    };
    for (int i = 0; i < 100000; ++i) {
      State s{u(-3, 3), u(-3, 3), u(-7, 7), u(0.0, 1.4), u(-0.45, 0.45)};
      Control c{u(-0.5, 1.5), u(-0.6, 0.6)};
      if (i % 11 == 0) c.speed_cmd = u(0.0, 0.3);  // around the dead zone
      const auto a = as_array(model.step(s, c));
      const auto b = as_array(ref(s, c));
      const auto a2 = as_array(model_tick.step(s, c));
      const auto b2 = as_array(ref_tick(s, c));
      for (int k = 0; k < 5; ++k) {
        ASSERT_TRUE(same_bits(a[k], b[k])) << "residual " << residual << " i " << i << " k " << k;
        ASSERT_TRUE(same_bits(a2[k], b2[k]));
      }
    }
  }
}

namespace {

// -- solver scenes
// ---------------------------------------------------------------------------------

constexpr int kT = 30;

struct SceneEnv {
  VehicleGeometry g;
  DynamicsParams dyn;
  DynamicsModel model;
  DistanceField field;
  TrackProjector track;
  LocalTrackData data;
  LapProfile lap;
  GateConfig gate_cfg = test::binding_gate();
  GateTable table{gate_cfg.geometry, gate_cfg.limits};

  SceneEnv(const VehicleGeometry& geometry, const DynamicsParams& params)
      : g(geometry), dyn(params), model(dyn, g, dyn.dt_s) {
    data.frame_id = "base_link";
    data.station_spacing_m = kTrackStationSpacingM;
    data.valid_count = kTrackStations;
    for (int k = 0; k < kTrackStations; ++k) {
      data.x_m.push_back(k * kTrackStationSpacingM);
      data.y_m.push_back(0.0);
      data.half_width_left_m.push_back(0.5);
      data.half_width_right_m.push_back(0.5);
      data.sigma_m.push_back(0.02);
    }
    track.build(data, Pose2{});
    lap.active = true;
    lap.stamp_s = 0.0;
    lap.s0_m = 4.0;
    lap.n = 61;
    for (std::size_t i = 0; i < lap.n; ++i) {
      lap.v_bound[i] = i < 25 ? 0.9 : 0.55;  // a slow zone from 2.5 m on
      lap.q[i] = 9.0 - 1.1 * kLapStationSpacingM * static_cast<double>(i);
    }
  }
};

struct Row {
  std::vector<double> v;
};

std::vector<double> solve_row(const SolveOutput& o) {
  return {o.cmd.speed_cmd,        o.cmd.steering_cmd, o.best_cost,         o.plan_progress_m,
          o.plan_min_clearance_m, o.plan_speed_mps,   o.collision_fraction};
}

std::vector<double> bd_row(const CostBreakdown& b) {
  return {b.total,    b.clearance, b.lateral,  b.terminal, b.lap_trust,  b.lap_terminal,
          b.gate,     b.boundary,  b.progress, b.s_end_m,  b.progress_m, b.min_clearance_m,
          b.v_end_mps};
}

constexpr int kScenes = 4;
constexpr int kSolves = 3;

// Returns kSolves solve rows then one evaluate_sequence row, per scene.
std::vector<std::vector<double>> scene_rows(int scene) {
  Contract1Model resid(true);
  Contract1Model plain(false);
  const bool use_resid = scene != 3;
  SceneEnv e(use_resid ? resid.g : plain.g, use_resid ? resid.params : plain.params);
  if (scene == 3) {
    e.dyn.nominal.speed_deadband_mps = 0.22;  // a dead zone, no residual
    e.model = DynamicsModel(e.dyn, e.g, e.dyn.dt_s);
  }
  std::vector<PointXY> pts;
  for (double x = -1.0; x <= 5.0; x += 0.02) {
    pts.push_back({x, 0.5});
    pts.push_back({x, -0.5});
  }
  if (scene == 0 || scene == 2) pts.push_back({1.1, 0.2});  // an obstacle
  e.field.build(pts.data(), pts.size());
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 256;
  p.speed_floor_mps = 0.2;
  p.seed = 21 + static_cast<std::uint64_t>(scene);
  p.w_gate = scene >= 2 ? 300.0 : 0.0;
  p.w_lat = 4.0;
  SolveInput in;
  in.x0.v = 0.5;
  in.x0.delta = 0.04;
  in.x0.y = 0.03;
  in.v_cap_mps = 0.9;
  in.track = &e.track;
  in.field = &e.field;
  in.prev_cmd = Control{0.5, 0.04};
  in.wheel_speed_mps = 0.5;
  if (scene >= 1) {
    in.lap = &e.lap;
    in.lap_station_now_m = 0.3;
  }
  if (scene >= 2) {
    e.table.build(test::scan_of(e.gate_cfg.geometry, test::wall_across_road(1.6)).view());
    in.gate = &e.table;
    in.gate_travel_since_scan_m = 0.04;
  }
  MppiSolver s(p, e.model, e.g, 256);
  std::vector<std::vector<double>> rows;
  for (int i = 0; i < kSolves; ++i) rows.push_back(solve_row(s.solve(in)));
  std::vector<Control> seq(kT, Control{0.6, 0.12});
  CostBreakdown bd;
  s.evaluate_sequence(in, seq.data(), &bd);
  rows.push_back(bd_row(bd));
  return rows;
}

// Recorded on main, aarch64. Scene by scene: kSolves solve rows (speed cmd, steering cmd, best
// cost, plan progress, plan min clearance, plan speed, collision fraction), then the breakdown row
// of a fixed sequence (total, clearance, lateral, terminal, lap trust, lap terminal, gate,
// boundary, progress, s_end, progress m, min clearance, v_end).
using Table = std::vector<std::vector<double>>;
// RECORDED-SOLVER-TABLE-BEGIN
const Table kSolverTable = {
    {0x1.f697962d6754cp-2, -0x1.a0939ea2a6192p-4, -0x1.7ad8e9bd4b396p+2, 0x1.779df495f20fdp-1,
     0x1.3957aac25335bp-3, 0x1.1abe74b0f7ee3p-1, 0x1.f8p-2},
    {0x1.2b69af3d68002p-1, -0x1.2769e06a6a6cap-3, -0x1.a59090c1b065p+2, 0x1.938fc6c870c89p-1,
     0x1.5c424fe28255cp-3, 0x1.46dac6a8971fdp-1, 0x1.8p-4},
    {0x1.52af0712c90a6p-1, -0x1.ec686b47e6019p-4, -0x1.bc71104072fcfp+2, 0x1.9ffdb42b24d3ap-1,
     0x1.56a30e9750d71p-3, 0x1.63935b9d80bb1p-1, 0x1.38p-3},
    {0x1.3860fed289f58p+13, 0x1.413d00e8ecdc1p+1, 0x0p+0, 0x0p+0, 0x0p+0, 0x0p+0, 0x0p+0, 0x0p+0,
     -0x1.99cb8ec141c76p+2, 0x1.47d60bcdce392p-1, 0x1.47d60bcdce392p-1, 0x1.5b2c0319e3c8p-8,
     0x1.1929d6357d8dep-1},
    {0x1.86784ab87d1d9p-1, -0x1.5664dbb8306d7p-9, -0x1.4d84acda718c5p+3, 0x1.013b188b5a002p+0,
     0x1.3d15bba5a2ba7p-2, 0x1.8aaa0c51212b3p-1, 0x1.ap-5},
    {0x1.992633a35b3abp-1, -0x1.39f7d9d99b1a8p-6, -0x1.4de40f951985bp+3, 0x1.13eec0f3b3c86p+0,
     0x1.3f29d5226215ep-2, 0x1.ae4e7443a182ap-1, 0x1.4p-4},
    {0x1.9fa2753c2bb84p-1, -0x1.c55689683e649p-8, -0x1.5394296abe86p+3, 0x1.1a35201791a5fp+0,
     0x1.3ed3e867c7bfp-2, 0x1.b6584dde6903dp-1, 0x1.7p-4},
    {-0x1.42c27a676986dp+2, 0x1.7c223314f2899p+1, 0x0p+0, 0x0p+0, 0x0p+0, -0x1.00fb9b472d33p+3,
     0x0p+0, 0x0p+0, 0x0p+0, 0x1.98c1e4b7ac157p-1, 0x1.98c1e4b7ac157p-1, 0x1.3f3f80f6349dap-4,
     0x1.1941acd015e71p-1},
    {0x1.5fa98c7996a81p-1, -0x1.6c5a24c5f2018p-4, -0x1.936e3294cb348p+2, 0x1.a0e9a06690221p-1,
     0x1.33c66cdafc088p-3, 0x1.584b482f1ba76p-1, 0x1.f4p-2},
    {0x1.74cb1e6fc5f0ap-1, -0x1.e8f6f56dc00f6p-4, -0x1.b390c9b735d71p+2, 0x1.a596fde26ee3fp-1,
     0x1.5510f64920629p-3, 0x1.7209b16121c11p-1, 0x1.5p-3},
    {0x1.74f909dcdb9d8p-1, -0x1.1b833f4610562p-3, -0x1.c93fd7b47ac2fp+2, 0x1.a1bc2bf70769cp-1,
     0x1.56727cfe535a4p-3, 0x1.76077ede2842fp-1, 0x1.48p-3},
    {0x1.385ed953789b7p+13, 0x1.413d00e8ecdc1p+1, 0x0p+0, 0x0p+0, 0x0p+0, -0x1.aaf7874c128cep+2,
     0x0p+0, 0x0p+0, 0x0p+0, 0x1.47d60bcdce392p-1, 0x1.47d60bcdce392p-1, 0x1.5b2c0319e3c8p-8,
     0x1.1929d6357d8dep-1},
    {0x1.83660cbaa6892p-1, -0x1.024a90cfc96b4p-8, -0x1.50afef697b616p+3, 0x1.0660232247814p+0,
     0x1.3940b1ebee03dp-2, 0x1.898a1cfdbee8dp-1, 0x1.ap-5},
    {0x1.999855b501bbcp-1, -0x1.25f4f33e8c70fp-7, -0x1.5ceafdec2684p+3, 0x1.19b6393017da2p+0,
     0x1.3f9531ff8f56ep-2, 0x1.ad5b302cfb9fap-1, 0x1.cp-4},
    {0x1.b2341a0a40c91p-1, -0x1.6fb42f71339d1p-7, -0x1.58637777827bcp+3, 0x1.2007070c57351p+0,
     0x1.3fd606af2bb2dp-2, 0x1.b9d41dc8234ep-1, 0x1.fp-4},
    {-0x1.3085996b7b6e6p+2, 0x1.bfba1ede86767p+1, 0x0p+0, 0x0p+0, 0x0p+0, -0x1.08c325bb9b22p+3,
     0x0p+0, 0x0p+0, 0x0p+0, 0x1.a4bc19504c8e3p-1, 0x1.a4bc19504c8e3p-1, 0x1.15179b029835p-4,
     0x1.23c04c1f1af4p-1},
};
// RECORDED-SOLVER-TABLE-END

// -- dead reckoning
// ---------------------------------------------------------------------------------

Table ego_rows() {
  Table rows;
  for (const double understeer : {0.0, 0.11}) {
    EgoMotion e(0.31, understeer);
    for (int i = 0; i < 120; ++i) {
      const double t = 0.02 * i;
      const double v = i < 60 ? 0.4 + 0.01 * i : -0.3 + 0.012 * (i - 60);  // reverses
      const double d = 0.3 * std::sin(0.07 * i);
      e.push(t, v, d);
    }
    for (const auto& ft : std::vector<std::pair<double, double>>{
             {0.1, 0.3}, {0.3, 0.1}, {0.5, 1.1}, {1.2, 1.9}, {-0.1, 0.4}, {2.0, 2.6}}) {
      bool covered = false;
      const Pose2 p = e.relative_pose(ft.first, ft.second, &covered);
      rows.push_back(
          {p.x, p.y, p.yaw, covered ? 1.0 : 0.0, e.distance_between(ft.first, ft.second)});
    }
  }
  return rows;
}
// RECORDED-EGO-TABLE-BEGIN
const Table kEgoTable = {
    {0x1.954fb1daa5a0fp-4, 0x1.494d8f4b503a8p-9, 0x1.e8488d91386ecp-5, 0x1p+0,
     0x1.95810624dd2f1p-4},
    {-0x1.95345d7442f2dp-4, 0x1.bbe5c50f2c6b2p-9, -0x1.e8488d91386ecp-5, 0x1p+0,
     -0x1.95810624dd2f1p-4},
    {0x1.e3eab6eab1ebap-2, 0x1.f178fd64e2ff2p-5, 0x1.e2248c9c10943p-4, 0x1p+0,
     0x1.e872b020c49bcp-2},
    {-0x1.1313b35c32eeap-4, -0x1.108336557119ep-9, 0x1.26e592f474b34p-4, 0x1p+0,
     -0x1.13404ea4a8c17p-4},
    {0x1.e6cc5e4d89ccp-3, 0x1.1127a20f447c3p-7, 0x1.e2560909a6b25p-4, 0x0p+0, 0x1.e76c8b4395811p-3},
    {0x1.95c0ecc3bf7a8p-3, 0x1.23a974fd62503p-6, 0x1.6fbb04b6b1d75p-3, 0x1p+0,
     0x1.97f62b6ae7d57p-3},
    {0x1.95523942f44fp-4, 0x1.40ee5ae7398ecp-9, 0x1.daf91727f34bap-5, 0x1p+0, 0x1.95810624dd2f1p-4},
    {-0x1.9538a448d6a16p-4, 0x1.af347ae00422ap-9, -0x1.daf91727f34bap-5, 0x1p+0,
     -0x1.95810624dd2f1p-4},
    {0x1.e45fac5328086p-2, 0x1.d81b3c06ec4c9p-5, 0x1.cebd6ed0dc9edp-4, 0x1p+0,
     0x1.e872b020c49bcp-2},
    {-0x1.131453e23d6bep-4, -0x1.0e858967abddcp-9, 0x1.255cc0ea7c47bp-4, 0x1p+0,
     -0x1.13404ea4a8c17p-4},
    {0x1.e6d5344f1e914p-3, 0x1.09cda008123cap-7, 0x1.d3c77708bd136p-4, 0x0p+0,
     0x1.e76c8b4395811p-3},
    {0x1.95ceacd12fe4ap-3, 0x1.2055f3e4c4f07p-6, 0x1.6a9dcece66ba1p-3, 0x1p+0,
     0x1.97f62b6ae7d57p-3},
};
// RECORDED-EGO-TABLE-END

// -- model step goldens
// -----------------------------------------------------------------------------

Table step_rows() {
  Table rows;
  for (const bool residual : {true, false}) {
    Contract1Model m(residual);
    const DynamicsModel model(m.params, m.g, m.params.dt_s);
    const State states[] = {{0, 0, 0, 0.0, 0.0},
                            {0.1, -0.2, 0.7, 0.6, 0.15},
                            {1, 2, -2.0, 1.1, -0.3},
                            {0, 0, 0.1, 0.25, 0.4},
                            {0, 0, 0, 0.9, 0.0}};
    const Control ctls[] = {{0.8, 0.2}, {0.0, -0.4}, {0.15, 0.0}, {1.2, 0.5}, {0.6, -0.1}};
    for (const State& s : states) {
      for (const Control& c : ctls) {
        const State o = model.step(s, c);
        rows.push_back({o.x, o.y, o.yaw, o.v, o.delta});
      }
    }
  }
  return rows;
}
// RECORDED-STEP-TABLE-BEGIN
const Table kStepTable = {
    {0x0p+0, 0x0p+0, -0x1.873141e830c1ap-12, 0x1.9491cb4e59b82p-4, 0x1.72adfa028710dp-4},
    {0x0p+0, 0x0p+0, -0x1.0b8bc183d9e92p-11, 0x1.7c4a746dbe67cp-10, -0x1.1a09f04fd6126p-3},
    {0x0p+0, 0x0p+0, -0x1.c2f5898d942fcp-12, 0x1.b467b09dc7135p-6, 0x1.01dd337c316fep-8},
    {0x0p+0, 0x0p+0, -0x1.2693183f2609cp-12, 0x1.90aa12d84060fp-4, 0x1.47ae147ae147cp-3},
    {0x0p+0, 0x0p+0, -0x1.02eeae531b6d9p-11, 0x1.97b4c852ef849p-4, -0x1.01dd337c316fep-5},
    {0x1.f795747079676p-4, -0x1.7204ee9208c46p-3, 0x1.6d574b811a14p-1, 0x1.3fcf8f5157c65p-1,
     0x1.73aa80123f8f3p-3},
    {0x1.f795747079676p-4, -0x1.7204ee9208c46p-3, 0x1.6d497fbc40abep-1, 0x1.0bf190dbb848p-1,
     -0x1.47ae147ae149p-7},
    {0x1.f795747079676p-4, -0x1.7204ee9208c46p-3, 0x1.6d56a668e65b4p-1, 0x1.0b81614d656dep-1,
     0x1.84c4d959bb248p-4},
    {0x1.f795747079676p-4, -0x1.7204ee9208c46p-3, 0x1.6d5538711ba3bp-1, 0x1.62d0f077eff64p-1,
     0x1.12122474057f1p-2},
    {0x1.f795747079676p-4, -0x1.7204ee9208c46p-3, 0x1.6d4fbfdb5e85dp-1, 0x1.2e7567941e84ap-1,
     0x1.e770d8c7beab4p-5},
    {0x1.f448039c22f6dp-1, 0x1.f33274a2eb69p+0, -0x1.06300200f66c9p+1, 0x1.09158e3d2643ap+0,
     -0x1.1eb851eb851eap-3},
    {0x1.f448039c22f6dp-1, 0x1.f33274a2eb69p+0, -0x1.0637cfdd134d6p+1, 0x1.065b0fd56f6dap+0,
     -0x1.47587b38e70ffp-2},
    {0x1.f448039c22f6dp-1, 0x1.f33274a2eb69p+0, -0x1.0632198b112fep+1, 0x1.0634544ab3fe9p+0,
     -0x1.6c981c861682p-3},
    {0x1.f448039c22f6dp-1, 0x1.f33274a2eb69p+0, -0x1.062ff50f7aff1p+1, 0x1.1a6f020b4c391p+0,
     -0x1.1eb851eb851eap-3},
    {0x1.f448039c22f6dp-1, 0x1.f33274a2eb69p+0, -0x1.0633aac935cc6p+1, 0x1.05d8fdeead186p+0,
     -0x1.b51e530104698p-3},
    {0x1.978dbfaecdfcep-7, 0x1.472256fa308fp-10, 0x1.da91ede826f07p-4, 0x1.5a62579b7f1efp-2,
     0x1.551ad7ec9c77fp-2},
    {0x1.978dbfaecdfcep-7, 0x1.472256fa308fp-10, 0x1.da4a615871e94p-4, 0x1.9ebf79654b0b6p-3,
     0x1.eb851eb851eb8p-3},
    {0x1.978dbfaecdfcep-7, 0x1.472256fa308fp-10, 0x1.da96c8eec2755p-4, 0x1.d26288af2aaccp-3,
     0x1.f8ed9c73d6f2fp-3},
    {0x1.978dbfaecdfcep-7, 0x1.472256fa308fp-10, 0x1.daab77a2006e6p-4, 0x1.61b005e7e7dd2p-2,
     0x1.ad57bc57822f6p-2},
    {0x1.978dbfaecdfcep-7, 0x1.472256fa308fp-10, 0x1.da4991fb5a778p-4, 0x1.377209673c0c1p-2,
     0x1.eb851eb851eb8p-3},
    {0x1.70a3d70a3d70bp-5, 0x0p+0, 0x1.de409b11499a8p-13, 0x1.bd8cf70f0d346p-1,
     0x1.72adfa028710dp-4},
    {0x1.70a3d70a3d70bp-5, 0x0p+0, 0x1.bd4ed3241eb9cp-15, 0x1.a5b32e1154d47p-1,
     -0x1.1a09f04fd6126p-3},
    {0x1.70a3d70a3d70bp-5, 0x0p+0, 0x1.8f54345fb5d35p-13, 0x1.a544295fd04fcp-1,
     0x1.01dd337c316fep-8},
    {0x1.70a3d70a3d70bp-5, 0x0p+0, 0x1.9c7d41f45ed08p-13, 0x1.e07599df949fp-1,
     0x1.47ae147ae147cp-3},
    {0x1.70a3d70a3d70bp-5, 0x0p+0, 0x1.45be48f2fabfp-13, 0x1.ac462414b0a54p-1,
     -0x1.01dd337c316fep-5},
    {0x0p+0, 0x0p+0, 0x0p+0, 0x1.999999999999ap-4, 0x1.72adfa028710dp-4},
    {0x0p+0, 0x0p+0, 0x0p+0, 0x0p+0, -0x1.1a09f04fd6126p-3},
    {0x0p+0, 0x0p+0, 0x0p+0, 0x1.a73682fc2e7efp-6, 0x1.01dd337c316fep-8},
    {0x0p+0, 0x0p+0, 0x0p+0, 0x1.999999999999ap-4, 0x1.47ae147ae147cp-3},
    {0x0p+0, 0x0p+0, 0x0p+0, 0x1.999999999999ap-4, -0x1.01dd337c316fep-5},
    {0x1.f795747079676p-4, -0x1.7204ee9208c46p-3, 0x1.6d4e5bb7e0abp-1, 0x1.420cb10f22dddp-1,
     0x1.73aa80123f8f3p-3},
    {0x1.f795747079676p-4, -0x1.7204ee9208c46p-3, 0x1.6d4e5bb7e0abp-1, 0x1.0ccccccccccccp-1,
     -0x1.47ae147ae149p-7},
    {0x1.f795747079676p-4, -0x1.7204ee9208c46p-3, 0x1.6d4e5bb7e0abp-1, 0x1.0ccccccccccccp-1,
     0x1.84c4d959bb248p-4},
    {0x1.f795747079676p-4, -0x1.7204ee9208c46p-3, 0x1.6d4e5bb7e0abp-1, 0x1.65513bf97c131p-1,
     0x1.12122474057f1p-2},
    {0x1.f795747079676p-4, -0x1.7204ee9208c46p-3, 0x1.6d4e5bb7e0abp-1, 0x1.306a6b99f6433p-1,
     0x1.e770d8c7beab4p-5},
    {0x1.f448039c22f6dp-1, 0x1.f33274a2eb69p+0, -0x1.06380bdb7532dp+1, 0x1.09d283dfeaf45p+0,
     -0x1.1eb851eb851eap-3},
    {0x1.f448039c22f6dp-1, 0x1.f33274a2eb69p+0, -0x1.06380bdb7532dp+1, 0x1.0666666666667p+0,
     -0x1.47587b38e70ffp-2},
    {0x1.f448039c22f6dp-1, 0x1.f33274a2eb69p+0, -0x1.06380bdb7532dp+1, 0x1.0666666666667p+0,
     -0x1.6c981c861682p-3},
    {0x1.f448039c22f6dp-1, 0x1.f33274a2eb69p+0, -0x1.06380bdb7532dp+1, 0x1.1b74c955178efp+0,
     -0x1.1eb851eb851eap-3},
    {0x1.f448039c22f6dp-1, 0x1.f33274a2eb69p+0, -0x1.06380bdb7532dp+1, 0x1.0666666666667p+0,
     -0x1.b51e530104698p-3},
    {0x1.978dbfaecdfcep-7, 0x1.472256fa308fp-10, 0x1.daf3fafcefca8p-4, 0x1.5eaa825a17dfcp-2,
     0x1.551ad7ec9c77fp-2},
    {0x1.978dbfaecdfcep-7, 0x1.472256fa308fp-10, 0x1.daf3fafcefca8p-4, 0x1.a330ad6166159p-3,
     0x1.eb851eb851eb8p-3},
    {0x1.978dbfaecdfcep-7, 0x1.472256fa308fp-10, 0x1.daf3fafcefca8p-4, 0x1.d8177dc0ebe57p-3,
     0x1.f8ed9c73d6f2fp-3},
    {0x1.978dbfaecdfcep-7, 0x1.472256fa308fp-10, 0x1.daf3fafcefca8p-4, 0x1.6666666666666p-2,
     0x1.ad57bc57822f6p-2},
    {0x1.978dbfaecdfcep-7, 0x1.472256fa308fp-10, 0x1.daf3fafcefca8p-4, 0x1.3b65f76fbeaa8p-2,
     0x1.eb851eb851eb8p-3},
    {0x1.70a3d70a3d70bp-5, 0x0p+0, 0x0p+0, 0x1.bfce7eac5b178p-1, 0x1.72adfa028710dp-4},
    {0x1.70a3d70a3d70bp-5, 0x0p+0, 0x0p+0, 0x1.a666666666666p-1, -0x1.1a09f04fd6126p-3},
    {0x1.70a3d70a3d70bp-5, 0x0p+0, 0x0p+0, 0x1.a666666666666p-1, 0x1.01dd337c316fep-8},
    {0x1.70a3d70a3d70bp-5, 0x0p+0, 0x0p+0, 0x1.e3130996b44ccp-1, 0x1.47ae147ae147cp-3},
    {0x1.70a3d70a3d70bp-5, 0x0p+0, 0x0p+0, 0x1.ae2c39372e7cep-1, -0x1.01dd337c316fep-5},
};
// RECORDED-STEP-TABLE-END

void compare_table(const char* tag, const Table& got, const Table& want) {
  if (recording()) {
    for (std::size_t i = 0; i < got.size(); ++i) print_hex_row(tag, static_cast<int>(i), got[i]);
    return;
  }
  ASSERT_EQ(got.size(), want.size()) << tag << " table is not recorded";
  for (std::size_t i = 0; i < got.size(); ++i) {
    ASSERT_EQ(got[i].size(), want[i].size());
    for (std::size_t f = 0; f < got[i].size(); ++f) {
      expect_same(got[i][f], want[i][f],
                  std::string(tag) + " row " + std::to_string(i) + " field " + std::to_string(f));
    }
  }
}

}  // namespace

TEST(Contract1Identity, SolverMatchesTheNumbersRecordedOnMain) {
  Table got;
  for (int sc = 0; sc < kScenes; ++sc) {
    for (auto& r : scene_rows(sc)) got.push_back(std::move(r));
  }
  compare_table("solver", got, kSolverTable);
}

TEST(Contract1Identity, DeadReckoningMatchesTheNumbersRecordedOnMain) {
  compare_table("ego", ego_rows(), kEgoTable);
}

TEST(Contract1Identity, StepMatchesTheNumbersRecordedOnMain) {
  compare_table("step", step_rows(), kStepTable);
}
