// mppi_bench: solve time of the MPPI solver on a synthetic corridor scan, with the dynamics bundle
// the car would load. Three scenes: the track fully trusted (confidence 1), the free-space field
// alone (confidence 0, no track) and both in between (confidence 0.5), plus the time to build the
// free-space field of a scan. Prints p50 / p90 / p99 / max in ms per scene and the sample count;
// exit code 1 when a scene's solve p99 is above --budget-ms (default 12, the Jetson budget) or the
// field build p99 is above --field-budget-ms (default 3).
// --background-threads N runs N busy threads for the length of the run, to measure the solve with
// the rest of the stack competing for cores.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "binding_geometry.hpp"
#include "gate_binding.hpp"
#include "racer_planner/bundle_loader.hpp"
#include "racer_planner/distance_field.hpp"
#include "racer_planner/freespace_field.hpp"
#include "racer_planner/gate_predictor.hpp"
#include "racer_planner/lap_profile.hpp"
#include "racer_planner/local_track.hpp"
#include "racer_planner/mppi.hpp"

using namespace racer_planner;

namespace {
void usage() {
  std::fprintf(stderr,
               "usage: mppi_bench --bundle DIR [--samples 512] [--solves 2000] [--horizon 30]\n"
               "                  [--speed 0.6] [--budget-ms 12] [--field-budget-ms 3]\n"
               "                  [--background-threads 0]\n");
}

struct Stats {
  double p50, p90, p99, max;
};

Stats stats_of(std::vector<double>& ms) {
  std::sort(ms.begin(), ms.end());
  auto pct = [&](double q) {
    return ms[std::min(ms.size() - 1,
                       static_cast<std::size_t>(q * static_cast<double>(ms.size())))];
  };
  return Stats{pct(0.5), pct(0.9), pct(0.99), ms.back()};
}

void print_stats(const char* what, const Stats& s, double budget_ms) {
  std::printf("%s solve ms: p50 %.3f  p90 %.3f  p99 %.3f  max %.3f  (budget p99 %.1f)\n", what,
              s.p50, s.p90, s.p99, s.max, budget_ms);
}
}  // namespace

int main(int argc, char** argv) {
  std::string bundle;
  int samples = 512, solves = 2000, horizon = 30, background_threads = 0;
  double speed = 0.6, budget_ms = 12.0, field_budget_ms = 3.0;
  for (int i = 1; i < argc; ++i) {
    auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : ""; };
    if (!std::strcmp(argv[i], "--bundle"))
      bundle = next();
    else if (!std::strcmp(argv[i], "--samples"))
      samples = std::atoi(next());
    else if (!std::strcmp(argv[i], "--solves"))
      solves = std::atoi(next());
    else if (!std::strcmp(argv[i], "--horizon"))
      horizon = std::atoi(next());
    else if (!std::strcmp(argv[i], "--speed"))
      speed = std::atof(next());
    else if (!std::strcmp(argv[i], "--budget-ms"))
      budget_ms = std::atof(next());
    else if (!std::strcmp(argv[i], "--field-budget-ms"))
      field_budget_ms = std::atof(next());
    else if (!std::strcmp(argv[i], "--background-threads"))
      background_threads = std::atoi(next());
    else {
      usage();
      return 2;
    }
  }
  if (bundle.empty() || solves < 1 || background_threads < 0 || background_threads > 256 ||
      samples < kMinSamples || samples > kMaxSamples) {
    usage();
    return 2;
  }
  std::string why;
  const auto geometry = geometry_from_binding(VEHICLE_PARAMS, &why);
  if (!geometry) {
    std::fprintf(stderr, "mppi_bench: %s\n", why.c_str());
    return 2;
  }
  const LoadResult loaded = load_dynamics_bundle(bundle, *geometry, speed);
  if (!loaded.ok()) {
    std::fprintf(stderr, "mppi_bench: %s: %s\n", refusal_code_name(loaded.refusal->code),
                 loaded.refusal->message.c_str());
    return 2;
  }
  const DynamicsParams& dyn = loaded.bundle->params;
  const DynamicsModel model(dyn, *geometry, dyn.dt_s);

  // Synthetic corridor, 1.0 m wide, closed 5 m ahead, a point every 2 cm, the car in the middle
  // looking down it.
  std::vector<PointXY> pts;
  for (double x = -1.0; x <= 5.0; x += 0.02) {
    pts.push_back({x, 0.5});
    pts.push_back({x, -0.5});
  }
  for (double y = -0.5; y <= 0.5; y += 0.02) pts.push_back({5.0, y});
  DistanceField field;
  field.build(pts.data(), pts.size());
  FreespaceField freespace(*geometry);
  if (!freespace.build(field, pts.data(), pts.size(), nullptr)) {
    std::fprintf(stderr, "mppi_bench: the synthetic scan gave no free-space field\n");
    return 3;
  }
  // The same corridor as a 720 beam scan from the LiDAR head, for the gate table and the profile
  // scenes: two side walls at y = +-0.5 and a wall across at x = 5 (base_link).
  const auto gate_cfg = gate_from_binding(VEHICLE_PARAMS, &why);
  if (!gate_cfg) {
    std::fprintf(stderr, "mppi_bench: %s\n", why.c_str());
    return 2;
  }
  std::vector<float> scan_ranges(720);
  for (int i = 0; i < 720; ++i) {
    const double b = -M_PI + i * (2.0 * M_PI / 720.0) + gate_cfg->geometry.lidar_mount_yaw_rad;
    const double dx = std::cos(b), dy = std::sin(b);
    const double ox = gate_cfg->geometry.lidar_mount_x_m, oy = gate_cfg->geometry.lidar_mount_y_m;
    double best = 1e9;
    for (double wy : {0.5, -0.5}) {
      const double t = (wy - oy) / dy;
      if (std::fabs(dy) > 1e-9 && t > 0.0) best = std::min(best, t);
    }
    if (std::fabs(dx) > 1e-9) {
      const double t = (5.0 - ox) / dx;
      if (t > 0.0) best = std::min(best, t);
    }
    scan_ranges[static_cast<std::size_t>(i)] = best < 12.0 ? static_cast<float>(best) : INFINITY;
  }
  GateScan gate_scan;
  gate_scan.ranges = scan_ranges.data();
  gate_scan.count = scan_ranges.size();
  gate_scan.angle_min_rad = -M_PI;
  gate_scan.angle_increment_rad = 2.0 * M_PI / 720.0;
  gate_scan.range_min_m = 0.05;
  gate_scan.range_max_m = 12.0;
  GateTable gate_table(gate_cfg->geometry, gate_cfg->limits);
  gate_table.build(gate_scan);
  // A lap profile: bound 1.0 m/s, 0.9 s per metre of best lap.
  LapProfile lap_profile;
  {
    std::vector<float> s, v, q;
    for (int i = 0; i < 61; ++i) {
      s.push_back(static_cast<float>(0.1 * i));
      v.push_back(1.0f);
      q.push_back(static_cast<float>(10.0 - 0.9 * 0.1 * i));
    }
    make_lap_profile(s.data(), v.data(), q.data(), s.size(), v.size(), q.size(), 0.0, 0.0,
                     &lap_profile);
  }

  LocalTrackData track;
  track.frame_id = "base_link";
  track.station_spacing_m = kTrackStationSpacingM;
  track.valid_count = kTrackStations;
  for (int k = 0; k < kTrackStations; ++k) {
    track.x_m.push_back(k * kTrackStationSpacingM);
    track.y_m.push_back(0.0);
    track.half_width_left_m.push_back(0.5);
    track.half_width_right_m.push_back(0.5);
    track.sigma_m.push_back(0.02);
  }
  TrackProjector projector;
  projector.build(track, Pose2{});

  std::atomic<bool> stop{false};
  std::vector<std::thread> busy;
  for (int i = 0; i < background_threads; ++i) {
    busy.emplace_back([&stop, i] {
      volatile double x = 1.0 + i;
      while (!stop.load(std::memory_order_relaxed)) {
        for (int k = 0; k < 10000; ++k) x = std::sin(x) * 1.0000001 + 0.5;
      }
    });
  }
  auto finish_threads = [&] {
    stop.store(true);
    for (auto& t : busy) t.join();
  };

  std::printf("mppi_bench: model %s, samples %d, horizon %d, solves %d, background threads %d\n",
              loaded.bundle->model_id.c_str(), samples, horizon, solves, background_threads);

  struct Scene {
    const char* name;
    double confidence;
    bool use_track;
    bool use_free;
    bool use_gate;
    bool use_lap;
  };
  const Scene scenes[] = {{"track c=1.0     ", 1.0, true, false, false, false},
                          {"field only c=0  ", 0.0, false, true, false, false},
                          {"blend c=0.5     ", 0.5, true, true, false, false},
                          {"track +gate term", 1.0, true, false, true, false},
                          {"track +gate +lap", 1.0, true, false, true, true},
                          {"blend +gate +lap", 0.5, true, true, true, true}};
  bool over_budget = false;
  double base_p50 = 0.0;
  for (const Scene& scene : scenes) {
    MppiParams params;
    params.horizon_steps = horizon;
    params.samples = samples;
    params.seed = 7;
    params.speed_floor_mps = 0.23;
    MppiSolver solver(params, model, *geometry, samples);
    SolveInput in;
    in.x0.v = 0.5 * speed;
    in.v_cap_mps = speed;
    in.field = &field;
    in.confidence = scene.confidence;
    in.track = scene.use_track ? &projector : nullptr;
    in.freespace = scene.use_free ? &freespace : nullptr;
    in.gate = scene.use_gate ? &gate_table : nullptr;
    in.wheel_speed_mps = 0.5 * speed;
    in.lap = scene.use_lap ? &lap_profile : nullptr;
    std::vector<double> ms;
    ms.reserve(static_cast<std::size_t>(solves));
    for (int i = 0; i < solves + 50; ++i) {
      // Wobble the state a little so the warm start has something to do.
      in.x0.y = 0.05 * std::sin(0.1 * i);
      in.x0.yaw = 0.1 * std::sin(0.07 * i);
      in.x0.delta = 0.1 * std::sin(0.13 * i);
      const auto t0 = std::chrono::steady_clock::now();
      const SolveOutput out = solver.solve(in);
      const double dt =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      if (!std::isfinite(out.cmd.speed_cmd)) {
        finish_threads();
        return 3;
      }
      if (i >= 50) ms.push_back(dt);  // the first 50 are warm up
    }
    const Stats s = stats_of(ms);
    print_stats(scene.name, s, budget_ms);
    if (!scene.use_gate && scene.confidence == 1.0) base_p50 = s.p50;
    if (scene.use_gate && scene.confidence == 1.0) {
      std::printf("  p50 against the same scene without the hooks: %+.3f ms\n", s.p50 - base_p50);
    }
    if (s.p99 > budget_ms) over_budget = true;
  }
  {
    // The gate table build: the scan thread's extra work per scan.
    std::vector<double> gate_ms;
    const int builds = std::max(200, solves / 5);
    for (int i = 0; i < builds + 20; ++i) {
      scan_ranges[static_cast<std::size_t>(i % 720)] = 1.0f + 0.001f * static_cast<float>(i % 7);
      const auto t0 = std::chrono::steady_clock::now();
      gate_table.build(gate_scan);
      const auto t1 = std::chrono::steady_clock::now();
      if (i >= 20) gate_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    const Stats gs = stats_of(gate_ms);
    std::printf("gate table      build ms: p50 %.3f  p90 %.3f  p99 %.3f  max %.3f  (720 beams)\n",
                gs.p50, gs.p90, gs.p99, gs.max);
  }

  // The free-space field of one scan (the distance field is built by the same callback, timed
  // separately).
  std::vector<double> field_ms, df_ms;
  const int builds = std::max(50, solves / 10);
  for (int i = 0; i < builds + 10; ++i) {
    // a slightly different scan every time so the work is not cached
    for (std::size_t k = 0; k < pts.size(); k += 7) pts[k].y += (i % 2 == 0 ? 0.001 : -0.001);
    const auto t0 = std::chrono::steady_clock::now();
    field.build(pts.data(), pts.size());
    const auto t1 = std::chrono::steady_clock::now();
    freespace.build(field, pts.data(), pts.size(), nullptr);
    const auto t2 = std::chrono::steady_clock::now();
    if (i >= 10) {
      df_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
      field_ms.push_back(std::chrono::duration<double, std::milli>(t2 - t1).count());
    }
  }
  finish_threads();
  const Stats ds = stats_of(df_ms);
  const Stats fs = stats_of(field_ms);
  std::printf("distance field  build ms: p50 %.3f  p90 %.3f  p99 %.3f  max %.3f\n", ds.p50, ds.p90,
              ds.p99, ds.max);
  std::printf(
      "free-space field build ms: p50 %.3f  p90 %.3f  p99 %.3f  max %.3f  (budget p99 %.1f)\n",
      fs.p50, fs.p90, fs.p99, fs.max, field_budget_ms);
  if (fs.p99 > field_budget_ms) over_budget = true;
  return over_budget ? 1 : 0;
}
