// mppi_bench: solve time of the MPPI solver on a synthetic corridor scan, with the dynamics bundle
// the car would load. Three scenes: the track fully trusted (confidence 1), the free-space field
// alone (confidence 0, no track) and both in between (confidence 0.5), plus the time to build the
// free-space field of a scan. Prints p50 / p90 / p99 / max in ms per scene and the sample count;
// exit code 1 when a scene's solve p99 is above --budget-ms (default 12, the Jetson budget) or the
// field build p99 is above --field-budget-ms (default 3).
// --background-threads N (or --background-load: one per CPU) runs busy threads for the length of
// the run, to measure the solve with
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
#include "racer_planner/bundle_loader.hpp"
#include "racer_planner/distance_field.hpp"
#include "racer_planner/freespace_field.hpp"
#include "racer_planner/local_track.hpp"
#include "racer_planner/mppi.hpp"
#include "racer_planner/planner.hpp"

using namespace racer_planner;

namespace {
void usage() {
  std::fprintf(stderr,
               "usage: mppi_bench --bundle DIR [--samples 512] [--solves 2000] [--horizon 30]\n"
               "                  [--speed 0.6] [--budget-ms 12] [--field-budget-ms 3]\n"
               "                  [--background-threads 0 | --background-load]\n");
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
    else if (!std::strcmp(argv[i], "--background-load"))
      background_threads = static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
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
  };
  const Scene scenes[] = {{"track c=1.0     ", 1.0, true, false},
                          {"field only c=0  ", 0.0, false, true},
                          {"blend c=0.5     ", 0.5, true, true}};
  bool over_budget = false;
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
    if (s.p99 > budget_ms) over_budget = true;
  }

  // The whole planner pipeline on the same corridor: tick() (solve and output in one call, what the
  // planner has always had) and, on the code with the solve / publish split, the two halves apart:
  // solve() and publish(), the publish path being what the node's 50 Hz publisher timer runs.
  {
    PlannerConfig pc;
    pc.logic.v_max_mps = speed;
#ifndef RACER_BENCH_OLD
    pc.logic.require_gated_for_start = false;
    pc.logic.start_ticks = 5;
#endif
    pc.mppi.samples = samples;
    pc.mppi.seed = 7;
    pc.max_samples = samples;
    Planner planner(pc, dyn, *geometry);
    auto make_input = [&](double now) {
      track.stamp_s = now - 0.1;
      TickInput in;
      in.now_s = now;
      in.has_odom = true;
      in.odom_stamp_s = now;
      in.wheel_speed_mps = 0.5 * speed;
      in.has_scan = true;
      in.scan_stamp_s = now - 0.1;
      in.field = &field;
      in.freespace = &freespace;
      in.track = &track;
      in.has_gated = true;
      in.gated_stamp_s = now;
      in.gated_speed_mps = 1.0;
      return in;
    };
    double t = 1000.0;
    std::vector<double> tick_ms, solve_ms, publish_ms;
    for (int i = 0; i < solves + 300; ++i) {
      t += 0.02;
      const TickInput in = make_input(t);
      const bool measure = i >= 300;  // warm up: leave STARTING, ramp, warm start
#ifdef RACER_BENCH_OLD
      const auto a = std::chrono::steady_clock::now();
      planner.tick(in);
      const auto b = std::chrono::steady_clock::now();
      if (measure) tick_ms.push_back(std::chrono::duration<double, std::milli>(b - a).count());
#else
      PublishInput pin;
      pin.now_s = t;
      pin.has_gated = true;
      pin.gated_stamp_s = t;
      const auto a = std::chrono::steady_clock::now();
      planner.solve(in);
      const auto b = std::chrono::steady_clock::now();
      planner.publish(pin);
      const auto c = std::chrono::steady_clock::now();
      if (measure) {
        solve_ms.push_back(std::chrono::duration<double, std::milli>(b - a).count());
        publish_ms.push_back(std::chrono::duration<double, std::milli>(c - b).count());
        tick_ms.push_back(std::chrono::duration<double, std::milli>(c - a).count());
      }
#endif
    }
    print_stats("planner tick (solve + output) ", stats_of(tick_ms), budget_ms);
    if (!solve_ms.empty()) {
      print_stats("planner solve()               ", stats_of(solve_ms), budget_ms);
      const Stats ps = stats_of(publish_ms);
      std::printf("planner publish() path ms: p50 %.4f  p90 %.4f  p99 %.4f  max %.4f\n", ps.p50,
                  ps.p90, ps.p99, ps.max);
    }
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
