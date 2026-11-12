// mppi_bench: solve time of the MPPI solver on a synthetic corridor, with the dynamics bundle the
// car would load. Prints p50 / p90 / p99 / max in ms and the sample count; exit code 1 when p99
// is above --budget-ms (default 12, the Jetson budget). --background-threads N runs N busy threads
// for the length of the run, to measure the solve with the rest of the stack competing for cores.
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
#include "racer_planner/local_track.hpp"
#include "racer_planner/mppi.hpp"

using namespace racer_planner;

namespace {
void usage() {
  std::fprintf(stderr,
               "usage: mppi_bench --bundle DIR [--samples 512] [--solves 2000] [--horizon 30]\n"
               "                  [--speed 0.6] [--budget-ms 12] [--background-threads 0]\n");
}
}  // namespace

int main(int argc, char** argv) {
  std::string bundle;
  int samples = 512, solves = 2000, horizon = 30, background_threads = 0;
  double speed = 0.6, budget_ms = 12.0;
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

  // Synthetic corridor, 1.0 m wide, walls every 2 cm, the car in the middle looking down it.
  std::vector<PointXY> pts;
  for (double x = -1.0; x <= 4.0; x += 0.02) {
    pts.push_back({x, 0.5});
    pts.push_back({x, -0.5});
  }
  DistanceField field;
  field.build(pts.data(), pts.size());
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

  MppiParams params;
  params.horizon_steps = horizon;
  params.samples = samples;
  params.seed = 7;
  MppiSolver solver(params, model, *geometry, samples);
  SolveInput in;
  in.x0.v = 0.5 * speed;
  in.v_cap_mps = speed;
  in.track = &projector;
  in.field = &field;
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
      stop.store(true);
      for (auto& t : busy) t.join();
      return 3;
    }
    if (i >= 50) ms.push_back(dt);  // the first 50 are warm up
  }
  stop.store(true);
  for (auto& t : busy) t.join();
  std::sort(ms.begin(), ms.end());
  auto pct = [&](double q) {
    return ms[std::min(ms.size() - 1, static_cast<std::size_t>(q * ms.size()))];
  };
  const double p99 = pct(0.99);
  std::printf("mppi_bench: model %s, samples %d, horizon %d, solves %zu, background threads %d\n",
              loaded.bundle->model_id.c_str(), samples, horizon, ms.size(), background_threads);
  std::printf("solve ms: p50 %.3f  p90 %.3f  p99 %.3f  max %.3f  (budget p99 %.1f)\n", pct(0.5),
              pct(0.9), p99, ms.back(), budget_ms);
  return p99 > budget_ms ? 1 : 0;
}
