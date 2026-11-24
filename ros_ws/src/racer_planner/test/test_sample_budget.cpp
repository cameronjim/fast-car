// SampleBudget, L2 and L4. L2: for any sequence of solve times (seeded generators, bursts,
// outliers, sustained overload, recovery) the count stays within [min, max], never grows on a tick
// whose solve was over the halve limit, and never falls on a tick whose solve was not over it. L4:
// the solve times of the D v0 cw run of session 2026-11-22 (a car run on which the budget sat at
// about 200 of 512 samples) fed through the budget.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "racer_planner/planner_logic.hpp"

using namespace racer_planner;

namespace {

SampleBudgetParams car_budget() {
  SampleBudgetParams p;
  p.max_samples = 512;
  return p;
}

TEST(SampleBudgetProperties, CountStaysInBoundsAndOnlyMovesTheAllowedWay) {
  for (std::uint64_t seed = 1; seed <= 200; ++seed) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    SampleBudgetParams p = car_budget();
    p.window = 1 + static_cast<int>(rng() % 40);
    p.halve_count = 1 + static_cast<int>(rng() % 6);
    p.restore_count = 1 + static_cast<int>(rng() % 45);
    SampleBudget b(p);
    // A regime that changes every few hundred ticks: p(over), p(under), rest mid band.
    double p_over = 0.0, p_under = 0.9;
    int prev = b.current();
    ASSERT_EQ(prev, 512);
    for (int i = 0; i < 6000; ++i) {
      if (i % 250 == 0) {
        p_over = u(rng) < 0.5 ? 0.005 : u(rng) * 0.6;
        p_under = u(rng);
      }
      const double r = u(rng);
      double t;
      if (r < p_over) {
        t = 0.0151 + u(rng) * 0.05;
      } else if (r < p_over + (1.0 - p_over) * p_under) {
        t = u(rng) * 0.0059;
      } else {
        t = 0.006 + u(rng) * 0.009;
      }
      if (i % 997 == 0) t = std::numeric_limits<double>::quiet_NaN();
      const int now = b.update(t, nullptr);
      ASSERT_GE(now, 128) << "seed " << seed << " tick " << i;
      ASSERT_LE(now, 512) << "seed " << seed << " tick " << i;
      if (t > 0.015) {
        ASSERT_LE(now, prev) << "grew on a solve over budget, seed " << seed << " tick " << i;
      } else {
        ASSERT_GE(now, prev) << "fell on a solve within budget, seed " << seed << " tick " << i;
      }
      prev = now;
    }
  }
}

TEST(SampleBudgetProperties, AnIsolatedLateSolveNeverChangesTheCount) {
  for (std::uint64_t seed = 1; seed <= 50; ++seed) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    SampleBudget b(car_budget());
    int since_late = 1000;
    for (int i = 0; i < 5000; ++i) {
      // At most 2 late solves in any 25: late only when 13 or more ticks since the last one.
      const bool late = since_late >= 13 && u(rng) < 0.2;
      since_late = late ? 0 : since_late + 1;
      const double t = late ? 0.016 + u(rng) * 0.04 : u(rng) * 0.0149;
      ASSERT_EQ(b.update(t, nullptr), 512) << "seed " << seed << " tick " << i;
    }
  }
}

std::vector<double> load_solve_times_s() {
  const std::string path = std::string(RACER_PLANNER_FIXTURE_DIR) + "/dv0_cw_solve_times_ms.csv";
  std::ifstream in(path);
  std::vector<double> out;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    out.push_back(std::stod(line) * 1e-3);
  }
  return out;
}

TEST(SampleBudgetReplay, CarSolveTimesOfTheDV0RunKeepMostTicksAtFullSamples) {
  const std::vector<double> times = load_solve_times_s();
  ASSERT_EQ(times.size(), 10052u);  // 201 s at 50 Hz
  // Recorded: 54 solves over 15 ms, none of them sustained.
  ASSERT_EQ(std::count_if(times.begin(), times.end(), [](double t) { return t > 0.015; }), 54);
  SampleBudget b(car_budget());
  int at_max = 0, lowest = 512;
  int prev = b.current();
  // The sample count a tick USES is the count the previous solve left, as in the node.
  for (const double t : times) {
    if (prev == 512) ++at_max;
    prev = b.update(t, nullptr);
    lowest = std::min(lowest, prev);
  }
  const double fraction = static_cast<double>(at_max) / static_cast<double>(times.size());
  std::printf("replay: %d of %zu ticks at 512 (%.2f percent), lowest count %d\n", at_max,
              times.size(), fraction * 100.0, lowest);
  EXPECT_GT(fraction, 0.90);
}

}  // namespace
