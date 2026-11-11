#include <gtest/gtest.h>

#include <cmath>

#include "racer_planner/rng.hpp"

using namespace racer_planner;

TEST(GaussianRng, SameSeedSameSequenceDifferentSeedDifferent) {
  GaussianRng a(5), b(5), c(6);
  bool differs = false;
  for (int i = 0; i < 1000; ++i) {
    const double x = a.normal();
    EXPECT_EQ(x, b.normal());
    if (x != c.normal()) differs = true;
  }
  EXPECT_TRUE(differs);
}

TEST(GaussianRng, MomentsOfAStandardNormal) {
  GaussianRng r(1);
  const int n = 2000000;
  double s1 = 0, s2 = 0, s3 = 0, s4 = 0;
  int beyond3 = 0;
  for (int i = 0; i < n; ++i) {
    const double x = r.normal();
    s1 += x;
    s2 += x * x;
    s3 += x * x * x;
    s4 += x * x * x * x;
    if (std::fabs(x) > 3.0) ++beyond3;
  }
  EXPECT_NEAR(s1 / n, 0.0, 0.005);
  EXPECT_NEAR(s2 / n, 1.0, 0.01);
  EXPECT_NEAR(s3 / n, 0.0, 0.03);
  EXPECT_NEAR(s4 / n, 3.0, 0.06);
  // P(|x| > 3) = 0.0026998
  EXPECT_NEAR(static_cast<double>(beyond3) / n, 0.0026998, 0.0004);
}

TEST(GaussianRng, UniformIsInTheOpenUnitInterval) {
  GaussianRng r(3);
  double lo = 1, hi = 0;
  for (int i = 0; i < 100000; ++i) {
    const double u = r.uniform();
    lo = std::min(lo, u);
    hi = std::max(hi, u);
  }
  EXPECT_GT(lo, 0.0);
  EXPECT_LT(hi, 1.0);
  EXPECT_LT(lo, 1e-3);
  EXPECT_GT(hi, 1.0 - 1e-3);
}
