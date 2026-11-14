// make_lap_profile (one case per refusal reason) and the profile lookups.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "racer_planner/lap_profile.hpp"

using namespace racer_planner;

namespace {

struct Msg {
  std::vector<float> s, v, q;
  double stamp = 10.0;
  double now = 10.05;

  explicit Msg(std::size_t n = 61, float s0 = 1.0f) {
    for (std::size_t i = 0; i < n; ++i) {
      s.push_back(s0 + 0.1f * static_cast<float>(i));
      v.push_back(1.0f);
      q.push_back(5.0f - 0.1f * static_cast<float>(i));
    }
  }

  LapProfileStatus make(LapProfile* out) const {
    return make_lap_profile(s.data(), v.data(), q.data(), s.size(), v.size(), q.size(), stamp, now,
                            out);
  }
};

}  // namespace

TEST(LapProfileMake, AGoodMessageIsCopiedAndActive) {
  Msg m;
  LapProfile p;
  ASSERT_EQ(m.make(&p), LapProfileStatus::kOk);
  EXPECT_TRUE(p.active);
  EXPECT_EQ(p.n, 61u);
  EXPECT_DOUBLE_EQ(p.stamp_s, 10.0);
  EXPECT_NEAR(p.s0_m, 1.0, 1e-6);
  EXPECT_NEAR(p.spacing_m, 0.1, 1e-6);
  EXPECT_FLOAT_EQ(static_cast<float>(p.v_bound[7]), m.v[7]);
  EXPECT_FLOAT_EQ(static_cast<float>(p.q[60]), m.q[60]);
}

TEST(LapProfileMake, EmptyArraysAreNoProfileNotAnError) {
  LapProfile p;
  p.active = true;
  EXPECT_EQ(make_lap_profile(nullptr, nullptr, nullptr, 0, 0, 0, 1.0, 1.0, &p),
            LapProfileStatus::kNone);
  EXPECT_FALSE(p.active);
}

TEST(LapProfileMake, ArraysOfDifferentLengthsAreRefused) {
  Msg m;
  LapProfile p;
  EXPECT_EQ(make_lap_profile(m.s.data(), m.v.data(), m.q.data(), 61, 60, 61, 10.0, 10.0, &p),
            LapProfileStatus::kBad);
  EXPECT_EQ(make_lap_profile(m.s.data(), m.v.data(), m.q.data(), 61, 61, 60, 10.0, 10.0, &p),
            LapProfileStatus::kBad);
  EXPECT_EQ(make_lap_profile(m.s.data(), m.v.data(), m.q.data(), 0, 61, 61, 10.0, 10.0, &p),
            LapProfileStatus::kBad);
  // only the station array empty, the other two not: refused, not "none"
  EXPECT_EQ(make_lap_profile(nullptr, m.v.data(), m.q.data(), 0, 61, 61, 10.0, 10.0, &p),
            LapProfileStatus::kBad);
  EXPECT_FALSE(p.active);
}

TEST(LapProfileMake, OneStationAndTooManyAreRefusedTheEdgesAreNot) {
  LapProfile p;
  {
    Msg m(1);
    EXPECT_EQ(m.make(&p), LapProfileStatus::kBad);
  }
  {
    Msg m(2);
    EXPECT_EQ(m.make(&p), LapProfileStatus::kOk);
  }
  {
    Msg m(128);
    EXPECT_EQ(m.make(&p), LapProfileStatus::kOk);
    EXPECT_EQ(p.n, 128u);
  }
  {
    Msg m(129);
    p.active = true;
    EXPECT_EQ(m.make(&p), LapProfileStatus::kBad);
    EXPECT_FALSE(p.active);
  }
}

TEST(LapProfileMake, NonFiniteValuesAreRefusedInEveryArray) {
  const float nan = std::nanf("");
  const float inf = std::numeric_limits<float>::infinity();
  LapProfile p;
  for (float bad : {nan, inf, -inf}) {
    {
      Msg m;
      m.s[30] = bad;
      EXPECT_EQ(m.make(&p), LapProfileStatus::kBad) << "s " << bad;
    }
    {
      Msg m;
      m.v[30] = bad;
      EXPECT_EQ(m.make(&p), LapProfileStatus::kBad) << "v " << bad;
    }
    {
      Msg m;
      m.q[30] = bad;
      EXPECT_EQ(m.make(&p), LapProfileStatus::kBad) << "q " << bad;
    }
  }
  Msg m;
  m.stamp = std::nan("");
  EXPECT_EQ(m.make(&p), LapProfileStatus::kBad);
  Msg m2;
  m2.now = std::numeric_limits<double>::infinity();
  EXPECT_EQ(m2.make(&p), LapProfileStatus::kBad);
}

TEST(LapProfileMake, StationSpacingMustBeTenCentimetresWithinAMillimetre) {
  LapProfile p;
  {
    Msg m;
    for (std::size_t i = 0; i < m.s.size(); ++i) m.s[i] = 1.0f + 0.2f * static_cast<float>(i);
    EXPECT_EQ(m.make(&p), LapProfileStatus::kBad) << "0.2 m spacing";
  }
  {
    Msg m;
    m.s[40] = m.s[39];  // not increasing
    EXPECT_EQ(m.make(&p), LapProfileStatus::kBad);
  }
  {
    Msg m;
    m.s[40] -= 0.05f;  // a dip
    EXPECT_EQ(m.make(&p), LapProfileStatus::kBad);
  }
  {
    Msg m;  // one step 0.5 mm off: accepted
    for (std::size_t i = 21; i < m.s.size(); ++i) m.s[i] += 0.0005f;
    EXPECT_EQ(m.make(&p), LapProfileStatus::kOk);
  }
  {
    Msg m;  // one step 1.5 mm off: refused
    for (std::size_t i = 21; i < m.s.size(); ++i) m.s[i] += 0.0015f;
    EXPECT_EQ(m.make(&p), LapProfileStatus::kBad);
  }
}

TEST(LapProfileMake, NegativeSpeedIsRefusedZeroIsNot) {
  LapProfile p;
  Msg m;
  m.v[5] = 0.0f;
  EXPECT_EQ(m.make(&p), LapProfileStatus::kOk) << "0 is a caution zone";
  m.v[5] = -0.01f;
  EXPECT_EQ(m.make(&p), LapProfileStatus::kBad);
}

TEST(LapProfileMake, CostToGoMustBeNonIncreasingWithinAMillisecond) {
  LapProfile p;
  {
    Msg m;
    m.q[20] = m.q[19] + 0.0005f;  // a 0.5 ms rise: noise
    EXPECT_EQ(m.make(&p), LapProfileStatus::kOk);
  }
  {
    Msg m;
    m.q[20] = m.q[19] + 0.002f;  // a 2 ms rise: refused
    EXPECT_EQ(m.make(&p), LapProfileStatus::kBad);
  }
  {
    Msg m;  // negative values past the lap end are fine (Q(L + x) = Q(x) - T_best)
    for (std::size_t i = 0; i < m.q.size(); ++i) m.q[i] = 1.0f - 0.1f * static_cast<float>(i);
    EXPECT_EQ(m.make(&p), LapProfileStatus::kOk);
  }
}

TEST(LapProfileMake, AgeMustBeInZeroToHalfASecondInclusive) {
  LapProfile p;
  Msg m;
  m.stamp = 10.0;
  m.now = 10.0;
  EXPECT_EQ(m.make(&p), LapProfileStatus::kOk) << "age 0";
  m.now = 10.5;
  EXPECT_EQ(m.make(&p), LapProfileStatus::kOk) << "age exactly 0.5 s";
  m.now = std::nextafter(10.5, 11.0);
  EXPECT_EQ(m.make(&p), LapProfileStatus::kBad) << "just over 0.5 s";
  m.now = std::nextafter(10.0, 9.0);
  EXPECT_EQ(m.make(&p), LapProfileStatus::kBad) << "from the future";
  m.now = 20.0;
  EXPECT_EQ(m.make(&p), LapProfileStatus::kBad) << "stale";
  EXPECT_FALSE(p.active);
}

TEST(LapProfileLookup, AtBetweenAndBeyondStations) {
  Msg m;  // s 1.0 .. 7.0, v 1, q 5 .. -1
  m.v[10] = 2.0f;
  LapProfile p;
  ASSERT_EQ(m.make(&p), LapProfileStatus::kOk);
  // at a station
  EXPECT_NEAR(p.v_bound_at(2.0), 2.0, 1e-5);
  EXPECT_NEAR(p.q_at(2.0), 4.0, 1e-5);
  // halfway between 1.9 (v 1, q 4.1) and 2.0 (v 2, q 4.0)
  EXPECT_NEAR(p.v_bound_at(1.95), 1.5, 1e-5);
  EXPECT_NEAR(p.q_at(1.95), 4.05, 1e-5);
  // before the first station: the first values
  EXPECT_NEAR(p.v_bound_at(0.0), 1.0, 1e-9);
  EXPECT_NEAR(p.q_at(0.0), 5.0, 1e-9);
  // beyond the last: v holds, q continues with its last slope (-1 s per m)
  EXPECT_NEAR(p.v_bound_at(9.0), 1.0, 1e-9);
  EXPECT_NEAR(p.q_at(7.5), -1.5, 1e-5);
  EXPECT_NEAR(p.q_at(9.0), -3.0, 1e-5);
  // NaN station: the first entry, never NaN
  EXPECT_TRUE(std::isfinite(p.v_bound_at(std::nan(""))));
  EXPECT_TRUE(std::isfinite(p.q_at(std::nan(""))));
}

TEST(LapProfileLookup, CostToGoStaysMonotoneAcrossTheLapEnd) {
  // Q(L + x) = Q(x) - T_best: a profile that starts 2 m before the lap end and runs through it.
  const double t_best = 9.0;
  const double lap_m = 7.0;
  std::vector<float> s, v, q;
  for (int i = 0; i < 61; ++i) {
    const double st = 5.0 + 0.1 * i;  // 5.0 .. 11.0, the lap closes at 7.0
    s.push_back(static_cast<float>(st));
    v.push_back(1.0f);
    const double x = st > lap_m ? st - lap_m : st;
    const double qx = t_best * (1.0 - x / lap_m);  // time to finish from station x on a lap
    q.push_back(static_cast<float>(st > lap_m ? qx - t_best : qx));
  }
  LapProfile p;
  ASSERT_EQ(make_lap_profile(s.data(), v.data(), q.data(), 61, 61, 61, 1.0, 1.0, &p),
            LapProfileStatus::kOk);
  double prev = p.q_at(4.0);
  for (double st = 4.0; st <= 13.0; st += 0.013) {
    const double now = p.q_at(st);
    EXPECT_LE(now, prev + 1e-9) << st;
    prev = now;
  }
  EXPECT_LT(p.q_at(11.0), 0.0) << "past the lap end the cost-to-go is negative";
}

TEST(LapProfileLookup, AnInactiveProfileConstrainsNothing) {
  LapProfile p;
  EXPECT_FALSE(p.active);
  EXPECT_TRUE(std::isinf(p.v_bound_at(1.0)));
  EXPECT_EQ(p.q_at(1.0), 0.0);
}
