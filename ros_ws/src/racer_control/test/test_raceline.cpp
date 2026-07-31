// L1 tests for racer_control::Raceline (claude-docs/12-testing.md).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "racer_control/raceline.hpp"

namespace racer_control {
namespace {

// Populated by CMakeLists.txt via a compile definition pointing at
// test/fixtures/tiny_raceline.csv (see that file's own header comment).
#ifndef RACER_CONTROL_TEST_FIXTURE_DIR
#define RACER_CONTROL_TEST_FIXTURE_DIR "."
#endif

const double kHalfPi = std::atan(1.0) * 2.0;
const double kPi = std::atan(1.0) * 4.0;

std::string fixture_path(const std::string& name) {
  return std::string(RACER_CONTROL_TEST_FIXTURE_DIR) + "/" + name;
}

std::vector<RacelinePoint> four_corner_points() {
  return {
      RacelinePoint{0.0, 0.0, 0.0, 0.0, 0.0, 3.0},
      RacelinePoint{1.0, 1.0, 0.0, 0.0, 0.0, 3.0},
      RacelinePoint{2.0, 1.0, 1.0, kHalfPi, 0.0, 2.0},
      RacelinePoint{3.0, 0.0, 1.0, kPi, 0.0, 2.0},
  };
}

TEST(Raceline, RejectsEmptyPointList) {
  EXPECT_THROW(Raceline(std::vector<RacelinePoint>{}), RacelineLoadError);
}

TEST(Raceline, NearestIndexPicksClosestPoint) {
  Raceline raceline(four_corner_points());
  EXPECT_EQ(raceline.nearest_index(0.1, 0.1), 0u);
  EXPECT_EQ(raceline.nearest_index(0.9, 0.05), 1u);
  EXPECT_EQ(raceline.nearest_index(1.0, 0.9), 2u);
  EXPECT_EQ(raceline.nearest_index(0.1, 1.0), 3u);
}

TEST(Raceline, AdvanceToLookaheadWrapsAroundClosedLoop) {
  Raceline raceline(four_corner_points());
  // From index 3 (0, 1), the only points ahead (with wraparound) are index 0 (0,0) at
  // distance 1.0 and index 1 (1,0) at distance sqrt(2). A lookahead of 1.2 m should land on
  // index 1 after wrapping through index 0.
  std::size_t idx = raceline.advance_to_lookahead(3, 0.0, 1.0, 1.2);
  EXPECT_EQ(idx, 1u);
}

TEST(Raceline, AdvanceToLookaheadReturnsFarthestWhenLoopTooShort) {
  Raceline raceline(four_corner_points());
  std::size_t idx = raceline.advance_to_lookahead(0, 0.0, 0.0, 1000.0);
  // Farthest point from the origin among the four corners is (1, 1) at index 2.
  EXPECT_EQ(idx, 2u);
}

TEST(Raceline, LoadFromCsvParsesFixtureFile) {
  Raceline raceline = Raceline::load_from_csv(fixture_path("tiny_raceline.csv"));
  ASSERT_EQ(raceline.size(), 5u);
  EXPECT_DOUBLE_EQ(raceline.at(0).s_m, 0.0);
  EXPECT_DOUBLE_EQ(raceline.at(2).x_m, 2.0);
  EXPECT_NEAR(raceline.at(3).heading_rad, kHalfPi, 1e-9);
  EXPECT_DOUBLE_EQ(raceline.at(4).target_speed_mps, 2.0);
}

TEST(Raceline, LoadFromCsvThrowsOnMissingFile) {
  EXPECT_THROW(Raceline::load_from_csv(fixture_path("does_not_exist.csv")), RacelineLoadError);
}

TEST(Raceline, LoadFromCsvThrowsOnBadHeader) {
  // Reuses the fixture directory; asserts against a file this test writes itself so the
  // error path is exercised without depending on another fixture file's exact contents.
  std::string path = fixture_path("bad_header_scratch.csv");
  {
    std::ofstream out(path);
    out << "not,the,right,header\n1,2,3\n";
  }
  EXPECT_THROW(Raceline::load_from_csv(path), RacelineLoadError);
  std::remove(path.c_str());
}

TEST(Raceline, LoadFromCsvThrowsOnNonNumericField) {
  std::string path = fixture_path("bad_number_scratch.csv");
  {
    std::ofstream out(path);
    out << "s_m,x_m,y_m,heading_rad,curvature_1pm,target_speed_mps\n";
    out << "0.0,not_a_number,0.0,0.0,0.0,3.0\n";
  }
  EXPECT_THROW(Raceline::load_from_csv(path), RacelineLoadError);
  std::remove(path.c_str());
}

}  // namespace
}  // namespace racer_control
