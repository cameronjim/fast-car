#include <gtest/gtest.h>

#include <cmath>

#include "racer_planner/local_track.hpp"

using namespace racer_planner;

namespace {

LocalTrackData straight_track(int valid = kTrackStations, double hw = 0.5) {
  LocalTrackData t;
  t.frame_id = "base_link";
  t.station_spacing_m = kTrackStationSpacingM;
  t.valid_count = valid;
  for (int k = 0; k < kTrackStations; ++k) {
    t.x_m.push_back(k * kTrackStationSpacingM);
    t.y_m.push_back(0.0);
    t.half_width_left_m.push_back(hw);
    t.half_width_right_m.push_back(hw);
    t.sigma_m.push_back(0.05);
  }
  return t;
}

// Left turn of radius R: the centre of the circle is at (0, R).
LocalTrackData arc_track(double radius) {
  LocalTrackData t = straight_track();
  for (int k = 0; k < kTrackStations; ++k) {
    const double th = k * kTrackStationSpacingM / radius;
    t.x_m[k] = radius * std::sin(th);
    t.y_m[k] = radius * (1.0 - std::cos(th));
  }
  return t;
}

}  // namespace

TEST(LocalTrackValidation, GoodTrackIsOk) {
  EXPECT_EQ(validate_track(straight_track()), TrackValidity::Ok);
}

TEST(LocalTrackValidation, ZeroValidCountIsNoTrack) {
  EXPECT_EQ(validate_track(straight_track(0)), TrackValidity::NoTrack);
}

TEST(LocalTrackValidation, EveryArrayMustHaveSixteenEntries) {
  for (int which = 0; which < 5; ++which) {
    LocalTrackData t = straight_track();
    std::vector<double>* arrays[5] = {&t.x_m, &t.y_m, &t.half_width_left_m, &t.half_width_right_m,
                                      &t.sigma_m};
    arrays[which]->pop_back();
    EXPECT_EQ(validate_track(t), TrackValidity::WrongLength) << which;
    arrays[which]->push_back(0.0);
    arrays[which]->push_back(0.0);
    EXPECT_EQ(validate_track(t), TrackValidity::WrongLength) << which;
  }
}

TEST(LocalTrackValidation, ValidCountAboveSixteenIsWrongLength) {
  EXPECT_EQ(validate_track(straight_track(17)), TrackValidity::WrongLength);
}

TEST(LocalTrackValidation, SpacingMustBeTwentyCentimetres) {
  LocalTrackData t = straight_track();
  t.station_spacing_m = 0.2 + 2e-6;
  EXPECT_EQ(validate_track(t), TrackValidity::WrongSpacing);
  t.station_spacing_m = 0.2 + 5e-7;  // inside 1e-6
  EXPECT_EQ(validate_track(t), TrackValidity::Ok);
  t.station_spacing_m = std::nan("");
  EXPECT_EQ(validate_track(t), TrackValidity::WrongSpacing);
}

TEST(LocalTrackValidation, FrameMustBeBaseLink) {
  LocalTrackData t = straight_track();
  t.frame_id = "laser";
  EXPECT_EQ(validate_track(t), TrackValidity::WrongFrame);
}

TEST(LocalTrackValidation, NonFiniteValueAmongValidStationsIsRefused) {
  for (int which = 0; which < 5; ++which) {
    LocalTrackData t = straight_track(10);
    std::vector<double>* arrays[5] = {&t.x_m, &t.y_m, &t.half_width_left_m, &t.half_width_right_m,
                                      &t.sigma_m};
    (*arrays[which])[9] = std::nan("");
    EXPECT_EQ(validate_track(t), TrackValidity::NonFinite) << which;
    (*arrays[which])[9] = 0.0;
    (*arrays[which])[3] = INFINITY;
    EXPECT_EQ(validate_track(t), TrackValidity::NonFinite) << which;
  }
}

TEST(LocalTrackValidation, NonFiniteBeyondValidCountIsFine) {
  LocalTrackData t = straight_track(10);
  t.x_m[12] = std::nan("");
  t.sigma_m[15] = INFINITY;
  EXPECT_EQ(validate_track(t), TrackValidity::Ok);
}

TEST(TrackProjection, StraightLine) {
  TrackProjector p;
  ASSERT_TRUE(p.build(straight_track(), Pose2{}));
  EXPECT_NEAR(p.s_valid_end(), 15 * 0.2, 1e-12);
  int hint = p.locate(0.5, 0.1);
  Projection a = p.project(0.5, 0.1, &hint);
  EXPECT_NEAR(a.s, 0.5, 1e-12);
  EXPECT_NEAR(a.d, 0.1, 1e-12);  // left of travel is positive
  hint = p.locate(1.3, -0.25);
  Projection b = p.project(1.3, -0.25, &hint);
  EXPECT_NEAR(b.s, 1.3, 1e-12);
  EXPECT_NEAR(b.d, -0.25, 1e-12);
  EXPECT_NEAR(b.half_left, 0.5, 1e-12);
  EXPECT_NEAR(b.half_right, 0.5, 1e-12);
}

TEST(TrackProjection, ArcLengthAndOffsetOnACircle) {
  const double R = 1.0;
  TrackProjector p;
  ASSERT_TRUE(p.build(arc_track(R), Pose2{}));
  // A point on the centreline at 40 degrees, and one pushed 0.1 m outward (to the right of a left
  // turn).
  const double th = 40.0 * M_PI / 180.0;
  int hint = p.locate(R * std::sin(th), R * (1.0 - std::cos(th)));
  Projection on = p.project(R * std::sin(th), R * (1.0 - std::cos(th)), &hint);
  EXPECT_NEAR(on.s, R * th, 2e-3);  // polyline chords are a hair shorter than the arc
  EXPECT_NEAR(on.d, 0.0, 6e-3);
  const double rr = R + 0.1;
  hint = p.locate(rr * std::sin(th), R - rr * std::cos(th));
  Projection out = p.project(rr * std::sin(th), R - rr * std::cos(th), &hint);
  EXPECT_NEAR(out.s, R * th, 4e-3);
  EXPECT_NEAR(out.d, -0.1, 6e-3);
  const double ri = R - 0.1;
  hint = p.locate(ri * std::sin(th), R - ri * std::cos(th));
  Projection in = p.project(ri * std::sin(th), R - ri * std::cos(th), &hint);
  EXPECT_NEAR(in.d, 0.1, 6e-3);
}

TEST(TrackProjection, HalfWidthsInterpolateAlongTheTrack) {
  LocalTrackData t = straight_track();
  for (int k = 0; k < kTrackStations; ++k) {
    t.half_width_left_m[k] = 0.4 + 0.01 * k;
    t.half_width_right_m[k] = 0.6 - 0.01 * k;
  }
  TrackProjector p;
  ASSERT_TRUE(p.build(t, Pose2{}));
  int hint = 0;
  const Projection pr = p.project(0.5, 0.0, &hint);  // between station 2 (0.4) and 3 (0.6)
  EXPECT_NEAR(pr.half_left, 0.4 + 0.01 * 2.5, 1e-12);
  EXPECT_NEAR(pr.half_right, 0.6 - 0.01 * 2.5, 1e-12);
}

TEST(TrackProjection, ProgressIsCappedAtTheLastValidStation) {
  TrackProjector p;
  ASSERT_TRUE(p.build(straight_track(8), Pose2{}));  // visible to x = 1.4
  int hint = p.locate(3.0, 0.05);
  const Projection pr = p.project(3.0, 0.05, &hint);
  EXPECT_NEAR(pr.s, 1.4, 1e-12);
  EXPECT_NEAR(pr.d, 0.05, 1e-12);
  EXPECT_NEAR(p.s_valid_end(), 1.4, 1e-12);
}

TEST(TrackProjection, BehindStationZeroGivesNegativeArcLength) {
  TrackProjector p;
  ASSERT_TRUE(p.build(straight_track(), Pose2{}));
  int hint = 0;
  EXPECT_NEAR(p.project(-0.3, 0.0, &hint).s, -0.3, 1e-12);
}

TEST(TrackProjection, FrameChangeMovesThePolyline) {
  TrackProjector p;
  // The track's frame origin is at (1, 0) in the target frame, rotated 90 degrees left, so the
  // straight track now runs along +y from (1, 0).
  ASSERT_TRUE(p.build(straight_track(), Pose2{1.0, 0.0, M_PI / 2.0}));
  int hint = 0;
  const Projection pr = p.project(1.2, 0.6, &hint);
  EXPECT_NEAR(pr.s, 0.6, 1e-9);
  EXPECT_NEAR(pr.d, -0.2, 1e-9);  // x larger than the line is to the right of +y travel
}

TEST(TrackProjection, HintedSearchAgreesWithBruteForceAlongAPath) {
  TrackProjector p;
  ASSERT_TRUE(p.build(arc_track(0.9), Pose2{}));
  int hint = p.locate(0.0, 0.0);
  for (int i = 0; i < 60; ++i) {
    const double th = 0.02 * i;
    const double x = 0.9 * std::sin(th) + 0.05;
    const double y = 0.9 * (1.0 - std::cos(th));
    const Projection h = p.project(x, y, &hint);
    int fresh = p.locate(x, y);
    const Projection f = p.project(x, y, &fresh);
    EXPECT_NEAR(h.s, f.s, 1e-9) << i;
    EXPECT_NEAR(h.d, f.d, 1e-9) << i;
  }
}

TEST(TrackProjection, BuildFailsOnAnInvalidOrShortTrack) {
  TrackProjector p;
  EXPECT_FALSE(p.build(straight_track(1), Pose2{}));
  EXPECT_FALSE(p.valid());
  LocalTrackData bad = straight_track();
  bad.frame_id = "map";
  EXPECT_FALSE(p.build(bad, Pose2{}));
  EXPECT_TRUE(p.build(straight_track(2), Pose2{}));
  EXPECT_TRUE(p.valid());
}

TEST(TrackProjection, NegativeHalfWidthIsTreatedAsZero) {
  LocalTrackData t = straight_track();
  t.half_width_left_m[3] = -1.0;
  t.half_width_left_m[4] = -1.0;
  TrackProjector p;
  ASSERT_TRUE(p.build(t, Pose2{}));
  int hint = 0;
  EXPECT_EQ(p.project(0.7, 0.0, &hint).half_left, 0.0);
}
