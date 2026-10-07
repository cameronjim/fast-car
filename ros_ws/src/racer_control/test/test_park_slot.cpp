// park_slot.hpp: slot detection and lane measurement on synthetic 360 degree scans with the
// car's LiDAR geometry (yaw pi, mount 0.285 m ahead of the rear axle; park_test_helpers.hpp).
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "park_test_helpers.hpp"
#include "racer_control/park_slot.hpp"

namespace racer_control {
namespace {

using namespace park_test;

// Edges are reported on the bin grid (0.05 m): the obstacle ends inside the neighbouring bin.
constexpr double kEdgeTol = 0.05 + 1e-6;

TEST(ParkSlot, FindsAPocketOnTheRight) {
  // Row face 0.5 m right of the centreline, pocket from x 0.4 to 1.9 (1.5 m), 0.65 m deep, a wall
  // 0.9 m to the left.
  const auto world = pocket_world(ParkSide::kRight, 0.5, 0.4, 1.9, 0.65, 0.9);
  SlotDetector detector(car_detector(ParkSide::kRight));
  const SlotDetection d = detector.detect(synthetic_scan(Pose2{}, world));
  ASSERT_TRUE(d.found) << slot_reject_name(d.reject);
  EXPECT_NEAR(d.near_x_m, 0.4, kEdgeTol);
  EXPECT_NEAR(d.far_x_m, 1.9, kEdgeTol);
  // The slot never reads longer than it is.
  EXPECT_GE(d.near_x_m, 0.4 - 1e-9);
  EXPECT_LE(d.far_x_m, 1.9 + 1e-9);
  EXPECT_NEAR(d.row_lateral_m, 0.5, 0.01);
  EXPECT_NEAR(d.depth_m, 0.65, 0.02);
  EXPECT_NEAR(d.back_lateral_m, 1.15, 0.02);
  EXPECT_TRUE(d.back_seen);
  EXPECT_NEAR(d.row_angle_rad, 0.0, 0.01);
  ASSERT_TRUE(d.far_side_lateral_m.has_value());
  EXPECT_NEAR(*d.far_side_lateral_m, 0.9, 0.01);
}

TEST(ParkSlot, FindsAPocketOnTheLeft) {
  const auto world = pocket_world(ParkSide::kLeft, 0.6, -0.5, 0.7, 0.8);
  SlotDetector detector(car_detector(ParkSide::kLeft));
  const SlotDetection d = detector.detect(synthetic_scan(Pose2{}, world));
  ASSERT_TRUE(d.found) << slot_reject_name(d.reject);
  EXPECT_NEAR(d.near_x_m, -0.5, kEdgeTol);
  EXPECT_NEAR(d.far_x_m, 0.7, kEdgeTol);
  EXPECT_NEAR(d.row_lateral_m, 0.6, 0.01);
  EXPECT_NEAR(d.depth_m, 0.8, 0.02);
  EXPECT_FALSE(d.far_side_lateral_m.has_value());
  // The right-side detector does not see it.
  SlotDetector right(car_detector(ParkSide::kRight));
  EXPECT_FALSE(right.detect(synthetic_scan(Pose2{}, world)).found);
}

// The mount yaw and position are applied: the same world seen with the scan built for a forward-
// facing LiDAR at the rear axle (what a detector ignoring the mount would assume) puts the pocket
// on the other side and the edges 0.285 m off.
TEST(ParkSlot, MountYawAndOffsetAreApplied) {
  const auto world = pocket_world(ParkSide::kRight, 0.5, 0.4, 1.9, 0.65);
  SlotDetectorConfig wrong = car_detector(ParkSide::kRight);
  wrong.laser_yaw_offset_rad = 0.0;
  SlotDetector detector(wrong);
  EXPECT_FALSE(detector.detect(synthetic_scan(Pose2{}, world)).found);
  SlotDetectorConfig no_mount = car_detector(ParkSide::kRight);
  no_mount.lidar_mount_x_m = 0.0;
  SlotDetector shifted(no_mount);
  const SlotDetection d = shifted.detect(synthetic_scan(Pose2{}, world));
  ASSERT_TRUE(d.found);
  EXPECT_NEAR(d.near_x_m, 0.4 - kLidarX, kEdgeTol);
}

TEST(ParkSlot, NoPocket) {
  const std::vector<Segment2> world = {{{-5.0, -0.5}, {5.0, -0.5}}};
  SlotDetector detector(car_detector(ParkSide::kRight));
  const SlotDetection d = detector.detect(synthetic_scan(Pose2{}, world));
  EXPECT_FALSE(d.found);
  EXPECT_EQ(d.reject, SlotReject::kNoPocket);
  // Nothing at all on the parking side.
  const std::vector<Segment2> left_only = {{{-3.0, 0.8}, {3.0, 0.8}}};
  EXPECT_EQ(detector.detect(synthetic_scan(Pose2{}, left_only)).reject, SlotReject::kNoReturns);
}

TEST(ParkSlot, PocketTooShort) {
  // 0.6 m: shorter than chassis.length_m + 0.35 = 0.93 m.
  const auto world = pocket_world(ParkSide::kRight, 0.5, 0.5, 1.1, 0.65);
  SlotDetector detector(car_detector(ParkSide::kRight));
  const SlotDetection d = detector.detect(synthetic_scan(Pose2{}, world));
  EXPECT_FALSE(d.found);
  EXPECT_EQ(d.reject, SlotReject::kTooShort);
  EXPECT_NEAR(d.candidate_length_m, 0.6, 2.0 * kEdgeTol);
}

TEST(ParkSlot, PocketTooShallow) {
  // 0.3 m deep: shallower than chassis.width_m + 2 * 0.1 = 0.51 m.
  const auto world = pocket_world(ParkSide::kRight, 0.5, 0.3, 1.8, 0.3);
  SlotDetector detector(car_detector(ParkSide::kRight));
  const SlotDetection d = detector.detect(synthetic_scan(Pose2{}, world));
  EXPECT_FALSE(d.found);
  EXPECT_EQ(d.reject, SlotReject::kTooShallow);
  EXPECT_NEAR(d.candidate_depth_m, 0.3, 0.02);
}

TEST(ParkSlot, UnboundedPocketIsNotASlotYet) {
  // The pocket runs past the end of the window ahead (2.5 m): its far edge is not seen.
  const auto world = pocket_world(ParkSide::kRight, 0.5, 1.0, 6.0, 0.65);
  SlotDetector detector(car_detector(ParkSide::kRight));
  EXPECT_FALSE(detector.detect(synthetic_scan(Pose2{}, world)).found);
}

TEST(ParkSlot, RowNotParallelIsRefused) {
  // The whole world rotated by 0.15 rad about the car: the row is 8.6 degrees off.
  auto world = pocket_world(ParkSide::kRight, 0.5, 0.4, 1.9, 0.65);
  const Pose2 rot{0.0, 0.0, 0.15};
  for (Segment2& s : world) {
    s.a = to_world(rot, s.a);
    s.b = to_world(rot, s.b);
  }
  SlotDetector detector(car_detector(ParkSide::kRight));
  const SlotDetection d = detector.detect(synthetic_scan(Pose2{}, world));
  EXPECT_FALSE(d.found);
  EXPECT_EQ(d.reject, SlotReject::kRowAngle);
  EXPECT_NEAR(d.row_angle_rad, 0.15, 0.03);
}

TEST(ParkSlot, InvalidReturnsAreIgnored) {
  const auto world = pocket_world(ParkSide::kRight, 0.5, 0.4, 1.9, 0.65);
  ScanInput scan = synthetic_scan(Pose2{}, world);
  for (std::size_t i = 0; i < scan.ranges.size(); i += 7) {
    scan.ranges[i] = (i % 2 == 0) ? 0.0f : std::nanf("");
  }
  SlotDetector detector(car_detector(ParkSide::kRight));
  const SlotDetection d = detector.detect(scan);
  ASSERT_TRUE(d.found);
  EXPECT_NEAR(d.near_x_m, 0.4, kEdgeTol);
  EXPECT_NEAR(d.far_x_m, 1.9, kEdgeTol);
  ScanInput empty;
  EXPECT_EQ(detector.detect(empty).reject, SlotReject::kBadInput);
}

TEST(ParkSlot, MeasureLane) {
  const std::vector<Segment2> world = {{{-5.0, 0.8}, {5.0, 0.8}}, {{-5.0, -1.1}, {5.0, -1.1}}};
  const LaneMeasurement lane = measure_lane(synthetic_scan(Pose2{}, world), kLidarYaw, kLidarX,
                                            kLidarY, -0.4, 0.8, kWidth / 2.0, 3.0);
  ASSERT_TRUE(lane.left_m.has_value());
  ASSERT_TRUE(lane.right_m.has_value());
  EXPECT_NEAR(*lane.left_m, 0.8, 0.005);
  EXPECT_NEAR(*lane.right_m, 1.1, 0.005);
  const LaneMeasurement open = measure_lane(synthetic_scan(Pose2{}, {world[0]}), kLidarYaw, kLidarX,
                                            kLidarY, -0.4, 0.8, kWidth / 2.0, 3.0);
  EXPECT_TRUE(open.left_m.has_value());
  EXPECT_FALSE(open.right_m.has_value());
}

}  // namespace
}  // namespace racer_control
