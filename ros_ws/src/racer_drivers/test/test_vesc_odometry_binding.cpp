// L1 tests that pin vesc_odometry against the REAL config/vehicle_params.yaml, through the
// same generated binding vesc_odometry_node reads (CLAUDE.md invariant 2). No car number is
// typed in here: the drivetrain comes from VEHICLE_PARAMS, and the VESC's ERPM cap is read out
// of the newest committed VESC Tool motor export in config/vesc/ at test time.
//
// The cross-check that matters: actuation.throttle_full_scale_mps (2.91 m/s, schema 0.9.3) was
// DERIVED by hand on 2026-10-06 from l_max_erpm 6000, pole_pairs, gear_ratio and
// tires.nominal_radius_m. This node's conversion must reproduce it from the same inputs. If
// someone changes gear_ratio or the tyre radius (both PROVISIONAL) without re-deriving the
// throttle full scale, or raises l_max_erpm, this test fails and says which number moved.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>
#include <vehicle_params_generated.hpp>

#include "racer_drivers/vesc_odometry.hpp"

namespace vo = racer_drivers::vesc_odometry;

namespace {

/// The binding's three fields, exactly as vesc_odometry_node assembles them.
vo::DrivetrainConfig from_binding() {
  vo::DrivetrainConfig config;
  config.pole_pairs = static_cast<double>(VEHICLE_PARAMS.drivetrain.pole_pairs.value());
  config.gear_ratio = VEHICLE_PARAMS.drivetrain.gear_ratio.value();
  config.wheel_radius_m = VEHICLE_PARAMS.tires.nominal_radius_m.value();
  return config;
}

/// Newest config/vesc/*-motor.xml by file name (the names are ISO-dated, with a letter suffix
/// for a second export the same day, so lexical order is chronological order).
std::filesystem::path newest_motor_export() {
  std::vector<std::filesystem::path> exports;
  for (const auto& entry : std::filesystem::directory_iterator(RACER_VESC_CONFIG_DIR)) {
    const std::string name = entry.path().filename().string();
    if (name.size() > 10 && name.substr(name.size() - 10) == "-motor.xml") {
      exports.push_back(entry.path());
    }
  }
  if (exports.empty()) {
    return {};
  }
  std::sort(exports.begin(), exports.end());
  return exports.back();
}

/// <l_max_erpm>N</l_max_erpm> from a VESC Tool export, or NaN if absent.
double read_l_max_erpm(const std::filesystem::path& path) {
  std::ifstream file(path);
  std::stringstream buffer;
  buffer << file.rdbuf();
  const std::string text = buffer.str();
  std::smatch match;
  if (!std::regex_search(text, match,
                         std::regex(R"(<l_max_erpm>\s*([-0-9.eE+]+)\s*</l_max_erpm>)"))) {
    return std::nan("");
  }
  return std::stod(match[1].str());
}

}  // namespace

TEST(VescOdometryBinding, DrivetrainFieldsArePresentAndValid) {
  ASSERT_TRUE(VEHICLE_PARAMS.drivetrain.pole_pairs.has_value());
  ASSERT_TRUE(VEHICLE_PARAMS.drivetrain.gear_ratio.has_value());
  ASSERT_TRUE(VEHICLE_PARAMS.tires.nominal_radius_m.has_value());
  EXPECT_FALSE(vo::validate(from_binding())) << *vo::validate(from_binding());
}

TEST(VescOdometryBinding, ScaleFactorIsTheDerivationFromTheBinding) {
  // m/s per ERPM = 2 pi r / (60 * pole_pairs * gear_ratio), written out from the three binding
  // fields so a change to the header's arithmetic cannot pass by changing both sides.
  const double pole_pairs = static_cast<double>(*VEHICLE_PARAMS.drivetrain.pole_pairs);
  const double gear_ratio = *VEHICLE_PARAMS.drivetrain.gear_ratio;
  const double radius_m = *VEHICLE_PARAMS.tires.nominal_radius_m;
  const double expected = 2.0 * M_PI * radius_m / (60.0 * pole_pairs * gear_ratio);
  EXPECT_NEAR(vo::mps_per_erpm(from_binding()), expected, expected * 1e-12);
}

TEST(VescOdometryBinding, VescErpmCapReproducesThrottleFullScale) {
  const std::filesystem::path motor_xml = newest_motor_export();
  ASSERT_FALSE(motor_xml.empty()) << "no *-motor.xml under " << RACER_VESC_CONFIG_DIR;
  const double l_max_erpm = read_l_max_erpm(motor_xml);
  ASSERT_TRUE(std::isfinite(l_max_erpm)) << "no <l_max_erpm> in " << motor_xml;
  ASSERT_TRUE(VEHICLE_PARAMS.actuation.throttle_full_scale_mps.has_value());

  const double full_scale_mps = *VEHICLE_PARAMS.actuation.throttle_full_scale_mps;
  const double derived_mps = vo::erpm_to_wheel_speed_mps(l_max_erpm, from_binding());
  // throttle_full_scale_mps is stored to two decimals, so agreement "within rounding" is
  // half of 0.01 m/s.
  EXPECT_NEAR(derived_mps, full_scale_mps, 0.005)
      << motor_xml.filename() << " l_max_erpm " << l_max_erpm << " -> " << derived_mps
      << " m/s with pole_pairs " << *VEHICLE_PARAMS.drivetrain.pole_pairs << ", gear_ratio "
      << *VEHICLE_PARAMS.drivetrain.gear_ratio << ", nominal_radius_m "
      << *VEHICLE_PARAMS.tires.nominal_radius_m << "; actuation.throttle_full_scale_mps says "
      << full_scale_mps << ". One of them moved without the other being re-derived.";
  // And in reverse: full-scale throttle corresponds to the VESC's ERPM cap.
  EXPECT_NEAR(vo::wheel_speed_mps_to_erpm(full_scale_mps, from_binding()), l_max_erpm,
              l_max_erpm * 0.005 / full_scale_mps);
}
