// Shared helpers for the racer_planner gtests.
#ifndef RACER_PLANNER_TEST_SUPPORT_HPP_
#define RACER_PLANNER_TEST_SUPPORT_HPP_

#include <gtest/gtest.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "binding_geometry.hpp"
#include "racer_planner/dynamics_model.hpp"
#include "racer_planner/sha256.hpp"
#include "racer_planner/vehicle_geometry.hpp"

namespace racer_planner::test {

namespace fs = std::filesystem;

// The car's geometry from the generated vehicle_params binding (never typed into a test).
inline VehicleGeometry binding_geometry() {
  std::string why;
  const auto g = geometry_from_binding(VEHICLE_PARAMS, &why);
  if (!g) {
    ADD_FAILURE() << why;
    return VehicleGeometry{};
  }
  return *g;
}

inline std::string fixture_dir() { return std::string(RACER_PLANNER_FIXTURE_DIR) + "/dyn_fixture"; }

// The vehicle numbers the committed fixture's golden rollouts were generated with, read from its
// sidecar so the fixture tests do not move when an unrelated vehicle_params field does.
inline VehicleGeometry fixture_geometry() {
  const YAML::Node n = YAML::LoadFile(fixture_dir() + "/vehicle_geometry.yaml");
  VehicleGeometry g;
  g.wheelbase_m = n["wheelbase_m"].as<double>();
  g.width_m = n["width_m"].as<double>();
  g.rear_overhang_m = n["rear_overhang_m"].as<double>();
  g.front_overhang_m = n["front_overhang_m"].as<double>();
  g.steering_min_rad = n["steering_min_rad"].as<double>();
  g.steering_max_rad = n["steering_max_rad"].as<double>();
  g.steering_max_rate_rad_per_s = n["steering_max_rate_rad_per_s"].as<double>();
  g.max_acceleration_mps2 = n["max_acceleration_mps2"].as<double>();
  g.stop_deceleration_mps2 = n["stop_deceleration_mps2"].as<double>();
  g.lidar_mount_x_m = n["lidar_mount_x_m"].as<double>();
  g.lidar_mount_y_m = n["lidar_mount_y_m"].as<double>();
  g.lidar_mount_yaw_rad = n["lidar_mount_yaw_rad"].as<double>();
  g.schema_version = n["schema_version"].as<std::string>();
  g.sysid_session_id = n["sysid_session_id"].as<std::string>();
  return g;
}

// Nominal model parameters for tests that need a car but not a fitted one (the contract's
// placeholder shape). Not the car's constants: those come from the binding.
inline DynamicsParams plain_dynamics() {
  DynamicsParams p;
  p.dt_s = 0.05;
  p.v_max_mps = 1.0;
  return p;  // NominalParams defaults, residual off
}

inline std::string read_text(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

inline void write_text(const std::string& path, const std::string& text) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << text;
}

// A scratch copy of the fixture bundle that a test can damage.
class BundleCopy {
 public:
  BundleCopy() {
    static int counter = 0;
    dir_ = fs::temp_directory_path() /
           ("racer_planner_bundle_" + std::to_string(::getpid()) + "_" + std::to_string(counter++));
    fs::remove_all(dir_);
    fs::copy(fixture_dir(), dir_, fs::copy_options::recursive);
  }
  ~BundleCopy() {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  BundleCopy(const BundleCopy&) = delete;
  BundleCopy& operator=(const BundleCopy&) = delete;

  std::string dir() const { return dir_.string(); }
  std::string path(const std::string& name) const { return (dir_ / name).string(); }

  // Replaces the first occurrence of `from` in the manifest; fails the test when it is absent.
  void edit_manifest(const std::string& from, const std::string& to) {
    std::string text = read_text(path("manifest.yaml"));
    const auto pos = text.find(from);
    ASSERT_NE(pos, std::string::npos) << "manifest has no '" << from << "'";
    text.replace(pos, from.size(), to);
    write_text(path("manifest.yaml"), text);
  }

  // Rewrites a payload file and updates its checksum in the manifest, so only the content
  // problem is under test.
  void rewrite_payload(const std::string& file, const std::string& new_content) {
    std::string old_sha;
    ASSERT_TRUE(sha256_file_hex(path(file), &old_sha));
    write_text(path(file), new_content);
    std::string new_sha;
    ASSERT_TRUE(sha256_file_hex(path(file), &new_sha));
    edit_manifest(old_sha, new_sha);
  }

 private:
  fs::path dir_;
};

}  // namespace racer_planner::test

#endif  // RACER_PLANNER_TEST_SUPPORT_HPP_
