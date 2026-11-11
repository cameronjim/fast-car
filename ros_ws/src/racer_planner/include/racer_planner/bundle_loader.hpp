// Dynamics bundle loader (claude-docs/13 section 4.4). Checks every row of the refusal table
// that applies to kind "dynamics", in table order, and returns either the parameters or a
// Refusal naming the error. No exception escapes, no override, no downgrade to a warning.
#ifndef RACER_PLANNER_BUNDLE_LOADER_HPP_
#define RACER_PLANNER_BUNDLE_LOADER_HPP_

#include <optional>
#include <string>
#include <vector>

#include "racer_planner/dynamics_model.hpp"
#include "racer_planner/vehicle_geometry.hpp"

namespace racer_planner {

enum class RefusalCode {
  ManifestMissing,
  ManifestInvalid,
  BundleSchemaVersionMismatch,
  KindMismatch,
  PayloadMissing,
  ChecksumMismatch,
  VehicleParamsMismatch,
  IoContractMismatch,
  GoldenRolloutMismatch,
  ValidityExceeded,
};

const char* refusal_code_name(RefusalCode code);

struct Refusal {
  RefusalCode code;
  std::string message;
};

struct PayloadChecksum {
  std::string file;
  std::string sha256;
};

struct DynamicsBundle {
  std::string model_id;
  DynamicsParams params;
  std::vector<PayloadChecksum> payload;
  double golden_max_abs_error = 0.0;
};

struct LoadResult {
  std::optional<DynamicsBundle> bundle;
  std::optional<Refusal> refusal;
  bool ok() const { return bundle.has_value(); }
};

// The golden rollouts must be reproduced within this absolute tolerance in every state.
constexpr double kGoldenTolerance = 1e-4;

// requested_v_max_mps is the speed cap the node wants to run; above io.validity.v_max_mps it is
// a ValidityExceeded refusal, never a clamp.
LoadResult load_dynamics_bundle(const std::string& bundle_dir, const VehicleGeometry& geometry,
                                double requested_v_max_mps);

// Runs every rollout of a golden CSV through the model from its first row and compares each
// state. Returns a GoldenRolloutMismatch refusal on any problem. max_abs_error may be null.
std::optional<Refusal> verify_golden_rollouts(const DynamicsParams& params,
                                              const VehicleGeometry& geometry,
                                              const std::string& csv_path, double* max_abs_error);

}  // namespace racer_planner

#endif  // RACER_PLANNER_BUNDLE_LOADER_HPP_
