#include "racer_planner/bundle_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

#include "racer_planner/sha256.hpp"

namespace racer_planner {

const char* refusal_code_name(RefusalCode code) {
  switch (code) {
    case RefusalCode::ManifestMissing:
      return "ManifestMissing";
    case RefusalCode::ManifestInvalid:
      return "ManifestInvalid";
    case RefusalCode::BundleSchemaVersionMismatch:
      return "BundleSchemaVersionMismatch";
    case RefusalCode::KindMismatch:
      return "KindMismatch";
    case RefusalCode::PayloadMissing:
      return "PayloadMissing";
    case RefusalCode::ChecksumMismatch:
      return "ChecksumMismatch";
    case RefusalCode::VehicleParamsMismatch:
      return "VehicleParamsMismatch";
    case RefusalCode::IoContractMismatch:
      return "IoContractMismatch";
    case RefusalCode::GoldenRolloutMismatch:
      return "GoldenRolloutMismatch";
    case RefusalCode::ValidityExceeded:
      return "ValidityExceeded";
  }
  return "ManifestInvalid";
}

namespace {

struct RefusalError {
  Refusal refusal;
};

[[noreturn]] void refuse(RefusalCode code, const std::string& message) {
  throw RefusalError{Refusal{code, message}};
}

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}

std::string scalar_of(const YAML::Node& n, RefusalCode code, const std::string& what) {
  if (!n || !n.IsScalar()) refuse(code, what + " is missing or not a scalar");
  return n.Scalar();
}

double double_of(const YAML::Node& n, RefusalCode code, const std::string& what) {
  const std::string s = scalar_of(n, code, what);
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0' || !std::isfinite(v)) {
    refuse(code, what + " is not a finite number: " + s);
  }
  return v;
}

bool bool_of(const YAML::Node& n, RefusalCode code, const std::string& what) {
  const std::string s = lower(scalar_of(n, code, what));
  if (s == "true") return true;
  if (s == "false") return false;
  refuse(code, what + " is not a boolean: " + s);
}

bool read_file_bytes(const std::string& path, std::vector<unsigned char>* out) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  out->assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  return !in.bad();
}

bool file_exists(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return static_cast<bool>(in);
}

bool plain_file_name(const std::string& f) {
  return !f.empty() && f.find('/') == std::string::npos && f.find('\\') == std::string::npos &&
         f.find("..") == std::string::npos;
}

bool is_hex64(const std::string& s) {
  if (s.size() != 64) return false;
  return std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isxdigit(c) != 0; });
}

void expect_string_list(const YAML::Node& n, const std::vector<std::string>& expected,
                        const std::string& what) {
  if (!n || !n.IsSequence() || n.size() != expected.size()) {
    refuse(RefusalCode::IoContractMismatch, what + " differs from what the planner implements");
  }
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (!n[i].IsScalar() || n[i].Scalar() != expected[i]) {
      refuse(RefusalCode::IoContractMismatch,
             what + " differs from what the planner implements (entry " + std::to_string(i) +
                 ", expected " + expected[i] + ")");
    }
  }
}

template <std::size_t N>
std::array<double, N> double_array(const YAML::Node& n, const std::string& what) {
  if (!n || !n.IsSequence() || n.size() != N) {
    refuse(RefusalCode::IoContractMismatch, what + " must be a list of " + std::to_string(N));
  }
  std::array<double, N> out{};
  for (std::size_t i = 0; i < N; ++i) {
    out[i] = double_of(n[i], RefusalCode::IoContractMismatch, what);
  }
  return out;
}

std::optional<int> major_of(const std::string& v) {
  std::size_t i = 0;
  while (i < v.size() && std::isdigit(static_cast<unsigned char>(v[i]))) ++i;
  if (i == 0 || i > 6) return std::nullopt;
  if (i < v.size() && v[i] != '.') return std::nullopt;
  return std::atoi(v.substr(0, i).c_str());
}

std::vector<std::string> split_csv(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  std::stringstream ss(line);
  while (std::getline(ss, cur, ',')) {
    while (!cur.empty() && (cur.back() == '\r' || cur.back() == ' ')) cur.pop_back();
    out.push_back(cur);
  }
  return out;
}

bool parse_double(const std::string& s, double* out) {
  if (s.empty()) return false;
  char* end = nullptr;
  *out = std::strtod(s.c_str(), &end);
  return end != s.c_str() && *end == '\0';
}

}  // namespace

std::optional<Refusal> verify_golden_rollouts(const DynamicsParams& params,
                                              const VehicleGeometry& geometry,
                                              const std::string& csv_path, double* max_abs_error) {
  auto fail = [](const std::string& m) {
    return std::optional<Refusal>(Refusal{RefusalCode::GoldenRolloutMismatch, m});
  };
  try {
    std::ifstream in(csv_path);
    if (!in) return fail("cannot read " + csv_path);
    std::string line;
    if (!std::getline(in, line)) return fail("golden rollouts file is empty");
    while (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != "rollout,step,speed_cmd_mps,steering_cmd_rad,x_m,y_m,yaw_rad,v_mps,delta_rad") {
      return fail("golden rollouts header is not the contract header: " + line);
    }
    struct Row {
      Control u;
      State s;
    };
    std::map<long, std::vector<Row>> rollouts;
    long line_no = 1;
    while (std::getline(in, line)) {
      ++line_no;
      if (line.empty() || line == "\r") continue;
      const auto f = split_csv(line);
      double v[9];
      if (f.size() != 9) return fail("golden row " + std::to_string(line_no) + " has not 9 fields");
      for (int i = 0; i < 9; ++i) {
        if (!parse_double(f[i], &v[i]) || !std::isfinite(v[i])) {
          return fail("golden row " + std::to_string(line_no) + " has a non-numeric field");
        }
      }
      Row r;
      r.u = Control{v[2], v[3]};
      r.s = State{v[4], v[5], v[6], v[7], v[8]};
      auto& rows = rollouts[static_cast<long>(v[0])];
      if (static_cast<long>(v[1]) != static_cast<long>(rows.size())) {
        return fail("golden rollout " + std::to_string(static_cast<long>(v[0])) +
                    " steps are not consecutive from 0");
      }
      rows.push_back(r);
    }
    if (rollouts.size() < 8) return fail("fewer than 8 golden rollouts");
    const DynamicsModel model(params, geometry, params.dt_s);
    double worst = 0.0;
    for (const auto& kv : rollouts) {
      const auto& rows = kv.second;
      if (rows.size() < 2) return fail("a golden rollout has fewer than 2 rows");
      State s = rows[0].s;
      for (std::size_t i = 0; i + 1 < rows.size(); ++i) {
        s = model.step(s, rows[i].u);
        const State& g = rows[i + 1].s;
        const double e[5] = {std::fabs(s.x - g.x), std::fabs(s.y - g.y), std::fabs(s.yaw - g.yaw),
                             std::fabs(s.v - g.v), std::fabs(s.delta - g.delta)};
        for (double x : e) {
          if (!(x <= kGoldenTolerance)) {
            return fail("rollout " + std::to_string(kv.first) + " step " + std::to_string(i + 1) +
                        " differs from the model by " + std::to_string(x) + " (limit 1e-4)");
          }
          worst = std::max(worst, x);
        }
      }
    }
    if (max_abs_error != nullptr) *max_abs_error = worst;
    return std::nullopt;
  } catch (...) {
    return fail("golden rollouts could not be evaluated");
  }
}

LoadResult load_dynamics_bundle(const std::string& bundle_dir, const VehicleGeometry& geometry,
                                double requested_v_max_mps) {
  LoadResult result;
  RefusalCode stage = RefusalCode::ManifestInvalid;  // code for an unexpected parse error
  try {
    const std::string dir =
        bundle_dir.empty() || bundle_dir.back() == '/' ? bundle_dir : bundle_dir + "/";
    // 1. ManifestMissing
    const std::string manifest_path = dir + "manifest.yaml";
    if (!file_exists(manifest_path)) {
      refuse(RefusalCode::ManifestMissing, "no manifest.yaml in " + bundle_dir);
    }
    YAML::Node root;
    try {
      root = YAML::LoadFile(manifest_path);
    } catch (const std::exception& e) {
      refuse(RefusalCode::ManifestMissing,
             std::string("manifest.yaml does not parse: ") + e.what());
    }
    // 2. ManifestInvalid: exactly the contract's top level keys, right types.
    if (!root || !root.IsMap()) refuse(RefusalCode::ManifestInvalid, "manifest is not a mapping");
    const std::set<std::string> expected = {"bundle_schema_version",
                                            "kind",
                                            "model_id",
                                            "created_utc",
                                            "git_sha",
                                            "training_config_sha256",
                                            "vehicle_params",
                                            "payload",
                                            "io",
                                            "data",
                                            "metrics"};
    std::set<std::string> present;
    for (const auto& kv : root) {
      if (!kv.first.IsScalar()) refuse(RefusalCode::ManifestInvalid, "non scalar top level key");
      present.insert(kv.first.Scalar());
    }
    for (const auto& k : expected) {
      if (present.count(k) == 0) refuse(RefusalCode::ManifestInvalid, "missing field " + k);
    }
    for (const auto& k : present) {
      if (expected.count(k) == 0)
        refuse(RefusalCode::ManifestInvalid, "extra top level field " + k);
    }
    const RefusalCode inv = RefusalCode::ManifestInvalid;
    const std::string schema_version =
        scalar_of(root["bundle_schema_version"], inv, "bundle_schema_version");
    const std::string kind = scalar_of(root["kind"], inv, "kind");
    const std::string model_id = scalar_of(root["model_id"], inv, "model_id");
    scalar_of(root["created_utc"], inv, "created_utc");
    scalar_of(root["git_sha"], inv, "git_sha");
    scalar_of(root["training_config_sha256"], inv, "training_config_sha256");
    const YAML::Node vp = root["vehicle_params"];
    if (!vp.IsMap()) refuse(inv, "vehicle_params is not a mapping");
    const std::string vp_schema =
        scalar_of(vp["schema_version"], inv, "vehicle_params.schema_version");
    const std::string vp_sysid =
        scalar_of(vp["sysid_session_id"], inv, "vehicle_params.sysid_session_id");
    const YAML::Node payload_node = root["payload"];
    if (!payload_node.IsSequence()) refuse(inv, "payload is not a list");
    std::vector<PayloadChecksum> payload;
    for (const auto& item : payload_node) {
      if (!item.IsMap()) refuse(inv, "a payload entry is not a mapping");
      PayloadChecksum p;
      p.file = scalar_of(item["file"], inv, "payload file");
      p.sha256 = lower(scalar_of(item["sha256"], inv, "payload sha256"));
      if (!plain_file_name(p.file)) refuse(inv, "payload file name is not a plain file name");
      payload.push_back(p);
    }
    const YAML::Node io = root["io"];
    if (!io.IsMap()) refuse(inv, "io is not a mapping");
    if (!root["data"].IsSequence()) refuse(inv, "data is not a list");
    if (!root["metrics"].IsMap()) refuse(inv, "metrics is not a mapping");

    // 3. BundleSchemaVersionMismatch
    const auto major = major_of(schema_version);
    if (!major.has_value())
      refuse(inv, "bundle_schema_version is not a version: " + schema_version);
    if (*major != 1) {
      refuse(RefusalCode::BundleSchemaVersionMismatch, "bundle_schema_version " + schema_version +
                                                           " has major " + std::to_string(*major) +
                                                           ", the planner loads major 1");
    }
    // 4. KindMismatch
    if (kind != "dynamics") {
      refuse(RefusalCode::KindMismatch, "kind is '" + kind + "', the planner deploys 'dynamics'");
    }
    // 5. PayloadMissing and 6. ChecksumMismatch
    for (const auto& p : payload) {
      if (!file_exists(dir + p.file))
        refuse(RefusalCode::PayloadMissing, "payload file " + p.file + " is absent");
    }
    for (const auto& p : payload) {
      std::string actual;
      if (!sha256_file_hex(dir + p.file, &actual)) {
        refuse(RefusalCode::PayloadMissing, "payload file " + p.file + " cannot be read");
      }
      if (!is_hex64(p.sha256) || actual != p.sha256) {
        refuse(RefusalCode::ChecksumMismatch,
               "payload file " + p.file + " has sha256 " + actual + ", manifest says " + p.sha256);
      }
    }
    // 7. VehicleParamsMismatch
    if (vp_schema != geometry.schema_version) {
      refuse(RefusalCode::VehicleParamsMismatch, "bundle vehicle_params.schema_version " +
                                                     vp_schema + " differs from the live " +
                                                     geometry.schema_version);
    }
    if (vp_sysid != geometry.sysid_session_id) {
      refuse(RefusalCode::VehicleParamsMismatch, "bundle vehicle_params.sysid_session_id " +
                                                     vp_sysid + " differs from the live " +
                                                     geometry.sysid_session_id);
    }
    // 8. IoContractMismatch
    const RefusalCode iom = RefusalCode::IoContractMismatch;
    stage = iom;
    DynamicsParams params;
    expect_string_list(io["state"], {"x_m", "y_m", "yaw_rad", "v_mps", "delta_rad"}, "io.state");
    expect_string_list(io["control"], {"speed_cmd_mps", "steering_cmd_rad"}, "io.control");
    params.dt_s = double_of(io["dt_s"], iom, "io.dt_s");
    if (params.dt_s < 0.01 || params.dt_s > 0.1) {
      refuse(iom,
             "io.dt_s " + std::to_string(params.dt_s) + " is outside the supported 0.01 to 0.1 s");
    }
    const YAML::Node nom = io["nominal"];
    if (!nom.IsMap()) refuse(iom, "io.nominal is not a mapping");
    NominalParams& n = params.nominal;
    n.speed_gain = double_of(nom["speed_gain"], iom, "nominal.speed_gain");
    n.speed_tau_s = double_of(nom["speed_tau_s"], iom, "nominal.speed_tau_s");
    n.accel_max_mps2 = double_of(nom["accel_max_mps2"], iom, "nominal.accel_max_mps2");
    n.decel_max_mps2 = double_of(nom["decel_max_mps2"], iom, "nominal.decel_max_mps2");
    n.steer_gain_left = double_of(nom["steer_gain_left"], iom, "nominal.steer_gain_left");
    n.steer_gain_right = double_of(nom["steer_gain_right"], iom, "nominal.steer_gain_right");
    n.steer_offset_rad = double_of(nom["steer_offset_rad"], iom, "nominal.steer_offset_rad");
    n.steer_tau_s = double_of(nom["steer_tau_s"], iom, "nominal.steer_tau_s");
    n.command_delay_s = double_of(nom["command_delay_s"], iom, "nominal.command_delay_s");
    n.understeer_s2_per_m2 = double_of(nom["understeer_s2_per_m2"], iom, "nominal.understeer");
    if (n.speed_tau_s <= 0.0 || n.steer_tau_s <= 0.0 || n.accel_max_mps2 <= 0.0 ||
        n.decel_max_mps2 <= 0.0 || n.speed_gain <= 0.0 || n.steer_gain_left <= 0.0 ||
        n.steer_gain_right <= 0.0 || n.command_delay_s < 0.0 || n.command_delay_s > 0.5 ||
        n.understeer_s2_per_m2 < 0.0) {
      refuse(iom, "io.nominal has a value outside the range the planner implements");
    }
    const YAML::Node res = io["residual"];
    if (!res.IsMap()) refuse(iom, "io.residual is not a mapping");
    params.residual.enabled = bool_of(res["enabled"], iom, "residual.enabled");
    expect_string_list(res["inputs"], {"v_mps", "delta_rad", "speed_cmd_mps", "steering_cmd_rad"},
                       "residual.inputs");
    expect_string_list(res["outputs"], {"dv_dt_mps2", "dyaw_dt_rad_per_s"}, "residual.outputs");
    const YAML::Node layers = res["layers"];
    const int expect_layers[3][2] = {
        {kMlpIn, kMlpHidden}, {kMlpHidden, kMlpHidden}, {kMlpHidden, kMlpOut}};
    if (!layers.IsSequence() || layers.size() != 3) refuse(iom, "residual.layers shape differs");
    for (int i = 0; i < 3; ++i) {
      if (!layers[i].IsSequence() || layers[i].size() != 2 ||
          double_of(layers[i][0], iom, "layer") != expect_layers[i][0] ||
          double_of(layers[i][1], iom, "layer") != expect_layers[i][1]) {
        refuse(iom, "residual.layers shape differs from [[4,32],[32,32],[32,2]]");
      }
    }
    if (scalar_of(res["activation"], iom, "residual.activation") != "tanh") {
      refuse(iom, "residual.activation is not tanh");
    }
    params.residual.input_mean = double_array<kMlpIn>(res["input_mean"], "residual.input_mean");
    params.residual.input_std = double_array<kMlpIn>(res["input_std"], "residual.input_std");
    params.residual.output_scale =
        double_array<kMlpOut>(res["output_scale"], "residual.output_scale");
    for (double s : params.residual.input_std) {
      if (s == 0.0) refuse(iom, "residual.input_std has a zero entry");
    }
    const std::string residual_file = scalar_of(res["file"], iom, "residual.file");
    const std::string golden_file = scalar_of(io["golden"], iom, "io.golden");
    if (!plain_file_name(residual_file) || !plain_file_name(golden_file)) {
      refuse(iom, "residual.file or io.golden is not a plain file name");
    }
    auto listed = [&](const std::string& f) {
      return std::any_of(payload.begin(), payload.end(),
                         [&](const PayloadChecksum& p) { return p.file == f; });
    };
    if (!listed(residual_file))
      refuse(RefusalCode::PayloadMissing, residual_file + " is not listed in payload");
    if (!listed(golden_file))
      refuse(RefusalCode::PayloadMissing, golden_file + " is not listed in payload");
    const YAML::Node validity = io["validity"];
    if (!validity.IsMap()) refuse(iom, "io.validity is not a mapping");
    params.v_min_mps = double_of(validity["v_min_mps"], iom, "validity.v_min_mps");
    params.v_max_mps = double_of(validity["v_max_mps"], iom, "validity.v_max_mps");
    if (params.v_min_mps < 0.0 || params.v_max_mps <= params.v_min_mps) {
      refuse(iom, "io.validity is not a sensible forward speed range");
    }
    // residual weights: raw float32 little endian
    std::vector<unsigned char> bytes;
    if (!read_file_bytes(dir + residual_file, &bytes))
      refuse(RefusalCode::PayloadMissing, residual_file + " cannot be read");
    if (bytes.size() != static_cast<std::size_t>(kMlpWeightCount) * 4) {
      refuse(iom, residual_file + " has " + std::to_string(bytes.size()) + " bytes, layers imply " +
                      std::to_string(kMlpWeightCount * 4));
    }
    params.residual.weights.resize(kMlpWeightCount);
    for (int i = 0; i < kMlpWeightCount; ++i) {
      const std::uint32_t u = std::uint32_t(bytes[4 * i]) | (std::uint32_t(bytes[4 * i + 1]) << 8) |
                              (std::uint32_t(bytes[4 * i + 2]) << 16) |
                              (std::uint32_t(bytes[4 * i + 3]) << 24);
      float f;
      std::memcpy(&f, &u, sizeof(f));
      if (!std::isfinite(f)) refuse(iom, residual_file + " holds a non finite weight");
      params.residual.weights[i] = f;
    }
    // 9. GoldenRolloutMismatch
    double worst = 0.0;
    const auto golden = verify_golden_rollouts(params, geometry, dir + golden_file, &worst);
    if (golden.has_value()) throw RefusalError{*golden};
    // 10. ValidityExceeded
    if (!std::isfinite(requested_v_max_mps) || requested_v_max_mps > params.v_max_mps) {
      refuse(RefusalCode::ValidityExceeded,
             "requested speed cap " + std::to_string(requested_v_max_mps) +
                 " m/s is above the bundle validity v_max_mps " + std::to_string(params.v_max_mps));
    }
    DynamicsBundle bundle;
    bundle.model_id = model_id;
    bundle.params = params;
    bundle.payload = payload;
    bundle.golden_max_abs_error = worst;
    result.bundle = std::move(bundle);
  } catch (const RefusalError& e) {
    result.bundle.reset();
    result.refusal = e.refusal;
  } catch (const std::exception& e) {
    result.bundle.reset();
    result.refusal = Refusal{stage, std::string("unexpected error: ") + e.what()};
  } catch (...) {
    result.bundle.reset();
    result.refusal = Refusal{stage, "unexpected error"};
  }
  return result;
}

}  // namespace racer_planner
