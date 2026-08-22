// pure_pursuit_cli: tiny CLI harness for the cross-language base-controller divergence test
// (roadmap S.3, claude-docs/12-testing.md's L5 "envelope-in-env test" divergence pattern
// applied to the base controller instead of the envelope).
//
// Reads vehicle poses (x_m,y_m,yaw_rad CSV lines, no header) from a --states file or from
// stdin if --states is omitted, runs each one through the SAME racer_control_core
// PurePursuitController tracker_node.cpp uses, and prints one JSON object per line to
// stdout: {"steering_angle_rad": <v>, "speed_mps": <v>}.
//
// Physical constants (wheelbase, steering limit) come ONLY from the generated
// vehicle_params binding (CLAUDE.md invariant 2), exactly like tracker_node.cpp -- never a
// CLI flag, so this harness can never silently diverge from the real controller's physical
// assumptions by being passed the wrong number. Lookahead tuning gains ARE CLI flags (with
// defaults matching tracker_node.cpp's declared ROS param defaults: 0.4 / 1.5 / 0.4 --
// see that file's build_controller_config()) since they are genuine tuning knobs, not
// physical constants.
//
// This binary has no ROS dependency; it exists purely so
// ros_ws/src/racer_control/test/divergence/compare_divergence.py can run committed states
// (training/racer_train/tests/fixtures/divergence_states.csv, generated deterministically by
// training/racer_train/tests/generate_divergence_fixture.py) through the real C++ core from
// a language-agnostic harness and compare the result to that same script's committed
// divergence_expected.json.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <vehicle_params_generated.hpp>

#include "racer_control/pure_pursuit.hpp"
#include "racer_control/raceline.hpp"

namespace {

struct CliArgs {
  std::string raceline_path;
  std::string states_path;  // empty => read from stdin
  double lookahead_min_m = 0.4;
  double lookahead_max_m = 1.5;
  double lookahead_curvature_ref_1pm = 0.4;
};

void print_usage() {
  std::cerr << "usage: pure_pursuit_cli --raceline <path> [--states <path>] "
               "[--lookahead-min-m <v>] [--lookahead-max-m <v>] "
               "[--lookahead-curvature-ref-1pm <v>]\n"
               "Reads x_m,y_m,yaw_rad CSV lines (no header) from --states, or stdin if "
               "--states is omitted. Prints one JSON object per line to stdout.\n";
}

std::optional<CliArgs> parse_args(int argc, char** argv) {
  CliArgs args;
  bool has_raceline = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next_value = [&](const char* flag) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "pure_pursuit_cli: missing value for " << flag << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "--raceline") {
      args.raceline_path = next_value("--raceline");
      has_raceline = true;
    } else if (arg == "--states") {
      args.states_path = next_value("--states");
    } else if (arg == "--lookahead-min-m") {
      args.lookahead_min_m = std::stod(next_value("--lookahead-min-m"));
    } else if (arg == "--lookahead-max-m") {
      args.lookahead_max_m = std::stod(next_value("--lookahead-max-m"));
    } else if (arg == "--lookahead-curvature-ref-1pm") {
      args.lookahead_curvature_ref_1pm = std::stod(next_value("--lookahead-curvature-ref-1pm"));
    } else {
      std::cerr << "pure_pursuit_cli: unknown argument '" << arg << "'\n";
      return std::nullopt;
    }
  }
  if (!has_raceline) {
    std::cerr << "pure_pursuit_cli: --raceline <path> is required\n";
    return std::nullopt;
  }
  return args;
}

std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

}  // namespace

int main(int argc, char** argv) {
  std::optional<CliArgs> parsed = parse_args(argc, argv);
  if (!parsed) {
    print_usage();
    return 2;
  }
  const CliArgs& args = *parsed;

  racer_control::Raceline raceline = racer_control::Raceline::load_from_csv(args.raceline_path);

  racer_control::PurePursuitConfig config;
  config.wheelbase_m = VEHICLE_PARAMS.chassis.wheelbase_m;
  config.max_steering_angle_rad = VEHICLE_PARAMS.steering.max_angle_rad;
  config.lookahead_min_m = args.lookahead_min_m;
  config.lookahead_max_m = args.lookahead_max_m;
  config.lookahead_curvature_ref_1pm = args.lookahead_curvature_ref_1pm;
  racer_control::PurePursuitController controller(config);

  std::ifstream file_input;
  std::istream* input = &std::cin;
  if (!args.states_path.empty()) {
    file_input.open(args.states_path);
    if (!file_input.is_open()) {
      std::cerr << "pure_pursuit_cli: could not open states file '" << args.states_path << "'\n";
      return 1;
    }
    input = &file_input;
  }

  std::string line;
  while (std::getline(*input, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    std::vector<std::string> fields = split_csv_line(line);
    if (fields.size() != 3) {
      std::cerr << "pure_pursuit_cli: expected 3 fields (x_m,y_m,yaw_rad), got " << fields.size()
                << ": '" << line << "'\n";
      return 1;
    }
    const double x_m = std::stod(fields[0]);
    const double y_m = std::stod(fields[1]);
    const double yaw_rad = std::stod(fields[2]);
    const racer_control::PurePursuitCommand cmd =
        controller.compute_command(raceline, x_m, y_m, yaw_rad);
    // %.17g round-trips a double exactly (17 significant digits is always enough) so the
    // Python-side comparison never loses precision the C++ core itself did not lose.
    std::printf("{\"steering_angle_rad\": %.17g, \"speed_mps\": %.17g}\n", cmd.steering_angle_rad,
                cmd.speed_mps);
  }

  return 0;
}
