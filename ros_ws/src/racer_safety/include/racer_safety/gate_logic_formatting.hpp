// Internal only -- NOT part of gate_logic.hpp's public API, and deliberately not exposed to
// safety_node.cpp. These are /safety/events `detail` string-formatting helpers used
// exclusively by gate_logic.cpp's SafetyGateLogic::evaluate(). They live in their own
// translation unit (src/gate_logic_formatting.cpp) precisely so gate_logic.cpp/.hpp's 100%
// branch-coverage gate (claude-docs/12-testing.md) measures only actual DECISION logic:
// std::to_string()/std::string operator+ chains are implemented as inline/template code in
// libstdc++ that an unoptimized (-O0, used for the coverage build so counts are predictable)
// compile attributes to the CALLING line, which showed up as ~30 "uncovered branches" purely
// from message formatting, not from any untested decision path, the first time this file was
// written with the formatting inlined directly into evaluate(). Moving it out is the honest
// fix the task anticipated ("if lcov branch counting produces noise ... scope the report to
// the gate source files") -- gate_logic.cpp now contains only the actual gate decisions, and
// this file is excluded from the coverage filter entirely (its own coverage is unmeasured,
// which is correct: formatting a log message is not "decision logic").
#ifndef RACER_SAFETY_GATE_LOGIC_FORMATTING_HPP_
#define RACER_SAFETY_GATE_LOGIC_FORMATTING_HPP_

#include <string>

namespace racer_safety::formatting {

std::string watchdog_detail(double timeout_s, double age_s);
std::string command_sanity_detail();
std::string bounds_clamp_detail(double steering_min_rad, double steering_max_rad,
                                double speed_min_mps, double speed_max_mps);
std::string rate_limit_detail(double dt_s);
std::string ttc_brake_detail(double ttc_s, double brake_threshold_s);
std::string ttc_warning_detail(double ttc_s, double warning_threshold_s);
std::string covariance_detail(double speed_fraction);

}  // namespace racer_safety::formatting

#endif  // RACER_SAFETY_GATE_LOGIC_FORMATTING_HPP_
