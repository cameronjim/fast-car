// Parameter-declaration helpers shared by gap_follow_node and wall_follow_node (GitHub issue
// 26). Internal to racer_control's node executables (not installed, not part of the ROS-free
// core). claude-docs/10-conventions.md: every parameter is declared with a descriptor and a
// range; an out-of-range value is rejected by rclcpp at declaration time, which throws and
// stops the node from starting.
#ifndef RACER_CONTROL_REACTIVE_NODE_PARAMS_HPP_
#define RACER_CONTROL_REACTIVE_NODE_PARAMS_HPP_

#include <cstdint>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace racer_control {

inline double declare_ranged_double(rclcpp::Node& node, const std::string& name,
                                    double default_value, double from_value, double to_value,
                                    const std::string& description) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from_value;
  range.to_value = to_value;
  descriptor.floating_point_range = {range};
  return node.declare_parameter<double>(name, default_value, descriptor);
}

inline int64_t declare_ranged_int(rclcpp::Node& node, const std::string& name,
                                  int64_t default_value, int64_t from_value, int64_t to_value,
                                  const std::string& description) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = from_value;
  range.to_value = to_value;
  range.step = 1;
  descriptor.integer_range = {range};
  return node.declare_parameter<int64_t>(name, default_value, descriptor);
}

inline bool declare_described_bool(rclcpp::Node& node, const std::string& name, bool default_value,
                                   const std::string& description) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  return node.declare_parameter<bool>(name, default_value, descriptor);
}

}  // namespace racer_control

#endif  // RACER_CONTROL_REACTIVE_NODE_PARAMS_HPP_
