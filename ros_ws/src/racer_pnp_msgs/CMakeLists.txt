cmake_minimum_required(VERSION 3.8)
project(racer_pnp_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocalTrack.msg"
  "msg/PlannerStatus.msg"
  "msg/LapEvent.msg"
  "msg/LapGuidance.msg"
  DEPENDENCIES std_msgs
)

ament_package()
