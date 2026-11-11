cmake_minimum_required(VERSION 3.8)
project(racer_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Keypoint2D must come before the messages that contain it.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyEvent.msg"
  "msg/Keypoint2D.msg"
  "msg/ObjectDetection.msg"
  "msg/ObjectDetectionArray.msg"
  "msg/PoseDetection.msg"
  "msg/PoseDetectionArray.msg"
  "msg/SpiderbotAlert.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()
