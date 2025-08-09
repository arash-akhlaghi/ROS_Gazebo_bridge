cmake_minimum_required(VERSION 3.8)
project(my_bridge_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose2D.msg"
  "msg/GatesPositions.msg"
)

ament_package()

