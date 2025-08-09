cmake_minimum_required(VERSION 3.8)
project(gate3_position_publisher)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_bridge_msgs REQUIRED)

find_package(gz-transport12 REQUIRED)
find_package(gz-msgs9 REQUIRED)

add_executable(gates_position_publisher src/gates_position_publisher.cpp)

ament_target_dependencies(gates_position_publisher
  rclcpp
  geometry_msgs
  my_bridge_msgs
)

target_link_libraries(gates_position_publisher
  gz-transport12::gz-transport12
  gz-msgs9::gz-msgs9
)

install(TARGETS gates_position_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
