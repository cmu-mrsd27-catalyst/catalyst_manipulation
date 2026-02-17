cmake_minimum_required(VERSION 3.8)
project(catalyst_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(catalyst_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(motion_planner src/motion_planner_node.cpp)
target_compile_features(motion_planner PUBLIC cxx_std_17)
ament_target_dependencies(
  motion_planner
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  catalyst_interfaces
  sensor_msgs
)
target_link_libraries(motion_planner nlohmann_json::nlohmann_json)

install(TARGETS motion_planner
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
