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
find_package(geometric_shapes REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

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
# xArm SDK (for guide/teach mode)
target_include_directories(motion_planner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../external/xarm_ros2/xarm_sdk/cxx/include
)
target_link_libraries(motion_planner
  nlohmann_json::nlohmann_json
  ${CMAKE_CURRENT_SOURCE_DIR}/../../install/xarm_sdk/lib/libxarm_cxx_sdk.so
)

add_executable(scene_manager src/scene_manager_node.cpp)
target_compile_features(scene_manager PUBLIC cxx_std_17)
ament_target_dependencies(
  scene_manager
  rclcpp
  moveit_ros_planning_interface
  geometric_shapes
  shape_msgs
  moveit_msgs
  catalyst_interfaces
)
target_link_libraries(scene_manager nlohmann_json::nlohmann_json yaml-cpp)

install(TARGETS motion_planner scene_manager
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
