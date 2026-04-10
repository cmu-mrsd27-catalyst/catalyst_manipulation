cmake_minimum_required(VERSION 3.8)
project(catalyst_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(catalyst_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(bt_executor
  src/bt_executor_node.cpp
  src/json_service_node.cpp
  src/set_bool_service_node.cpp
  src/empty_service_node.cpp
  src/task_action_node.cpp
  src/system_healthy_node.cpp
)
target_compile_features(bt_executor PUBLIC cxx_std_17)
target_include_directories(bt_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(bt_executor
  rclcpp
  rclcpp_action
  behaviortree_cpp
  catalyst_interfaces
  std_srvs
  ament_index_cpp
  std_msgs
)
target_link_libraries(bt_executor nlohmann_json::nlohmann_json)

install(TARGETS bt_executor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
