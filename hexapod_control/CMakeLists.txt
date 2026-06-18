cmake_minimum_required(VERSION 3.8)
project(hexapod_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hexapod_servo REQUIRED)
find_package(hexapod_ik REQUIRED)
find_package(hexapod_gait REQUIRED)

# =============================================================================
# Task 7: Control Orchestrator node
# =============================================================================
# TODO: Uncomment when implementing control_node.cpp
#
# add_library(control_node SHARED src/control_node.cpp)
# target_include_directories(control_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(control_node
#   rclcpp rclcpp_components geometry_msgs sensor_msgs
#   hexapod_servo hexapod_ik hexapod_gait
# )
#
# rclcpp_components_register_node(control_node
#   PLUGIN "hexapod_control::ControlNode"
#   EXECUTABLE control_node_exec
# )

# =============================================================================
# Task 9: Integration test
# =============================================================================
# TODO: Uncomment when implementing test_control_node.cpp
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(test_control_node test/test_control_node.cpp)
#   target_link_libraries(test_control_node control_node)
# endif()

# =============================================================================
# Install targets
# =============================================================================
# TODO: Uncomment when you have library targets
# install(TARGETS control_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )
#
# install(DIRECTORY include/
#   DESTINATION include
# )

ament_package()
