cmake_minimum_required(VERSION 3.8)
project(hexapod_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find installed libraries from hexapod_ik and hexapod_gait
find_library(IK_SOLVER_LIB ik_solver PATHS ${CMAKE_SOURCE_DIR}/../hexapod_ik)
find_library(GAIT_GEN_LIB gait_generator PATHS ${CMAKE_SOURCE_DIR}/../hexapod_gait)

# =============================================================================
# Control Orchestrator node
# =============================================================================
add_executable(control_node_exec src/control_node.cpp)
target_include_directories(control_node_exec PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../hexapod_ik/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../hexapod_gait/include>
)
target_link_libraries(control_node_exec ${IK_SOLVER_LIB} ${GAIT_GEN_LIB} yaml-cpp)
ament_target_dependencies(control_node_exec
  rclcpp geometry_msgs sensor_msgs
)

# =============================================================================
# Gait cycle test executable
# =============================================================================
add_executable(test_gait_cycle_exec src/test_gait_cycle.cpp)
target_include_directories(test_gait_cycle_exec PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../hexapod_ik/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../hexapod_gait/include>
)
target_link_libraries(test_gait_cycle_exec ${IK_SOLVER_LIB} ${GAIT_GEN_LIB})
ament_target_dependencies(test_gait_cycle_exec
  rclcpp sensor_msgs
)

# =============================================================================
# Pose test executable (FK verification, joint sweep)
# =============================================================================
add_executable(pose_test_node src/pose_test_node.cpp)
target_include_directories(pose_test_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../hexapod_ik/include>
)
target_link_libraries(pose_test_node ${IK_SOLVER_LIB} yaml-cpp)
ament_target_dependencies(pose_test_node
  rclcpp sensor_msgs
)

# =============================================================================
# Install
# =============================================================================
install(TARGETS control_node_exec test_gait_cycle_exec pose_test_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
