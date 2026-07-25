cmake_minimum_required(VERSION 3.8)
project(hexapod_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# =============================================================================
# IK Solver library (pure math, no ROS dependency)
# =============================================================================
add_library(ik_solver SHARED src/ik_solver.cpp)
target_include_directories(ik_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ik_solver PUBLIC cxx_std_17)

# =============================================================================
# Unit tests for IK solver
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ik_solver test/test_ik_solver.cpp)
  target_link_libraries(test_ik_solver ik_solver)
  target_include_directories(test_ik_solver PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_per_leg_ik test/test_per_leg_ik.cpp)
  target_link_libraries(test_per_leg_ik ik_solver)
  target_include_directories(test_per_leg_ik PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

# =============================================================================
# ROS node (IK → /joint_targets)
# =============================================================================
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(ik_node_exec src/ik_node.cpp)
target_include_directories(ik_node_exec PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(ik_node_exec ik_solver yaml-cpp)
ament_target_dependencies(ik_node_exec rclcpp sensor_msgs geometry_msgs)

add_executable(test_ik_stance_exec src/test_ik_stance.cpp)
target_include_directories(test_ik_stance_exec PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(test_ik_stance_exec ik_solver)
ament_target_dependencies(test_ik_stance_exec rclcpp sensor_msgs)

add_executable(test_all_legs_stance_exec src/test_all_legs_stance.cpp)
target_include_directories(test_all_legs_stance_exec PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(test_all_legs_stance_exec ik_solver)
ament_target_dependencies(test_all_legs_stance_exec rclcpp sensor_msgs)

add_executable(test_leg_ik_seq_exec src/test_leg_ik_seq.cpp)
target_include_directories(test_leg_ik_seq_exec PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(test_leg_ik_seq_exec ik_solver)
ament_target_dependencies(test_leg_ik_seq_exec rclcpp sensor_msgs)

# =============================================================================
# Install
# =============================================================================
install(TARGETS ik_solver ik_node_exec test_ik_stance_exec test_all_legs_stance_exec test_leg_ik_seq_exec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
