cmake_minimum_required(VERSION 3.8)
project(hexapod_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# =============================================================================
# Task 5: IK Solver library (pure math, no ROS dependency)
# =============================================================================
add_library(ik_solver SHARED src/ik_solver.cpp)
target_include_directories(ik_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ik_solver PUBLIC cxx_std_17)

# =============================================================================
# Task 5: Unit tests for IK solver
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ik_solver test/test_ik_solver.cpp)
  target_link_libraries(test_ik_solver ik_solver)
  target_include_directories(test_ik_solver PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

# =============================================================================
# Install targets
# =============================================================================
install(TARGETS ik_solver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
