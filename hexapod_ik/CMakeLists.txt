cmake_minimum_required(VERSION 3.8)
project(hexapod_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# =============================================================================
# Task 5: IK Solver library (pure math, no ROS dependency)
# =============================================================================
# TODO: Uncomment when implementing ik_solver.cpp
# add_library(ik_solver SHARED src/ik_solver.cpp)
# target_include_directories(ik_solver PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# =============================================================================
# Task 5: Unit tests for IK solver
# =============================================================================
# TODO: Uncomment when implementing test_ik_solver.cpp
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(test_ik_solver test/test_ik_solver.cpp)
#   target_link_libraries(test_ik_solver ik_solver)
# endif()

# =============================================================================
# Task 5: Install targets
# =============================================================================
# TODO: Uncomment when you have library targets
# install(TARGETS ik_solver
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )
#
# install(DIRECTORY include/
#   DESTINATION include
# )

ament_package()
