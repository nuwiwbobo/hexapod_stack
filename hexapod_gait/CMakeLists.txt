cmake_minimum_required(VERSION 3.8)
project(hexapod_gait)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# =============================================================================
# Task 6: Wave Gait Generator library (pure math, no ROS dependency)
# =============================================================================
# TODO: Uncomment when implementing gait_generator.cpp
# add_library(gait_generator SHARED src/gait_generator.cpp)
# target_include_directories(gait_generator PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# =============================================================================
# Task 6: Unit tests for gait generator
# =============================================================================
# TODO: Uncomment when implementing test_gait_generator.cpp
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(test_gait_generator test/test_gait_generator.cpp)
#   target_link_libraries(test_gait_generator gait_generator)
# endif()

# =============================================================================
# Task 6: Install targets
# =============================================================================
# TODO: Uncomment when you have library targets
# install(TARGETS gait_generator
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )
#
# install(DIRECTORY include/
#   DESTINATION include
# )

ament_package()
