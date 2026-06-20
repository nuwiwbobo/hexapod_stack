cmake_minimum_required(VERSION 3.8)
project(hexapod_gait)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# =============================================================================
# Wave Gait Generator library (pure math, no ROS dependency)
# =============================================================================
add_library(gait_generator SHARED src/gait_generator.cpp)
target_include_directories(gait_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(gait_generator PUBLIC cxx_std_17)

# =============================================================================
# Unit tests for gait generator
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gait_generator test/test_gait_generator.cpp)
  target_link_libraries(test_gait_generator gait_generator)
  target_include_directories(test_gait_generator PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

# =============================================================================
# Install targets
# =============================================================================
install(TARGETS gait_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
