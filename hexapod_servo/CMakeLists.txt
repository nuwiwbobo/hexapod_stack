cmake_minimum_required(VERSION 3.8)
project(hexapod_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# =============================================================================
# ServoDriver library (unit conversion + hardware control)
# =============================================================================
add_library(servo_driver SHARED src/servo_driver.cpp)
target_include_directories(servo_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(servo_driver PUBLIC cxx_std_17)
ament_target_dependencies(servo_driver dynamixel_sdk)

# =============================================================================
# Unit tests (pure conversion — no hardware needed)
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_servo_driver test/test_servo_driver.cpp)
  target_link_libraries(test_servo_driver servo_driver)
  target_include_directories(test_servo_driver PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

# =============================================================================
# Task 3: ROS node wrapper
# =============================================================================
# TODO: Uncomment when implementing servo_node.cpp
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
#
# add_library(servo_node SHARED src/servo_node.cpp)
# target_include_directories(servo_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(servo_node rclcpp rclcpp_components sensor_msgs std_msgs)
# target_link_libraries(servo_node servo_driver)
#
# rclcpp_components_register_node(servo_node
#   PLUGIN "hexapod_servo::ServoNode"
#   EXECUTABLE servo_node_exec
# )

# =============================================================================
# Install
# =============================================================================
install(TARGETS servo_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
