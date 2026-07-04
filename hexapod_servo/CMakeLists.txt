cmake_minimum_required(VERSION 3.8)
project(hexapod_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(yaml-cpp REQUIRED)

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
# ROS node wrapper
# =============================================================================
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(servo_node_exec src/servo_node.cpp)
target_include_directories(servo_node_exec PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(servo_node_exec servo_driver yaml-cpp)
ament_target_dependencies(servo_node_exec rclcpp sensor_msgs std_msgs)

add_executable(read_positions_exec src/read_positions.cpp)
target_include_directories(read_positions_exec PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(read_positions_exec yaml-cpp)
ament_target_dependencies(read_positions_exec rclcpp sensor_msgs ament_index_cpp)

# =============================================================================
# Install
# =============================================================================
install(TARGETS servo_driver servo_node_exec read_positions_exec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
