cmake_minimum_required(VERSION 3.8)
project(hexapod_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(servo_driver SHARED src/servo_driver.cpp)
target_include_directories(servo_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(servo_driver rclcpp sensor_msgs dynamixel_sdk)

add_library(servo_node SHARED src/servo_node.cpp)
target_include_directories(servo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(servo_node rclcpp rclcpp_lifecycle rclcpp_components sensor_msgs std_msgs yaml-cpp)
target_link_libraries(servo_node servo_driver)

rclcpp_components_register_node(servo_node
  PLUGIN "hexapod_servo::ServoNode"
  EXECUTABLE servo_node_exec
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_servo test/test_servo.cpp)
  target_link_libraries(test_servo servo_driver)
endif()

install(TARGETS servo_driver servo_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
