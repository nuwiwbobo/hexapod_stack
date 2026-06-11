cmake_minimum_required(VERSION 3.8)
project(hexapod_gait)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(hexapod_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(gait_engine SHARED src/gait_engine.cpp)
target_include_directories(gait_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gait_engine rclcpp hexapod_msgs geometry_msgs)

add_library(gait_node SHARED src/gait_node.cpp)
target_include_directories(gait_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gait_node rclcpp rclcpp_lifecycle rclcpp_components hexapod_msgs geometry_msgs)
target_link_libraries(gait_node gait_engine)

rclcpp_components_register_node(gait_node
  PLUGIN "hexapod_gait::GaitNode"
  EXECUTABLE gait_node_exec
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gait test/test_gait.cpp)
  target_link_libraries(test_gait gait_engine)
endif()

install(TARGETS gait_engine gait_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
