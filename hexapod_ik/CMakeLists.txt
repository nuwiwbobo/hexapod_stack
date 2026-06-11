cmake_minimum_required(VERSION 3.8)
project(hexapod_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(hexapod_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(ik_solver SHARED src/ik_solver.cpp)
target_include_directories(ik_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ik_solver rclcpp hexapod_msgs sensor_msgs geometry_msgs)

add_library(ik_node SHARED src/ik_node.cpp)
target_include_directories(ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ik_node rclcpp rclcpp_lifecycle rclcpp_components hexapod_msgs sensor_msgs geometry_msgs)
target_link_libraries(ik_node ik_solver)

rclcpp_components_register_node(ik_node
  PLUGIN "hexapod_ik::IkNode"
  EXECUTABLE ik_node_exec
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ik test/test_ik.cpp)
  target_link_libraries(test_ik ik_solver)
endif()

install(TARGETS ik_solver ik_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
