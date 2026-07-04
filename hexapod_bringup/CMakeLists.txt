cmake_minimum_required(VERSION 3.8)
project(hexapod_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(hexapod_ik REQUIRED)

find_library(IK_SOLVER_LIB ik_solver PATHS ${CMAKE_SOURCE_DIR}/../hexapod_ik)

add_executable(gait_test_node src/gait_test_node.cpp)
target_include_directories(gait_test_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../hexapod_ik/include>
)
target_link_libraries(gait_test_node ${IK_SOLVER_LIB} yaml-cpp)
ament_target_dependencies(gait_test_node rclcpp sensor_msgs)

add_executable(test_ik_exec src/test_ik.cpp)
target_include_directories(test_ik_exec PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../hexapod_ik/include>
)
target_link_libraries(test_ik_exec ${IK_SOLVER_LIB} yaml-cpp)
ament_target_dependencies(test_ik_exec rclcpp sensor_msgs geometry_msgs ament_index_cpp)

add_executable(test_ik_stance_exec src/test_ik_stance.cpp)
target_include_directories(test_ik_stance_exec PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../hexapod_ik/include>
)
target_link_libraries(test_ik_stance_exec ${IK_SOLVER_LIB} yaml-cpp)
ament_target_dependencies(test_ik_stance_exec rclcpp sensor_msgs ament_index_cpp)

install(TARGETS gait_test_node test_ik_exec test_ik_stance_exec
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
