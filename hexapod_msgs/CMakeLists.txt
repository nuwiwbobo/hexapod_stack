cmake_minimum_required(VERSION 3.8)
project(hexapod_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# =============================================================================
# Task 4: Custom message definitions
# =============================================================================
# These messages are used by the IK, gait, and control packages.
#
# Message files to generate:
#   - RPY.msg          (roll, pitch, yaw)
#   - Pose.msg         (position + orientation)
#   - LegJoints.msg    (coxa, femur, tibia angles)
#   - LegsJoints.msg   (array of 6 LegJoints)
#   - FeetPositions.msg (array of 6 Poses)
#
# Run: colcon build --packages-select hexapod_msgs

set(msg_files
  "msg/RPY.msg"
  "msg/Pose.msg"
  "msg/LegJoints.msg"
  "msg/LegsJoints.msg"
  "msg/FeetPositions.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs
)

ament_package()
