cmake_minimum_required(VERSION 3.8)
project(audix_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EspTelemetry.msg"
  "msg/IrState.msg"
  "srv/Move.srv"
  "srv/RawCommand.srv"
  "srv/TwistCommand.srv"
  "srv/DirectionCommand.srv"
  "srv/RotateCommand.srv"
  "srv/SetRobotMode.srv"
  "srv/LiftMoveSteps.srv"
  "srv/AuditMission.srv"
  "srv/ShelfScan.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
