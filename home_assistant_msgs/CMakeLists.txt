cmake_minimum_required(VERSION 3.5)
project(home_assistant_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/mobile_robot/MobileRobotState.msg"
  "msg/mobile_robot/MobileRobotFleetState.msg"
  "msg/mobile_robot/Mission.msg"
  "msg/mobile_robot/Location.msg"
  "msg/mobile_robot/Path.msg"
  "msg/mobile_robot/Images.msg"
  "msg/mobile_robot/OperatingMode.msg"
  "msg/mobile_robot/Payload.msg"
  "msg/mobile_robot/Task.msg"
  "msg/mobile_robot/TaskParameter.msg"
  "msg/mobile_robot/MobileRobotProblem.msg"
  "msg/traffic_light/ClassicTrafficLightState.msg"
)

set(action_files
  "action/MobileRobotMission.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES unique_identifier_msgs std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
