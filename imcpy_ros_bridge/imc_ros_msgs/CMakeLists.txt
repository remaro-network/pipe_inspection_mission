cmake_minimum_required(VERSION 3.8)
project(imc_ros_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Custom messages and services
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowSingleReference.action"
  "msg/Goto.msg"  
  "msg/PlanControl.msg"
  "msg/PlanControlState.msg"
  "msg/VehicleState.msg"
  "msg/EstimatedState.msg"
  "msg/RemoteState.msg"
  "msg/SonarData.msg"
  "msg/DesiredHeading.msg"
  "msg/DesiredHeadingRate.msg"
  "msg/DesiredPitch.msg"
  "msg/DesiredRoll.msg"
  "msg/DesiredSpeed.msg"
  "msg/DesiredZ.msg"
  "msg/PlanDB.msg"
  "msg/PlanSpecification.msg"
  "msg/PlanDBInformation.msg"
  "msg/PlanDBState.msg"
  "msg/PlanManeuver.msg"
  "msg/Maneuver.msg"
  "msg/PolygonVertex.msg"
  "msg/FollowRefState.msg"
  "msg/FollowReference.msg"
  "msg/Reference.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
