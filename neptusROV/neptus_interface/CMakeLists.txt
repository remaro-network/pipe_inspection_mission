cmake_minimum_required(VERSION 3.8)
project(neptus_interface)

set(AIRSIM_ROOT $ENV{HOME}/dev/UNav-Sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
    ${AIRSIM_ROOT}/AirLib/deps/eigen3
    ${AIRSIM_ROOT}/AirLib/deps/rpclib/include
    ${AIRSIM_ROOT}/AirLib/include
    ${AIRSIM_ROOT}/MavLinkCom/include
    ${AIRSIM_ROOT}/MavLinkCom/common_utils
    ${RPC_LIB_INCLUDES}
)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

add_executable(neptus2unavsim src/neptus_pose_to_unavsim_pose.cpp)
ament_target_dependencies(neptus2unavsim rclcpp geometry_msgs)

target_link_libraries(neptus2unavsim
  pthread
  AirLib
  rpc
  ${rclcpp_LIBRARIES}
)


install(TARGETS
  neptus2unavsim
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
