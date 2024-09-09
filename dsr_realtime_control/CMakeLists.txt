cmake_minimum_required(VERSION 3.8)
project(dsr_realtime_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dsr_msgs2 REQUIRED)

SET( COMMON_INC_FILES
  ../common2/include
)
SET( COMMON_LIB_FILES
  ../common2/lib
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${COMMON_INC_FILES}
)

LINK_LIBRARIES(DRFL PocoFoundation PocoNet)

if($ENV{ROS_DISTRO} STREQUAL "humble")                          
  LINK_DIRECTORIES ("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_64")
else()
  message("Unknown ROS distro:")
  message($ENV{ROS_DISTRO})
endif()

add_executable(realtime_control src/realtime_control.cpp)
ament_target_dependencies(realtime_control rclcpp dsr_msgs2)

add_executable(rt_api_test src/rt_api_test.cpp)
ament_target_dependencies(rt_api_test rclcpp dsr_msgs2)

add_executable(rt_init src/rt_init.cpp)
ament_target_dependencies(rt_init rclcpp dsr_msgs2)

add_executable(rt_shutdown src/rt_shutdown.cpp)
ament_target_dependencies(rt_shutdown rclcpp dsr_msgs2)

install(TARGETS
  realtime_control
  rt_api_test
  rt_init
  rt_shutdown
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()