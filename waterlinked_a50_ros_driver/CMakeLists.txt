cmake_minimum_required(VERSION 3.5)
project(waterlinked_a50_ros_driver)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DVL.msg"
  "msg/DVLBeam.msg"
  "msg/DVLDR.msg"
  "msg/CommandResponse.msg"
  "msg/ConfigCommand.msg"
  "msg/ConfigStatus.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
 )

ament_package()
