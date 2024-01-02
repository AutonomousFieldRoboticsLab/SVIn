cmake_minimum_required(VERSION 3.8.3)
project(okvis_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)


# Sharmin: for SvinHealth msg
rosidl_generate_interfaces(${PROJECT_NAME} "msg/SvinHealth.msg" "srv/OdometryTrigger.srv" DEPENDENCIES std_msgs geometry_msgs nav_msgs)
ament_export_dependencies(rosidl_default_runtime)

# we really want to use Release here
set(CMAKE_BUILD_TYPE Release)

message(STATUS "Building with ${CMAKE_BUILD_TYPE}")

# # The configure step fails at catkin_package() if this directory
# # doesn't exist yet. Create it at configure time!
# file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/include)

# let's pull in the okvis plain cmake project
set(BUILD_SHARED_LIBS OFF)
add_subdirectory(okvis)


# compile the ROS dependent stuff into a library

#removed -g flag which produces debubugging information 
#-g is not needed for release builds

add_definitions(-fPIC -march=native -O3 -Wextra -Winit-self -Woverloaded-virtual
                -Wnon-virtual-dtor -Wsign-promo -Wno-long-long -std=c++17)

add_library(${PROJECT_NAME}_lib STATIC src/Subscriber.cpp
                               src/Publisher.cpp
                               src/RosParametersReader.cpp
                               include/okvis/Subscriber.hpp
                               include/okvis/Publisher.hpp
                               include/okvis/RosParametersReader.hpp)

add_dependencies(${PROJECT_NAME}_lib okvis_multisensor_processing)



target_link_libraries(${PROJECT_NAME}_lib 
  ${catkin_LIBRARIES}
  okvis_util
  okvis_kinematics
  okvis_time
  okvis_cv 
  okvis_common
  okvis_ceres
  okvis_timing
  okvis_matcher
  okvis_frontend 
  okvis_multisensor_processing
  pthread
)


include_directories(
  okvis/okvis_util/include
  okvis/okvis_kinematics/include
  okvis/okvis_time/include
  okvis/okvis_cv/include
  okvis/okvis_common/include
  okvis/okvis_ceres/include
  okvis/okvis_timing/include
  okvis/okvis_matcher/include
  okvis/okvis_frontend/include 
  okvis/okvis_multisensor_processing/include
)

# now the actual applications
add_executable(okvis_node src/okvis_node.cpp )
target_link_libraries(okvis_node ${PROJECT_NAME}_lib )
add_executable(okvis_node_synchronous src/okvis_node_synchronous.cpp)
target_link_libraries(okvis_node_synchronous ${PROJECT_NAME}_lib )
add_executable(dataset_convertor src/dataset_convertor.cpp)
target_link_libraries(dataset_convertor ${PROJECT_NAME}_lib )

add_executable(stereo_sync src/stereo_sync.cpp)
target_link_libraries(stereo_sync ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
