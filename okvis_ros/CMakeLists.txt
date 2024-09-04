cmake_minimum_required(VERSION 3.8.3)
project(okvis_ros)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

find_package(OpenCV REQUIRED)

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
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
# Sharmin: for SvinHealth msg
rosidl_generate_interfaces(${PROJECT_NAME} "msg/SvinHealth.msg" "srv/OdometryTrigger.srv" DEPENDENCIES std_msgs geometry_msgs nav_msgs)

# It need to be either set to Release or RelwithDebInfo here
set(VALID_BUILD_TYPES "Release" "RelWithDebInfo")
if (CMAKE_BUILD_TYPE)
    message(STATUS "Build type specified as ${CMAKE_BUILD_TYPE}")
    if (NOT ${CMAKE_BUILD_TYPE} IN_LIST VALID_BUILD_TYPES)
        message(FATAL_ERROR "Build type should either be Release or RelWithDebInfo")
    endif ()
else ()
    message(STATUS "Build type not specified, using Release")
    set(CMAKE_BUILD_TYPE Release)
endif ()

message(STATUS "Building with ${CMAKE_BUILD_TYPE}")

# # The configure step fails at catkin_package() if this directory
# # doesn't exist yet. Create it at configure time!
# file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/include)

# let's pull in the okvis plain cmake project
set(BUILD_SHARED_LIBS OFF)
add_subdirectory(okvis)


add_definitions(-fPIC -march=native -O3 -Wextra -Winit-self -Woverloaded-virtual
        -Wnon-virtual-dtor -Wsign-promo -Wno-long-long -std=c++20)

list(APPEND ament_libraries
        rclcpp
        rosidl_default_generators
        pcl_ros
        cv_bridge
        sensor_msgs
        geometry_msgs
        tf2
        tf2_ros
        tf2_sensor_msgs
        tf2_geometry_msgs
        std_msgs
        nav_msgs
        std_srvs
        visualization_msgs
        image_transport
        pcl_conversions
)

ament_export_dependencies(rosidl_default_runtime)


add_library(${PROJECT_NAME}_lib STATIC src/Subscriber.cpp
        src/Publisher.cpp
        src/RosParametersReader.cpp
        include/okvis/Subscriber.hpp
        include/okvis/Publisher.hpp
        include/okvis/RosParametersReader.hpp)
add_dependencies(${PROJECT_NAME}_lib okvis_multisensor_processing)
ament_target_dependencies(${PROJECT_NAME}_lib ${ament_libraries})
rosidl_target_interfaces(${PROJECT_NAME}_lib ${PROJECT_NAME} rosidl_typesupport_cpp)


target_include_directories(${PROJECT_NAME}_lib PUBLIC
        include
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

target_link_libraries(${PROJECT_NAME}_lib
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

install(TARGETS ${PROJECT_NAME}_lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        PUBLIC_HEADER DESTINATION include
)
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)


# now the actual applications
add_executable(okvis_node src/okvis_node.cpp)
ament_target_dependencies(okvis_node ${ament_libraries})
target_link_libraries(okvis_node ${PROJECT_NAME}_lib)
rosidl_target_interfaces(okvis_node ${PROJECT_NAME} rosidl_typesupport_cpp)
install(TARGETS okvis_node DESTINATION lib/${PROJECT_NAME})

# add_executable(okvis_node_synchronous src/okvis_node_synchronous.cpp)
# ament_target_dependencies(okvis_node_synchronous ${ament_libraries})
# target_link_libraries(okvis_node_synchronous ${PROJECT_NAME}_lib )
# rosidl_target_interfaces(okvis_node_synchronous ${PROJECT_NAME} rosidl_typesupport_cpp) 
# install(TARGETS okvis_node_synchronous DESTINATION lib/${PROJECT_NAME})

# add_executable(dataset_convertor src/dataset_convertor.cpp)
# target_link_libraries(dataset_convertor ${PROJECT_NAME}_lib )
# install(TARGETS dataset_convertor DESTINATION lib/${PROJECT_NAME})

add_executable(stereo_sync src/stereo_sync.cpp)
ament_target_dependencies(stereo_sync ${ament_libraries})
target_include_directories(stereo_sync PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stereo_sync ${OpenCV_LIBRARIES})
install(TARGETS stereo_sync DESTINATION lib/${PROJECT_NAME})

add_executable(uncompress_image src/uncompress_image.cpp)
ament_target_dependencies(uncompress_image ${ament_libraries})
install(TARGETS uncompress_image DESTINATION lib/${PROJECT_NAME})


#All the necessary files in relative path needs to be copied to the install directory
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY ../config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY ../rviz_config/ DESTINATION share/${PROJECT_NAME}/rviz_config/)

ament_package()
