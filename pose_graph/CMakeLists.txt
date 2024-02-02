cmake_minimum_required(VERSION 3.8.3)
project(pose_graph)
include(ExternalProject)


if(CMAKE_BUILD_TYPE)
    message(STATUS "Build type specified as ${CMAKE_BUILD_TYPE}")
else()
    message(STATUS "Build type not specified, using Release")
    set(CMAKE_BUILD_TYPE Release)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 17)

# changed to -march=native. Working in Ubuntu 18.04 and 20.04
set(CMAKE_CXX_FLAGS "-march=native -O3 -fPIC -Wall")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(okvis_ros REQUIRED)
find_package(std_srvs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package( brisk 2 REQUIRED )
find_package(Ceres 2.1 REQUIRED)
include_directories(BEFORE ${BRISK_INCLUDE_DIRS})
message(STATUS "Using system brisk. Found at ${BRISK_INCLUDE_DIRS}.")

include_directories(BEFORE  ${GLOG_INCLUDE_DIRS})
include_directories(include 
                    ThirdParty
                    ${catkin_INCLUDE_DIRS}
                    ${CERES_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${BRISK_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

list(APPEND ament_libraries 
      rclcpp
      pcl_ros 
      cv_bridge 
      sensor_msgs 
      std_msgs 
      nav_msgs 
      pcl_conversions
      visualization_msgs
      std_srvs
      okvis_ros
      )

add_library(${PROJECT_NAME}_lib 
    src/pose_graph/GlobalMapping.cpp
    src/pose_graph/Keyframe.cpp
    src/pose_graph/LoopClosure.cpp
    src/pose_graph/Parameters.cpp
    src/pose_graph/PoseGraph.cpp
    src/pose_graph/Publisher.cpp
    src/pose_graph/Subscriber.cpp
    src/pose_graph/SwitchingEstimator.cpp
    src/utils/CameraPoseVisualization.cpp
    src/utils/UtilsOpenCV.cpp
    src/utils/Utils.cpp
    src/utils/Statistics.cpp
    # src/utils/LoopClosureUtils.cpp
    ThirdParty/DBoW/BowVector.cpp
    ThirdParty/DBoW/FBrief.cpp
    ThirdParty/DBoW/FeatureVector.cpp
    ThirdParty/DBoW/QueryResults.cpp
    ThirdParty/DBoW/ScoringObject.cpp
    ThirdParty/DUtils/Random.cpp
    ThirdParty/DUtils/Timestamp.cpp
    ThirdParty/DVision/BRIEF.cpp
    ThirdParty/VocabularyBinary.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES} 
  ${BRISK_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}_lib ${ament_libraries})
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

add_executable(${PROJECT_NAME}_node src/pose_graph_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${ament_libraries})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
