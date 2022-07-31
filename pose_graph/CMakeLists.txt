cmake_minimum_required(VERSION 2.8.3)
project(pose_graph)
include(ExternalProject)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

# changed to -march=native. Working in Ubuntu 18.04 and 20.04
set(CMAKE_CXX_FLAGS "-march=native -fPIC -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    cv_bridge
    roslib
    okvis_ros  # For SvinHealth msg
    pcl_ros
    pcl_conversions
    )

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(OpenCV)
find_package(PCL  REQUIRED)
find_package(Boost COMPONENTS filesystem system REQUIRED)
find_package(opengv REQUIRED)
find_package(Glog REQUIRED)

# brisk
find_package( brisk 2 REQUIRED )
include_directories(BEFORE ${BRISK_INCLUDE_DIRS})
message(STATUS "Using system brisk. Found at ${BRISK_INCLUDE_DIRS}.")

find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(BEFORE  ${GLOG_INCLUDE_DIRS})
include_directories(include 
                    ThirdParty
                    ${catkin_INCLUDE_DIRS}
                    ${CERES_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${BRISK_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS okvis_ros pcl_ros pcl_conversions)

add_library(${PROJECT_NAME} 
    src/pose_graph/GlobalMapping.cpp
    src/pose_graph/KFMatcher.cpp
    src/pose_graph/LoopClosing.cpp
    src/pose_graph/Parameters.cpp
    src/pose_graph/PoseGraphOptimization.cpp
    src/pose_graph/Publisher.cpp
    src/pose_graph/Subscriber.cpp
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

add_dependencies(${PROJECT_NAME} Dependencies brisk)
target_link_libraries(${PROJECT_NAME}
  opengv
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES} 
  ${BRISK_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}) 

add_executable(${PROJECT_NAME}_node src/pose_graph_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
