cmake_minimum_required(VERSION 2.8.3)
project(pose_graph)
include(ExternalProject)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")

# wanted to use march=native which causes segfault. We need to debug later.
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -msse4.1")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    cv_bridge
    roslib
    okvis_ros  # For SvinHealth msg
    pcl_ros
    pcl_conversions
    image_transport
    )

find_package(OpenCV)
find_package(PCL  REQUIRED)

# brisk
find_package( brisk 2 REQUIRED )
include_directories(BEFORE ${BRISK_INCLUDE_DIRS})
message(STATUS "Using system brisk. Found at ${BRISK_INCLUDE_DIRS}.")

find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(include 
                    ThirdParty
                    ${catkin_INCLUDE_DIRS}
                    ${CERES_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${BRISK_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS okvis_ros pcl_ros pcl_conversions)

add_library(${PROJECT_NAME} 
    src/pose_graph/GlobalMapping.cpp
    src/pose_graph/KFMatcher.cpp
    src/pose_graph/LoopClosing.cpp
    src/pose_graph/Parameters.cpp
    src/pose_graph/PoseGraphOptimization.cpp
    src/pose_graph/Publisher.cpp
    src/pose_graph/Subscriber.cpp
    src/utility/CameraPoseVisualization.cpp
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
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${CERES_LIBRARIES} ${BRISK_LIBRARIES} ${PCL_LIBRARIES}) 

add_executable(${PROJECT_NAME}_node src/pose_graph_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
