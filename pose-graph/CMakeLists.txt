cmake_minimum_required(VERSION 2.8.3)
project(pose_graph)
include(ExternalProject)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -msse4.1")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    cv_bridge
    roslib
    okvis_ros  # For SvinHealth msg
    )

find_package(OpenCV)

# brisk
find_package( brisk 2 REQUIRED )
include_directories(BEFORE ${BRISK_INCLUDE_DIRS})
message(STATUS "Using system brisk. Found at ${BRISK_INCLUDE_DIRS}.")

find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories( include ${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} ${BRISK_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS okvis_ros)

add_executable(pose_graph
    src/pose_graph_node.cpp
    src/LoopClosing.cpp
    src/KFMatcher.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
    )
add_dependencies(${PROJECT_NAME} Dependencies brisk)
target_link_libraries(pose_graph ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${CERES_LIBRARIES} ${BRISK_LIBRARIES}) 
#message("catkin_lib  ${catkin_LIBRARIES}")
