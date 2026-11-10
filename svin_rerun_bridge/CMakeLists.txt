cmake_minimum_required(VERSION 3.16)

project(svin_rerun_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

# Keep this URL pinned to the official SDK bundle for the API used by this
# package. Do not replace it with the moving "latest" URL.
include(FetchContent)
FetchContent_Declare(
  rerun_sdk
  URL https://github.com/rerun-io/rerun/releases/download/0.40.0/rerun_cpp_sdk.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(rerun_sdk)

add_library(${PROJECT_NAME}_conversions STATIC
  src/conversions.cpp
  src/compressed_image_bundle.cpp
  src/pinhole.cpp
)
target_include_directories(${PROJECT_NAME}_conversions
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_conversions
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
)
target_link_libraries(${PROJECT_NAME}_conversions ${OpenCV_LIBS})
target_compile_options(${PROJECT_NAME}_conversions PRIVATE -Wall -Wextra -Wpedantic)

add_library(${PROJECT_NAME}_core STATIC
  src/svin_rerun_bridge.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
)
target_link_libraries(${PROJECT_NAME}_core
  ${PROJECT_NAME}_conversions
  rerun_sdk
  ${OpenCV_LIBS}
)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(${PROJECT_NAME}_node
  src/node_main.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS
  ${PROJECT_NAME}_conversions
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_conversions
    test/test_conversions.cpp
  )
  target_link_libraries(test_conversions ${PROJECT_NAME}_conversions)
  ament_target_dependencies(test_conversions
    sensor_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    cv_bridge
  )

  ament_add_gtest(test_pinhole
    test/test_pinhole.cpp
  )
  target_link_libraries(test_pinhole ${PROJECT_NAME}_conversions)
  ament_target_dependencies(test_pinhole sensor_msgs)

  ament_add_gtest(test_compressed_image_bundle
    test/test_compressed_image_bundle.cpp
  )
  target_link_libraries(test_compressed_image_bundle ${PROJECT_NAME}_conversions)
  ament_target_dependencies(test_compressed_image_bundle sensor_msgs)
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs cv_bridge)
ament_export_libraries(${PROJECT_NAME}_conversions ${PROJECT_NAME}_core)

ament_package()
