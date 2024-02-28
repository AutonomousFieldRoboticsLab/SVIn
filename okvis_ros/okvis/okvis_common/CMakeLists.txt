cmake_minimum_required(VERSION 3.8.11)

project(okvis_common)


# require Eigen
find_package( Eigen REQUIRED )
find_package(OpenCV REQUIRED)

include_directories(${EIGEN_INCLUDE_DIR}) 
include_directories(${OpenCV_INCLUDE_DIRS})

# build the library
add_library(${PROJECT_NAME} STATIC 
  src/VioInterface.cpp
  src/VioParametersReader.cpp
  include/okvis/FrameTypedefs.hpp
  include/okvis/Measurements.hpp
  include/okvis/Parameters.hpp
  include/okvis/Variables.hpp
  include/okvis/VioBackendInterface.hpp
  include/okvis/VioFrontendInterface.hpp
  include/okvis/VioInterface.hpp
  include/okvis/VioParametersReader.hpp)

# and link it                     
target_link_libraries(${PROJECT_NAME}
  PUBLIC okvis_util
  PUBLIC okvis_kinematics 
  PUBLIC okvis_time 
  PUBLIC okvis_cv
  ${OpenCV_LIBS}
)

# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets 
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")
