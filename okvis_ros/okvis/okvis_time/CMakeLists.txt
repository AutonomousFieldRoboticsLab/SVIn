cmake_minimum_required(VERSION 3.8.11)

project(okvis_time)

# build the library
add_library(${PROJECT_NAME}
  src/Time.cpp
  src/Duration.cpp)

# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets 
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")
