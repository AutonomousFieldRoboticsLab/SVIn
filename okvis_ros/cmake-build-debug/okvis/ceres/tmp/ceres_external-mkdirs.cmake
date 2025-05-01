# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/cmake-build-debug/okvis/ceres/src/ceres_external")
  file(MAKE_DIRECTORY "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/cmake-build-debug/okvis/ceres/src/ceres_external")
endif()
file(MAKE_DIRECTORY
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/cmake-build-debug/okvis/ceres/src/ceres_external-build"
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/cmake-build-debug"
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/cmake-build-debug/okvis/ceres/tmp"
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/cmake-build-debug/okvis/ceres/src/ceres_external-stamp"
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/cmake-build-debug/okvis/ceres/src"
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/cmake-build-debug/okvis/ceres/src/ceres_external-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/cmake-build-debug/okvis/ceres/src/ceres_external-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/cmake-build-debug/okvis/ceres/src/ceres_external-stamp${cfgdir}") # cfgdir has leading slash
endif()
