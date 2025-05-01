# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "okvis_ros: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iokvis_ros:/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(okvis_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/msg/SvinHealth.msg" NAME_WE)
add_custom_target(_okvis_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "okvis_ros" "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/msg/SvinHealth.msg" "geometry_msgs/Point32:std_msgs/Header"
)

get_filename_component(_filename "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/srv/OdometryTrigger.srv" NAME_WE)
add_custom_target(_okvis_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "okvis_ros" "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/srv/OdometryTrigger.srv" "geometry_msgs/Point:geometry_msgs/PoseWithCovariance:geometry_msgs/Quaternion:geometry_msgs/Twist:geometry_msgs/Pose:nav_msgs/Odometry:geometry_msgs/TwistWithCovariance:std_msgs/Header:geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(okvis_ros
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/msg/SvinHealth.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/okvis_ros
)

### Generating Services
_generate_srv_cpp(okvis_ros
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/srv/OdometryTrigger.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/okvis_ros
)

### Generating Module File
_generate_module_cpp(okvis_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/okvis_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(okvis_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(okvis_ros_generate_messages okvis_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/msg/SvinHealth.msg" NAME_WE)
add_dependencies(okvis_ros_generate_messages_cpp _okvis_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/srv/OdometryTrigger.srv" NAME_WE)
add_dependencies(okvis_ros_generate_messages_cpp _okvis_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(okvis_ros_gencpp)
add_dependencies(okvis_ros_gencpp okvis_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS okvis_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(okvis_ros
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/msg/SvinHealth.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/okvis_ros
)

### Generating Services
_generate_srv_eus(okvis_ros
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/srv/OdometryTrigger.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/okvis_ros
)

### Generating Module File
_generate_module_eus(okvis_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/okvis_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(okvis_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(okvis_ros_generate_messages okvis_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/msg/SvinHealth.msg" NAME_WE)
add_dependencies(okvis_ros_generate_messages_eus _okvis_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/srv/OdometryTrigger.srv" NAME_WE)
add_dependencies(okvis_ros_generate_messages_eus _okvis_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(okvis_ros_geneus)
add_dependencies(okvis_ros_geneus okvis_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS okvis_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(okvis_ros
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/msg/SvinHealth.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/okvis_ros
)

### Generating Services
_generate_srv_lisp(okvis_ros
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/srv/OdometryTrigger.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/okvis_ros
)

### Generating Module File
_generate_module_lisp(okvis_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/okvis_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(okvis_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(okvis_ros_generate_messages okvis_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/msg/SvinHealth.msg" NAME_WE)
add_dependencies(okvis_ros_generate_messages_lisp _okvis_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/srv/OdometryTrigger.srv" NAME_WE)
add_dependencies(okvis_ros_generate_messages_lisp _okvis_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(okvis_ros_genlisp)
add_dependencies(okvis_ros_genlisp okvis_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS okvis_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(okvis_ros
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/msg/SvinHealth.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/okvis_ros
)

### Generating Services
_generate_srv_nodejs(okvis_ros
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/srv/OdometryTrigger.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/okvis_ros
)

### Generating Module File
_generate_module_nodejs(okvis_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/okvis_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(okvis_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(okvis_ros_generate_messages okvis_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/msg/SvinHealth.msg" NAME_WE)
add_dependencies(okvis_ros_generate_messages_nodejs _okvis_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/srv/OdometryTrigger.srv" NAME_WE)
add_dependencies(okvis_ros_generate_messages_nodejs _okvis_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(okvis_ros_gennodejs)
add_dependencies(okvis_ros_gennodejs okvis_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS okvis_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(okvis_ros
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/msg/SvinHealth.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/okvis_ros
)

### Generating Services
_generate_srv_py(okvis_ros
  "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/srv/OdometryTrigger.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/okvis_ros
)

### Generating Module File
_generate_module_py(okvis_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/okvis_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(okvis_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(okvis_ros_generate_messages okvis_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/msg/SvinHealth.msg" NAME_WE)
add_dependencies(okvis_ros_generate_messages_py _okvis_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cmb/singularity/svin_ws/src/SVIn/okvis_ros/srv/OdometryTrigger.srv" NAME_WE)
add_dependencies(okvis_ros_generate_messages_py _okvis_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(okvis_ros_genpy)
add_dependencies(okvis_ros_genpy okvis_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS okvis_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/okvis_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/okvis_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(okvis_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(okvis_ros_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(okvis_ros_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/okvis_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/okvis_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(okvis_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(okvis_ros_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(okvis_ros_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/okvis_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/okvis_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(okvis_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(okvis_ros_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(okvis_ros_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/okvis_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/okvis_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(okvis_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(okvis_ros_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(okvis_ros_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/okvis_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/okvis_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/okvis_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(okvis_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(okvis_ros_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(okvis_ros_generate_messages_py nav_msgs_generate_messages_py)
endif()
