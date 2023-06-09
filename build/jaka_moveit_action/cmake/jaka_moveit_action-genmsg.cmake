# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "jaka_moveit_action: 7 messages, 0 services")

set(MSG_I_FLAGS "-Ijaka_moveit_action:/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(jaka_moveit_action_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg" NAME_WE)
add_custom_target(_jaka_moveit_action_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_moveit_action" "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg" "jaka_moveit_action/jakacontrollerFeedback:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg" NAME_WE)
add_custom_target(_jaka_moveit_action_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_moveit_action" "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg" ""
)

get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg" NAME_WE)
add_custom_target(_jaka_moveit_action_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_moveit_action" "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg" ""
)

get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg" NAME_WE)
add_custom_target(_jaka_moveit_action_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_moveit_action" "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg" ""
)

get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg" NAME_WE)
add_custom_target(_jaka_moveit_action_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_moveit_action" "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg" "jaka_moveit_action/jakacontrollerResult:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerAction.msg" NAME_WE)
add_custom_target(_jaka_moveit_action_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_moveit_action" "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerAction.msg" "jaka_moveit_action/jakacontrollerActionResult:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:jaka_moveit_action/jakacontrollerActionGoal:jaka_moveit_action/jakacontrollerFeedback:jaka_moveit_action/jakacontrollerActionFeedback:jaka_moveit_action/jakacontrollerResult:jaka_moveit_action/jakacontrollerGoal:std_msgs/Header"
)

get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg" NAME_WE)
add_custom_target(_jaka_moveit_action_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_moveit_action" "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg" "jaka_moveit_action/jakacontrollerGoal:actionlib_msgs/GoalID:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_cpp(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_cpp(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_cpp(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_cpp(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_cpp(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerAction.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_cpp(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_moveit_action
)

### Generating Services

### Generating Module File
_generate_module_cpp(jaka_moveit_action
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_moveit_action
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(jaka_moveit_action_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(jaka_moveit_action_generate_messages jaka_moveit_action_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_cpp _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_cpp _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_cpp _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_cpp _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_cpp _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerAction.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_cpp _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_cpp _jaka_moveit_action_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jaka_moveit_action_gencpp)
add_dependencies(jaka_moveit_action_gencpp jaka_moveit_action_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jaka_moveit_action_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_eus(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_eus(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_eus(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_eus(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_eus(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerAction.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_eus(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_moveit_action
)

### Generating Services

### Generating Module File
_generate_module_eus(jaka_moveit_action
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_moveit_action
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(jaka_moveit_action_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(jaka_moveit_action_generate_messages jaka_moveit_action_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_eus _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_eus _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_eus _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_eus _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_eus _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerAction.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_eus _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_eus _jaka_moveit_action_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jaka_moveit_action_geneus)
add_dependencies(jaka_moveit_action_geneus jaka_moveit_action_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jaka_moveit_action_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_lisp(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_lisp(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_lisp(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_lisp(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_lisp(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerAction.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_lisp(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_moveit_action
)

### Generating Services

### Generating Module File
_generate_module_lisp(jaka_moveit_action
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_moveit_action
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(jaka_moveit_action_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(jaka_moveit_action_generate_messages jaka_moveit_action_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_lisp _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_lisp _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_lisp _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_lisp _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_lisp _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerAction.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_lisp _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_lisp _jaka_moveit_action_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jaka_moveit_action_genlisp)
add_dependencies(jaka_moveit_action_genlisp jaka_moveit_action_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jaka_moveit_action_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_nodejs(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_nodejs(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_nodejs(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_nodejs(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_nodejs(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerAction.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_nodejs(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_moveit_action
)

### Generating Services

### Generating Module File
_generate_module_nodejs(jaka_moveit_action
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_moveit_action
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(jaka_moveit_action_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(jaka_moveit_action_generate_messages jaka_moveit_action_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_nodejs _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_nodejs _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_nodejs _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_nodejs _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_nodejs _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerAction.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_nodejs _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_nodejs _jaka_moveit_action_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jaka_moveit_action_gennodejs)
add_dependencies(jaka_moveit_action_gennodejs jaka_moveit_action_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jaka_moveit_action_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_py(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_py(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_py(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_py(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_py(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerAction.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg;/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_moveit_action
)
_generate_msg_py(jaka_moveit_action
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_moveit_action
)

### Generating Services

### Generating Module File
_generate_module_py(jaka_moveit_action
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_moveit_action
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(jaka_moveit_action_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(jaka_moveit_action_generate_messages jaka_moveit_action_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionFeedback.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_py _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerFeedback.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_py _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerResult.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_py _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerGoal.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_py _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionResult.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_py _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerAction.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_py _jaka_moveit_action_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joy/JAKA_WS/devel/share/jaka_moveit_action/msg/jakacontrollerActionGoal.msg" NAME_WE)
add_dependencies(jaka_moveit_action_generate_messages_py _jaka_moveit_action_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jaka_moveit_action_genpy)
add_dependencies(jaka_moveit_action_genpy jaka_moveit_action_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jaka_moveit_action_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_moveit_action)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_moveit_action
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(jaka_moveit_action_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(jaka_moveit_action_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(jaka_moveit_action_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_moveit_action)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_moveit_action
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(jaka_moveit_action_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(jaka_moveit_action_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(jaka_moveit_action_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_moveit_action)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_moveit_action
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(jaka_moveit_action_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(jaka_moveit_action_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(jaka_moveit_action_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_moveit_action)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_moveit_action
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(jaka_moveit_action_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(jaka_moveit_action_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(jaka_moveit_action_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_moveit_action)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_moveit_action\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_moveit_action
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(jaka_moveit_action_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(jaka_moveit_action_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(jaka_moveit_action_generate_messages_py sensor_msgs_generate_messages_py)
endif()
