# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "custom_msgs: 8 messages, 1 services")

set(MSG_I_FLAGS "-Icustom_msgs:/home/papa/Project/rostest/src/custom_msgs/msg;-Icustom_msgs:/home/papa/Project/rostest/devel/share/custom_msgs/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(custom_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionResult.msg" NAME_WE)
add_custom_target(_custom_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "custom_msgs" "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionResult.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:std_msgs/Header:custom_msgs/navigationResult"
)

get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationAction.msg" NAME_WE)
add_custom_target(_custom_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "custom_msgs" "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationAction.msg" "custom_msgs/navigationActionGoal:custom_msgs/navigationFeedback:actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:custom_msgs/navigationResult:custom_msgs/navigationGoal:std_msgs/Header:custom_msgs/navigationActionFeedback:custom_msgs/navigationActionResult"
)

get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionGoal.msg" NAME_WE)
add_custom_target(_custom_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "custom_msgs" "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionGoal.msg" "actionlib_msgs/GoalID:std_msgs/Header:custom_msgs/navigationGoal"
)

get_filename_component(_filename "/home/papa/Project/rostest/src/custom_msgs/msg/Pos.msg" NAME_WE)
add_custom_target(_custom_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "custom_msgs" "/home/papa/Project/rostest/src/custom_msgs/msg/Pos.msg" ""
)

get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionFeedback.msg" NAME_WE)
add_custom_target(_custom_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "custom_msgs" "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionFeedback.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:custom_msgs/navigationFeedback:std_msgs/Header"
)

get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationResult.msg" NAME_WE)
add_custom_target(_custom_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "custom_msgs" "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationResult.msg" ""
)

get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationGoal.msg" NAME_WE)
add_custom_target(_custom_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "custom_msgs" "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationGoal.msg" ""
)

get_filename_component(_filename "/home/papa/Project/rostest/src/custom_msgs/srv/euler.srv" NAME_WE)
add_custom_target(_custom_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "custom_msgs" "/home/papa/Project/rostest/src/custom_msgs/srv/euler.srv" ""
)

get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationFeedback.msg" NAME_WE)
add_custom_target(_custom_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "custom_msgs" "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationFeedback.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msgs
)
_generate_msg_cpp(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationAction.msg"
  "${MSG_I_FLAGS}"
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionGoal.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationResult.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationGoal.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionFeedback.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msgs
)
_generate_msg_cpp(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msgs
)
_generate_msg_cpp(custom_msgs
  "/home/papa/Project/rostest/src/custom_msgs/msg/Pos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msgs
)
_generate_msg_cpp(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msgs
)
_generate_msg_cpp(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msgs
)
_generate_msg_cpp(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msgs
)
_generate_msg_cpp(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msgs
)

### Generating Services
_generate_srv_cpp(custom_msgs
  "/home/papa/Project/rostest/src/custom_msgs/srv/euler.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msgs
)

### Generating Module File
_generate_module_cpp(custom_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(custom_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(custom_msgs_generate_messages custom_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionResult.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_cpp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationAction.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_cpp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionGoal.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_cpp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/src/custom_msgs/msg/Pos.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_cpp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionFeedback.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_cpp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationResult.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_cpp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationGoal.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_cpp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/src/custom_msgs/srv/euler.srv" NAME_WE)
add_dependencies(custom_msgs_generate_messages_cpp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationFeedback.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_cpp _custom_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(custom_msgs_gencpp)
add_dependencies(custom_msgs_gencpp custom_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS custom_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msgs
)
_generate_msg_lisp(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationAction.msg"
  "${MSG_I_FLAGS}"
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionGoal.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationResult.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationGoal.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionFeedback.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msgs
)
_generate_msg_lisp(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msgs
)
_generate_msg_lisp(custom_msgs
  "/home/papa/Project/rostest/src/custom_msgs/msg/Pos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msgs
)
_generate_msg_lisp(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msgs
)
_generate_msg_lisp(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msgs
)
_generate_msg_lisp(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msgs
)
_generate_msg_lisp(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msgs
)

### Generating Services
_generate_srv_lisp(custom_msgs
  "/home/papa/Project/rostest/src/custom_msgs/srv/euler.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msgs
)

### Generating Module File
_generate_module_lisp(custom_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(custom_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(custom_msgs_generate_messages custom_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionResult.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_lisp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationAction.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_lisp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionGoal.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_lisp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/src/custom_msgs/msg/Pos.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_lisp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionFeedback.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_lisp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationResult.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_lisp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationGoal.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_lisp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/src/custom_msgs/srv/euler.srv" NAME_WE)
add_dependencies(custom_msgs_generate_messages_lisp _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationFeedback.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_lisp _custom_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(custom_msgs_genlisp)
add_dependencies(custom_msgs_genlisp custom_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS custom_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msgs
)
_generate_msg_py(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationAction.msg"
  "${MSG_I_FLAGS}"
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionGoal.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationResult.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationGoal.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionFeedback.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msgs
)
_generate_msg_py(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msgs
)
_generate_msg_py(custom_msgs
  "/home/papa/Project/rostest/src/custom_msgs/msg/Pos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msgs
)
_generate_msg_py(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationFeedback.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msgs
)
_generate_msg_py(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msgs
)
_generate_msg_py(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msgs
)
_generate_msg_py(custom_msgs
  "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msgs
)

### Generating Services
_generate_srv_py(custom_msgs
  "/home/papa/Project/rostest/src/custom_msgs/srv/euler.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msgs
)

### Generating Module File
_generate_module_py(custom_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(custom_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(custom_msgs_generate_messages custom_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionResult.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_py _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationAction.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_py _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionGoal.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_py _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/src/custom_msgs/msg/Pos.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_py _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationActionFeedback.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_py _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationResult.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_py _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationGoal.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_py _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/src/custom_msgs/srv/euler.srv" NAME_WE)
add_dependencies(custom_msgs_generate_messages_py _custom_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/papa/Project/rostest/devel/share/custom_msgs/msg/navigationFeedback.msg" NAME_WE)
add_dependencies(custom_msgs_generate_messages_py _custom_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(custom_msgs_genpy)
add_dependencies(custom_msgs_genpy custom_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS custom_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(custom_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(custom_msgs_generate_messages_cpp actionlib_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(custom_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(custom_msgs_generate_messages_lisp actionlib_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(custom_msgs_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(custom_msgs_generate_messages_py actionlib_msgs_generate_messages_py)
