# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "week0: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iweek0:/home/heatheramistani/Desktop/hri_assignments_2024/hri2024/src/hri_projects_2024/week0/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(week0_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/heatheramistani/Desktop/hri_assignments_2024/hri2024/src/hri_projects_2024/week0/msg/person.msg" NAME_WE)
add_custom_target(_week0_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "week0" "/home/heatheramistani/Desktop/hri_assignments_2024/hri2024/src/hri_projects_2024/week0/msg/person.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(week0
  "/home/heatheramistani/Desktop/hri_assignments_2024/hri2024/src/hri_projects_2024/week0/msg/person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/week0
)

### Generating Services

### Generating Module File
_generate_module_cpp(week0
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/week0
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(week0_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(week0_generate_messages week0_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/heatheramistani/Desktop/hri_assignments_2024/hri2024/src/hri_projects_2024/week0/msg/person.msg" NAME_WE)
add_dependencies(week0_generate_messages_cpp _week0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(week0_gencpp)
add_dependencies(week0_gencpp week0_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS week0_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(week0
  "/home/heatheramistani/Desktop/hri_assignments_2024/hri2024/src/hri_projects_2024/week0/msg/person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/week0
)

### Generating Services

### Generating Module File
_generate_module_eus(week0
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/week0
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(week0_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(week0_generate_messages week0_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/heatheramistani/Desktop/hri_assignments_2024/hri2024/src/hri_projects_2024/week0/msg/person.msg" NAME_WE)
add_dependencies(week0_generate_messages_eus _week0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(week0_geneus)
add_dependencies(week0_geneus week0_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS week0_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(week0
  "/home/heatheramistani/Desktop/hri_assignments_2024/hri2024/src/hri_projects_2024/week0/msg/person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/week0
)

### Generating Services

### Generating Module File
_generate_module_lisp(week0
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/week0
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(week0_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(week0_generate_messages week0_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/heatheramistani/Desktop/hri_assignments_2024/hri2024/src/hri_projects_2024/week0/msg/person.msg" NAME_WE)
add_dependencies(week0_generate_messages_lisp _week0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(week0_genlisp)
add_dependencies(week0_genlisp week0_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS week0_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(week0
  "/home/heatheramistani/Desktop/hri_assignments_2024/hri2024/src/hri_projects_2024/week0/msg/person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/week0
)

### Generating Services

### Generating Module File
_generate_module_nodejs(week0
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/week0
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(week0_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(week0_generate_messages week0_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/heatheramistani/Desktop/hri_assignments_2024/hri2024/src/hri_projects_2024/week0/msg/person.msg" NAME_WE)
add_dependencies(week0_generate_messages_nodejs _week0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(week0_gennodejs)
add_dependencies(week0_gennodejs week0_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS week0_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(week0
  "/home/heatheramistani/Desktop/hri_assignments_2024/hri2024/src/hri_projects_2024/week0/msg/person.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/week0
)

### Generating Services

### Generating Module File
_generate_module_py(week0
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/week0
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(week0_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(week0_generate_messages week0_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/heatheramistani/Desktop/hri_assignments_2024/hri2024/src/hri_projects_2024/week0/msg/person.msg" NAME_WE)
add_dependencies(week0_generate_messages_py _week0_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(week0_genpy)
add_dependencies(week0_genpy week0_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS week0_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/week0)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/week0
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(week0_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/week0)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/week0
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(week0_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/week0)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/week0
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(week0_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/week0)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/week0
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(week0_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/week0)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/week0\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/week0
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(week0_generate_messages_py std_msgs_generate_messages_py)
endif()
