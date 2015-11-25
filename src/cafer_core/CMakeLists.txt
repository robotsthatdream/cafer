cmake_minimum_required(VERSION 2.8.3)
project(cafer_core)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  roslib
  tf
)
find_package(Boost REQUIRED)
find_package(rostest REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Management.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   GetID.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

#Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  # LIBRARIES fastsim
  CATKIN_DEPENDS roscpp std_msgs roslib
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} 
)
#link_directories($ENV{HOME}/git/libfastsim/build/src /usr/lib/i386-linux-gnu)

add_executable(getid src/getid.cpp)
target_link_libraries(getid ${catkin_LIBRARIES})

add_library(cafer_core src/component.cpp)
target_link_libraries(cafer_core tbb ${catkin_LIBRARIES})

install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".cpp" EXCLUDE
)
# # 

# Basic tests on the CAFER component
add_executable(component_test_node test/component_test_node.cpp)
target_link_libraries(component_test_node cafer_core ${catkin_LIBRARIES})

add_executable(component_test_chgt_freq_node test/component_test_chgt_freq_node.cpp)
target_link_libraries(component_test_chgt_freq_node ${catkin_LIBRARIES})

add_executable(component_test_death_node test/component_test_death_node.cpp)
target_link_libraries(component_test_death_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(component_test test_launch/component_test.launch test/component_test.cpp)
  target_link_libraries(component_test cafer_core ${catkin_LIBRARIES})

  add_rostest(test_launch/component_test_frequency10.launch)

  add_rostest(test_launch/component_test_frequency20.launch)

  add_rostest_gtest(component_test_death test_launch/component_test_death.launch test/component_test_death.cpp)
  target_link_libraries(component_test_death cafer_core ${catkin_LIBRARIES})

  add_rostest_gtest(component_test_launch_death test_launch/component_test_launch_death.launch test/component_test_launch_death.cpp)
  target_link_libraries(component_test_launch_death cafer_core ${catkin_LIBRARIES})

  add_rostest_gtest(component_test_launch_death_tbb test_launch/component_test_launch_death_tbb.launch test/component_test_launch_death_tbb.cpp)
  target_link_libraries(component_test_launch_death_tbb cafer_core tbb ${catkin_LIBRARIES})
endif()


# ## Install project namespaced headers
# install(DIRECTORY src/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE)