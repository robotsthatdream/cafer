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
        topic_tools
        )
find_package(Boost COMPONENTS python filesystem REQUIRED)

find_package(rostest REQUIRED)
find_package(PythonLibs REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_COMPILER /usr/local/bin/clang++)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

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
        DBManager.msg
        Management.msg
        manager_test.msg
)

## Generate services in the 'srv' folder
add_service_files(
        FILES
        GetID.srv
)

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
        INCLUDE_DIRS include
        LIBRARIES cafer_core
        CATKIN_DEPENDS roscpp std_msgs message_runtime roslib topic_tools
        DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${rostest_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS}
)
#link_directories($ENV{HOME}/git/libfastsim/build/src /usr/lib/i386-linux-gnu)

add_executable(getid src/getid.cpp)
target_link_libraries(getid ${catkin_LIBRARIES})
add_dependencies(getid cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_library(cafer_core src/component.cpp src/db_manager/db_manager.cpp src/db_manager/_db_filesystem.cpp src/db_manager/_wave.cpp src/db_manager/_write_worker.cpp)
target_link_libraries(cafer_core tbb ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

add_dependencies(cafer_core cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

# Python wrapper SHARED library that wraps the CPP component
add_library(component SHARED src/component.cpp)
target_link_libraries(component tbb ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(component cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
set_target_properties(component PROPERTIES PREFIX "")

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# # 

# Basic tests on the CAFER component
add_executable(component_test_node test/component_test_node.cpp)
target_link_libraries(component_test_node cafer_core ${catkin_LIBRARIES})
add_dependencies(component_test_node cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(component_test_chgt_freq_node test/component_test_chgt_freq_node.cpp)
target_link_libraries(component_test_chgt_freq_node ${catkin_LIBRARIES})
add_dependencies(component_test_chgt_freq_node cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(component_test_death_node test/component_test_death_node.cpp)
target_link_libraries(component_test_death_node ${catkin_LIBRARIES})
add_dependencies(component_test_death_node cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(manager_test_subscribe_node test/manager_test_subscribe_node.cpp)
target_link_libraries(manager_test_subscribe_node ${catkin_LIBRARIES})
add_dependencies(manager_test_subscribe_node cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

#add_executable(manager_test_sub test/manager_test_subscribe.cpp)
#target_link_libraries(manager_test_sub cafer_core ${catkin_LIBRARIES})
#add_dependencies(manager_test_sub cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(manager_component_test_node test/manager_component_test.cpp)
target_link_libraries(manager_component_test_node cafer_core ${catkin_LIBRARIES})
add_dependencies(manager_component_test_node cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(component_test test_launch/component_test.launch test/component_test.cpp)
    target_link_libraries(component_test cafer_core ${catkin_LIBRARIES})

    add_rostest_gtest(component_test_death test_launch/component_test_death.launch test/component_test_death.cpp)
    target_link_libraries(component_test_death cafer_core ${catkin_LIBRARIES})

    #add_rostest(test_launch/component_test_frequency10.launch)

    #add_rostest(test_launch/component_test_frequency20.launch)

    add_rostest_gtest(component_test_launch_death test_launch/component_test_launch_death.launch test/component_test_launch_death.cpp)
    target_link_libraries(component_test_launch_death cafer_core ${catkin_LIBRARIES})

    add_rostest_gtest(component_test_launch_death_tbb test_launch/component_test_launch_death_tbb.launch test/component_test_launch_death_tbb.cpp)
    target_link_libraries(component_test_launch_death_tbb cafer_core tbb ${catkin_LIBRARIES})

    add_rostest_gtest(manager_test test_launch/manager_test.launch test/manager_test.cpp)
    target_link_libraries(manager_test cafer_core tbb ${catkin_LIBRARIES})

    #  add_rostest_gtest(manager_test_subscribe test_launch/manager_test_subscribe.launch test/manager_test.cpp)
    #  target_link_libraries(manager_test_subscribe cafer_core tbb ${catkin_LIBRARIES})

    add_rostest_gtest(manager_component_test test_launch/manager_component_test.launch test/manager_test.cpp)
    target_link_libraries(manager_component_test cafer_core tbb ${catkin_LIBRARIES})

endif ()

#Examples
add_executable(data_publisher_example example/data_publisher_example.cpp)
target_link_libraries(data_publisher_example ${catkin_LIBRARIES})
add_dependencies(data_publisher_example cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(data_manager_example example/data_manager_example.cpp)
target_link_libraries(data_manager_example cafer_core ${catkin_LIBRARIES})
add_dependencies(data_manager_example cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(managermap_example example/managermap_example.cpp)
target_link_libraries(managermap_example cafer_core ${catkin_LIBRARIES})
add_dependencies(managermap_example cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(managerqueue_example example/managerqueue_example.cpp)
target_link_libraries(managerqueue_example cafer_core ${catkin_LIBRARIES})
add_dependencies(managerqueue_example cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(basic_example_change_frequency example/basic_example_change_frequency.cpp)
target_link_libraries(basic_example_change_frequency cafer_core ${catkin_LIBRARIES})
add_dependencies(basic_example_change_frequency cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(basic_example_kill_node_topic_type example/basic_example_kill_node_topic_type.cpp)
target_link_libraries(basic_example_kill_node_topic_type cafer_core ${catkin_LIBRARIES})
add_dependencies(basic_example_kill_node_topic_type cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(basic_example_launch_set_nodes example/basic_example_launch_set_nodes.cpp)
target_link_libraries(basic_example_launch_set_nodes cafer_core ${catkin_LIBRARIES})
add_dependencies(basic_example_launch_set_nodes cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(basic_example_new_node example/basic_example_new_node.cpp)
target_link_libraries(basic_example_new_node cafer_core ${catkin_LIBRARIES})
add_dependencies(basic_example_new_node cafer_core_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
