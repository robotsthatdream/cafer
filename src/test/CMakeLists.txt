cmake_minimum_required(VERSION 2.8.3)
project(cafer_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cafer_client
)

find_package(Boost REQUIRED)
find_package(rostest REQUIRED)
#find_package(cafer_client REQUIRED)


catkin_package(
  #INCLUDE_DIRS include
  # LIBRARIES fastsim
  CATKIN_DEPENDS roscpp std_msgs cafer_client
  DEPENDS system_lib
)
 
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${cafer_client_INCLUDE_DIRS}
)

add_executable(cafer_client_test_node src/cafer_client_test_node.cpp)
add_dependencies(cafer_client_test_node cafer_client_hpp)
target_link_libraries(cafer_client_test_node cafer_client ${catkin_LIBRARIES})

add_executable(cafer_client_test_node_send_chgt_freq src/cafer_client_test_node_send_chgt_freq.cpp)
target_link_libraries(cafer_client_test_node_send_chgt_freq ${catkin_LIBRARIES})

add_executable(cafer_client_test_node_send_death src/cafer_client_test_node_send_death.cpp)
target_link_libraries(cafer_client_test_node_send_death ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(cafer_client_test launch/cafer_client_test.launch src/cafer_client_test.cpp)
  target_link_libraries(cafer_client_test cafer_client ${catkin_LIBRARIES})

  add_rostest(launch/cafer_client_test_frequency10.launch)

  add_rostest(launch/cafer_client_test_frequency20.launch)

  add_rostest_gtest(cafer_client_test_node_death launch/cafer_client_test_node_death.launch src/cafer_client_test_node_death.cpp)
  target_link_libraries(cafer_client_test_node_death cafer_client ${catkin_LIBRARIES})
endif()
