cmake_minimum_required(VERSION 2.8.3)
project(rosjack)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-narrowing")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
   FILES
   Audio.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

INCLUDE(FindPkgConfig)
pkg_check_modules (JACK jack)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${JACK_INCLUDE_DIRS}
)

add_executable(talkerjack src/talkerjack.cpp)
add_executable(listenerjack src/listenerjack.cpp)

add_dependencies(talker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(talker ${JACK_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(listener ${JACK_LIBRARIES} ${catkin_LIBRARIES})


