cmake_minimum_required(VERSION 2.8.3)
project(oculus_gazebo_navigator)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_ros
  geometry_msgs
  sensor_msgs
  nodelet
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES oculus_gazebo_navigator
 CATKIN_DEPENDS gazebo_plugins gazebo_ros gazebo_ros geometry_msgs nodelet roscpp rospy std_msgs tf sensor_msgs
)

# find and add gazebo
include (FindPkgConfig)
include(FindProtobuf)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

# protobuf is required for implementing Gazebo topics
find_package(protobuf)
find_package(Protobuf REQUIRED)
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)

# set the default path for built executables to the "bin" directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# set the default path for built libraries to the "lib" directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)


add_executable(oculus_keyboard_controller src/controllers/oculus_keyboard_teleop.cpp)
target_link_libraries(oculus_keyboard_controller ${catkin_LIBRARIES})

add_library(oculus_keyboard_plugin src/plugins/oculus_keyboard_teleop_plugin.cc)
target_link_libraries(oculus_keyboard_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PROTOBUF_LIBRARY})

add_library(oculus_ps3_plugin src/plugins/oculus_ps3_teleop_plugin.cc)
target_link_libraries(oculus_ps3_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PROTOBUF_LIBRARY})