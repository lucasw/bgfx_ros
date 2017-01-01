cmake_minimum_required(VERSION 2.8.3)
project(bgfx_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rospy
)

roslint_cpp()
roslint_python()

find_package(OpenGL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp roslint rospy message_runtime
)

include_directories(
  $ENV{HOME}/other/bgfx/include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${OPENGL_INCLUDE_DIR}
  $ENV{HOME}/other/bgfx/.build/linux64_gcc/bin
)

add_executable(bgfx_ros src/bgfx_ros.cpp)

add_dependencies(bgfx_ros bgfx_ros_gencpp)

target_link_libraries(bgfx_ros
  ${catkin_LIBRARIES}
  # opengl has to come after bgfx
  bgfx-shared-libRelease
  ${OPENGL_LIBRARIES}
)
