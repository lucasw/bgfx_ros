cmake_minimum_required(VERSION 2.8.3)
project(bgfx_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  roslint
  rospy
)

roslint_cpp()
roslint_python()

# http://stackoverflow.com/questions/23850472/how-to-use-sdl2-and-sdl-image-with-cmake
# find_package(SDL2 REQUIRED)
INCLUDE(FindPkgConfig)

PKG_SEARCH_MODULE(SDL2 REQUIRED sdl2)
# PKG_SEARCH_MODULE(SDL2IMAGE REQUIRED SDL2_image>=2.0.0)

add_definitions(-std=c++11)
####

find_package(OpenGL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp roslint rospy message_runtime
)

include_directories(
  $ENV{HOME}/other/bgfx/include
  $ENV{HOME}/other/bx/include
	${SDL2_INCLUDE_DIRS} # ${SDL2IMAGE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${OPENGL_INCLUDE_DIR}
  $ENV{HOME}/other/bgfx/.build/linux64_gcc/bin
)

add_executable(bgfx_ros src/bgfx_ros.cpp)

# add_dependencies(bgfx_ros bgfx_ros_gencpp)

target_link_libraries(bgfx_ros
  ${catkin_LIBRARIES}
  # opengl has to come after bgfx
  # bgfx-shared-libRelease
  bgfx-shared-libDebug
	${SDL2_LIBRARIES}  # ${SDL2IMAGE_LIBRARIES}
  ${OPENGL_LIBRARIES}
)
