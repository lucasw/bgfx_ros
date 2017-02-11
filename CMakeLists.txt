cmake_minimum_required(VERSION 2.8.3)
project(bgfx_ros)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  roslib
  roslint
  rospy
  tf
  visualization_msgs
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

set(BGFX_DIR $ENV{HOME}/other/bgfx CACHE PATH "location of bgfx directory")
set(BX_DIR $ENV{HOME}/other/bx CACHE PATH "location of bx directory")
# Build shaders
#  ~/other/bgfx/.build/linux64_gcc/bin/shadercDebug -f fs_cubes.sc -i $HOME/other/bgfx/src -o fs_cubes.bin --varyingdef ./varying.def.sc --platform linux -p 120 --type fragment
add_custom_command(OUTPUT fs_cubes.bin PRE_LINK
  COMMAND ${BGFX_DIR}/.build/linux64_gcc/bin/shadercRelease -f ${CMAKE_CURRENT_SOURCE_DIR}/src/fs_cubes.sc -i ${BGFX_DIR}/src -o ${CMAKE_CURRENT_SOURCE_DIR}/src/fs_cubes.bin --varyingdef ${CMAKE_CURRENT_SOURCE_DIR}/src/varying.def.sc --platform linux -p 120 --type fragment
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/varying.def.sc ${CMAKE_CURRENT_SOURCE_DIR}/src/fs_cubes.sc
  COMMENT "Fragment Shader"
  )
#  ~/other/bgfx/.build/linux64_gcc/bin/shadercDebug -f vs_cubes.sc -i $HOME/other/bgfx/src -o vs_cubes.bin --varyingdef ./varying.def.sc --platform linux -p 120 --type vertex
add_custom_command(OUTPUT vs_cubes.bin PRE_LINK
  COMMAND ${BGFX_DIR}/.build/linux64_gcc/bin/shadercRelease -f ${CMAKE_CURRENT_LIST_DIR}/src/vs_cubes.sc -i ${BGFX_DIR}/src -o ${CMAKE_CURRENT_SOURCE_DIR}/src/vs_cubes.bin --varyingdef ${CMAKE_CURRENT_LIST_DIR}/src/varying.def.sc --platform linux -p 120 --type vertex
  DEPENDS ${CMAKE_CURRENT_LIST_DIR}/src/varying.def.sc ${CMAKE_CURRENT_LIST_DIR}/src/vs_cubes.sc
  COMMENT "Vertex Shader"
  )

include_directories(
  ${BGFX_DIR}/include
  ${BX_DIR}/include
	${SDL2_INCLUDE_DIRS} # ${SDL2IMAGE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${OPENGL_INCLUDE_DIR}
  ${BGFX_DIR}/.build/linux64_gcc/bin
)

add_executable(bgfx_ros src/bgfx_ros.cpp fs_cubes.bin vs_cubes.bin)

find_library(bx_release libbxRelease.a ${BGFX_DIR}/.build/linux64_gcc/bin)
target_link_libraries(bgfx_ros
  ${catkin_LIBRARIES}
  ${bx_release}
  # opengl has to come after bgfx
  bgfx-shared-libRelease
	${SDL2_LIBRARIES}  # ${SDL2IMAGE_LIBRARIES}
  ${OPENGL_LIBRARIES}
)

add_executable(bgfx_ros_debug src/bgfx_ros.cpp fs_cubes.bin vs_cubes.bin)

# add_dependencies(bgfx_ros bgfx_ros_gencpp)

find_library(bx_debug libbxDebug.a ${BGFX_DIR}/.build/linux64_gcc/bin)
target_link_libraries(bgfx_ros_debug
  ${catkin_LIBRARIES}
  ${bx_debug}
  # opengl has to come after bgfx
  bgfx-shared-libDebug
	${SDL2_LIBRARIES}  # ${SDL2IMAGE_LIBRARIES}
  ${OPENGL_LIBRARIES}
)

###

install(FILES src/fs_cubes.bin src/vs_cubes.bin
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)
install(TARGETS bgfx_ros bgfx_ros_debug
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
