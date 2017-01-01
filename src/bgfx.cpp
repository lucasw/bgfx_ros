/**
  Copyright 2016 Lucas Walter

  https://github.com/lucasw/bgfx_ros

  g++ bgfx.cpp -I$HOME/other/bgfx/include -L$HOME/other/bgfx/.build/linux64_gcc/bin -lbgfx-shared-libRelease -lGLU -lGL
*/

#include <bgfx/bgfx.h>
#include <iostream>

int main(int argc, char** argv)
{
  if (!bgfx::init(bgfx::RendererType::Count, BGFX_PCI_ID_NONE))
  // if (!bgfx::init(bgfx::RendererType::OpenGL, BGFX_PCI_ID_NONE))
  // if (!bgfx::init(bgfx::RendererType::OpenGL, BGFX_PCI_ID_NVIDIA))
  {
    std::cerr << "bgfx init failed" << std::endl;
    return -1;
  }

  bgfx::reset(1280, 720, BGFX_RESET_VSYNC);

  bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
      0x303030ff, 1.0f, 0);

  bgfx::shutdown();

  return 0;
}
