/**
  Copyright 2016 Lucas Walter

  https://github.com/lucasw/bgfx_ros

  ~/other/bgfx/.build/linux64_gcc/bin/shadercDebug -f fs_cubes.sc -i $HOME/other/bgfx/src -o fs_cubes.bin --varyingdef ./varying.def.sc --platform linux --type fragment
  ~/other/bgfx/.build/linux64_gcc/bin/shadercDebug -f vs_cubes.sc -i $HOME/other/bgfx/src -o vs_cubes.bin --varyingdef ./varying.def.sc --platform linux --type vertex

  g++ bgfx.cpp -g -I$HOME/other/bgfx/include -I$HOME/other/bx/include -L$HOME/other/bgfx/.build/linux64_gcc/bin -lbgfx-shared-libDebug -lGLU -lGL `sdl2-config --cflags --libs`
*/

#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>
#include <bgfx/platform.h>
#include <bgfx/bgfx.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <vector>

int main(int argc, char** argv)
{
  SDL_Window* window = SDL_CreateWindow("bgfx_ros", SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED, 1280, 720,
      SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

	SDL_SysWMinfo wmi;
	SDL_VERSION(&wmi.version);
	if (!SDL_GetWindowWMInfo(window, &wmi) )
	{
		std::cerr << "couldn't get wm info" << std::endl;
		return 1;
	}

	bgfx::PlatformData pd;
	pd.ndt          = wmi.info.x11.display;
	pd.nwh          = (void*)(uintptr_t)wmi.info.x11.window;
	pd.context      = NULL;
	pd.backBuffer   = NULL;
	pd.backBufferDS = NULL;
	bgfx::setPlatformData(pd);

  if (!bgfx::init(bgfx::RendererType::Count, BGFX_PCI_ID_NONE))
  // if (!bgfx::init(bgfx::RendererType::OpenGL, BGFX_PCI_ID_NONE))
  // if (!bgfx::init(bgfx::RendererType::OpenGL, BGFX_PCI_ID_NVIDIA))
  {
    std::cerr << "bgfx init failed" << std::endl;
    return -1;
  }

  bgfx::reset(1280, 720, BGFX_RESET_VSYNC);

  bgfx::setDebug(BGFX_DEBUG_TEXT);
  bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
      0x303030ff, 1.0f, 0);


  bgfx::ShaderHandle fhandle;
  {
    const std::string path = "fs_cubes.bin";
    bgfx::Memory mem;
    // load bin into fmem - need to set fmem.data (uint8_t*) and fmem.size
    std::ifstream ifs(path.c_str(), std::ios::binary | std::ios::ate);
    std::ifstream::pos_type pos = ifs.tellg();
    std::vector<uint8_t> result(pos);
    mem.size = result.size();
    mem.data = &result[0];
    // TODO(lucasw) check if this worked
    fhandle = bgfx::createShader(&mem);
  }

  bgfx::ShaderHandle vhandle;
  {
    const std::string path = "vs_cubes.bin";
    bgfx::Memory mem;
    std::ifstream ifs(path.c_str(), std::ios::binary | std::ios::ate);
    std::ifstream::pos_type pos = ifs.tellg();
    std::vector<uint8_t> result(pos);
    mem.size = result.size();
    mem.data = &result[0];
    vhandle = bgfx::createShader(&mem);
  }

  bgfx::ProgramHandle program;
  // TODO(lucasw) check if this worked
  program = bgfx::createProgram(vhandle, fhandle, true);

  // while (true)
  for (size_t i = 0; i < 5; ++i)
  {
    bgfx::touch(0);

    bgfx::dbgTextClear();
    std::stringstream ss;
    ss << "test " << i;
    bgfx::dbgTextPrintf(20, 20,
        0x8f, ss.str().c_str());
    std::cout << ss.str() << std::endl;

    bgfx::frame();
	  // pause();
    usleep(50000);
  }

  bgfx::shutdown();

  return 0;
}
