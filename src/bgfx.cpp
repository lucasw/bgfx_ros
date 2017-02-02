/**
  Copyright 2016 Lucas Walter

  https://github.com/lucasw/bgfx_ros


  g++ bgfx.cpp -g -I$HOME/other/bgfx/include -I$HOME/other/bx/include -L$HOME/other/bgfx/.build/linux64_gcc/bin -lbgfx-shared-libDebug -lGLU -lGL `sdl2-config --cflags --libs`
*/

#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>
#include <bgfx/platform.h>
#include <bgfx/bgfx.h>
#include <iostream>
#include <unistd.h>

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

  bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
      0x303030ff, 1.0f, 0);

	pause();

  bgfx::shutdown();

  return 0;
}
