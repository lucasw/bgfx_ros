/**
  Copyright 2016 Lucas Walter

  https://github.com/lucasw/bgfx_ros

  ~/other/bgfx/.build/linux64_gcc/bin/shadercDebug -f fs_cubes.sc -i $HOME/other/bgfx/src -o fs_cubes.bin --varyingdef ./varying.def.sc --platform linux --type fragment
  ~/other/bgfx/.build/linux64_gcc/bin/shadercDebug -f vs_cubes.sc -i $HOME/other/bgfx/src -o vs_cubes.bin --varyingdef ./varying.def.sc --platform linux --type vertex

  g++ bgfx.cpp -g -I$HOME/other/bgfx/include -I$HOME/other/bx/include -L$HOME/other/bgfx/.build/linux64_gcc/bin -lbgfx-shared-libDebug -lGLU -lGL `sdl2-config --cflags --libs` -std=c++11
*/

#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>
#include <bgfx/platform.h>
#include <bgfx/bgfx.h>
#include <bx/fpumath.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <vector>

struct PosColorVertex
{
  float x_;
  float y_;
  float z_;
  uint32_t abgr_;

  static void init()
  {
    decl_
      .begin()
      .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
      .add(bgfx::Attrib::Color0,   4, bgfx::AttribType::Uint8, true)
      .end();
  };

  static bgfx::VertexDecl decl_;
};

bgfx::VertexDecl PosColorVertex::decl_;

int main(int argc, char** argv)
{
	uint32_t width = 1280;
	uint32_t height = 720;

  SDL_Window* window = SDL_CreateWindow("bgfx_ros", SDL_WINDOWPOS_UNDEFINED,
      SDL_WINDOWPOS_UNDEFINED, width, height,
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

  bgfx::reset(width, height, BGFX_RESET_VSYNC);

  bgfx::setDebug(BGFX_DEBUG_TEXT);
  bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
      0x303030ff, 1.0f, 0);

  PosColorVertex::init();
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

  // Make geometry
  std::vector<PosColorVertex> vertices;
  for (size_t xi = 0; xi < 2; ++xi)
  {
    for (size_t yi = 0; yi < 2; ++yi)
    {
      for (size_t zi = 0; zi < 2; ++zi)
      {
        PosColorVertex pcv;
        pcv.x_ = xi * 2.0 - 1.0;
        pcv.y_ = yi * 2.0 - 1.0;
        pcv.z_ = zi * 2.0 - 1.0;
        pcv.abgr_ = 0xff000000 + xi * 0xff0000 + yi * 0xff00 + zi * 0xff;
        vertices.push_back(pcv);
      }
    }
  }

  std::vector<uint16_t> triangle_list;
  triangle_list.push_back(0);
  triangle_list.push_back(1);
  triangle_list.push_back(2);

  triangle_list.push_back(1);
  triangle_list.push_back(3);
  triangle_list.push_back(2);

  triangle_list.push_back(4);
  triangle_list.push_back(6);
  triangle_list.push_back(5);

  triangle_list.push_back(0);
  triangle_list.push_back(2);
  triangle_list.push_back(4);

  std::cout << "########## vertex ##########" << std::endl;
  bgfx::VertexBufferHandle vbh;
  {
    bgfx::Memory mem;
    mem.data = (uint8_t*)(&vertices[0]);
    mem.size = vertices.size();
    vbh = bgfx::createVertexBuffer(&mem, PosColorVertex::decl_);
  }

  bgfx::IndexBufferHandle ibh;
  {
    bgfx::Memory mem;
    mem.data = (uint8_t*)&triangle_list[0];
    mem.size = triangle_list.size();
    ibh = bgfx::createIndexBuffer(&mem);
  }

	float at[3]  = { 0.0f, 0.0f,   0.0f };
	float eye[3] = { 0.0f, 0.0f, -35.0f };

  std::cout << "#########################" << std::endl;
  // while (true)
  for (size_t i = 0; i < 5; ++i)
  {
    bgfx::dbgTextClear();
    std::stringstream ss;
    ss << "test " << i;
    bgfx::dbgTextPrintf(20, 20,
        0x8f, ss.str().c_str());
    std::cout << ss.str() << std::endl;

		float view[16];
		bx::mtxLookAt(view, eye, at);

		float proj[16];
		bx::mtxProj(proj, 60.0f, float(width) / float(height), 0.1f, 100.0f);
		bgfx::setViewTransform(0, view, proj);

		// Set view 0 default viewport.
		bgfx::setViewRect(0, 0, 0, uint16_t(width), uint16_t(height) );

		bgfx::touch(0);

		// draw a single cube
		{
      const float fr = i * 0.01;
			uint32_t xx = 0.0;
      uint32_t yy = 0.0;
			float mtx[16];
			bx::mtxRotateXY(mtx, fr + xx*0.21f, fr + yy*0.37f);
			mtx[12] = -15.0f + float(xx)*3.0f;
			mtx[13] = -15.0f + float(yy)*3.0f;
			mtx[14] = 0.0f;

			// Set model matrix for rendering.
			bgfx::setTransform(mtx);

			// Set vertex and index buffer.
			bgfx::setVertexBuffer(vbh);
			bgfx::setIndexBuffer(ibh);

			// Set render states.
			bgfx::setState(0
				| BGFX_STATE_DEFAULT
				// | BGFX_STATE_PT_TRILIST    // this doesn't exist
				// | BGFX_STATE_PT_TRISTRIP    // this doesn't exist
				);

			// Submit primitive for rendering to view 0.
			bgfx::submit(0, program);
		}

    bgfx::frame();
	  // pause();
    usleep(50000);
  }

  bgfx::destroyIndexBuffer(ibh);
  bgfx::destroyVertexBuffer(vbh);
  bgfx::shutdown();

  return 0;
}
