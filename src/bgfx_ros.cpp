/**
  Copyright 2016 Lucas Walter

  https://github.com/lucasw/bgfx_ros
*/

#include <ros/ros.h>
#include <SDL2/SDL.h>
#include <bgfx/bgfx.h>

// this has to come after SDL.h
#include <bgfx/platform.h>

class BgfxRos
{

  SDL_window* window_;

  uint32_t width_;
  uint32_t height_;
  uint32_t reset_;

public:
  bool initted_;
  bool init()
  {
    window_ = SDL_CreateWindow("bgfx_ros", SDL_WINDOWPOS_UNDEFINED,
				SDL_WINDOWPOS_UNDEFINED, width_, height_,
				SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

    bgfx::sdlSetWindow(window_);

    ROS_INFO_STREAM(width_ << " " << height_ << " " << reset_);
    ROS_INFO_STREAM(bgfx::RendererType::Count << " "
        << BGFX_PCI_ID_NONE);
    if (!bgfx::init(bgfx::RendererType::Count, BGFX_PCI_ID_NONE))
    // if (!bgfx::init(bgfx::RendererType::OpenGL, BGFX_PCI_ID_NONE))
    // if (!bgfx::init(bgfx::RendererType::OpenGL, BGFX_PCI_ID_NVIDIA))
    {
      ROS_ERROR_STREAM("bgfx init failed");
      return false;
    }
    bgfx::reset(width_, height_, reset_);

    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
        0x303030ff, 1.0f, 0);

    return true;
  }

  BgfxRos(uint32_t width = 1280, uint32_t height = 720) :
    width_(width),
    height_(height),
    reset_(BGFX_RESET_VSYNC),
    initted_(false)
  {
    initted_ = init();
  }

  void shutdown()
  {
    if (!initted_) return;

    bgfx::shutdown();
  }

  ~BgfxRos()
  {
    shutdown();
  }

  void update()
  {
    if (!initted_) return;

    bgfx::setViewRect(0, 0, 0, uint16_t(width_), uint16_t(height_));
    bgfx::touch(0);
    bgfx::frame();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bgfx_ros");

  BgfxRos bgfx_ros;
  if (!bgfx_ros.initted_) return -1;

  ros::Rate rate(30);
  while (ros::ok())
  {
    bgfx_ros.update();
    rate.sleep();
  }

  bgfx_ros.shutdown();

  return 0;
}
