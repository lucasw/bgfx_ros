/**
  Copyright 2016 Lucas Walter

  https://github.com/lucasw/bgfx_ros
*/

#include <ros/ros.h>
#include <bgfx/bgfx.h>

class BgfxRos
{

  uint32_t width_;
  uint32_t height_;
  uint32_t reset_;

public:
  void init()
  {
    bgfx::init(bgfx::RendererType::Count, BGFX_PCI_ID_NONE);
    bgfx::reset(width_, height_, reset_);

    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
        0x303030ff, 1.0f, 0);
  }

  BgfxRos(uint32_t width = 1280, uint32_t height = 720) :
    width_(width),
    height_(height),
    reset_(BGFX_RESET_VSYNC)
  {
    init();
  }

  void shutdown()
  {
    bgfx::shutdown();
  }

  ~BgfxRos()
  {
    shutdown();
  }

  void update()
  {
    bgfx::setViewRect(0, 0, 0, uint16_t(width_), uint16_t(height_));
    bgfx::touch(0);
    bgfx::frame();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bgfx_ros");

  BgfxRos bgfx_ros;

  ros::Rate rate(30);
  while (ros::ok())
  {
    bgfx_ros.update();
    rate.sleep();
  }

  bgfx_ros.shutdown();

  return 0;
}
