/**
  Copyright 2016 Lucas Walter

  https://github.com/lucasw/bgfx_ros
*/

#include <ros/ros.h>
#include <bgfx/bgfx.h>

class BgfxRos
{
  public:

  uint32_t width_;
  uint32_t height_;
  uint32_t reset_;

  void init()
  {
    bgfx::init();
    bgfx::reset(width_, height_, reset_);
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bgfx_ros");

  while (ros::ok())
  {

  }

  return 0;
}
