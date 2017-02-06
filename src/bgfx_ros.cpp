/**
  Copyright 2016 Lucas Walter

  https://github.com/lucasw/bgfx_ros
*/

#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>
// this has to come after SDL.h
#include <bgfx/platform.h>
#include <bgfx/bgfx.h>
#include <bx/fpumath.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <iomanip>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
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

void createShaderFromFile(const std::string path,
    std::vector<uint8_t>& result,
    bgfx::ShaderHandle& handle)
{
  std::ifstream ifs(path.c_str(), std::ios::binary | std::ios::in);
  result = std::vector<uint8_t>((std::istreambuf_iterator<char>(ifs)),
      std::istreambuf_iterator<char>());
  // TODO(lucasw) bgfx_util.cpp loadMem appends this to the end, is it
  // needed here?  Each bin already has a trailing 0
  // result.push_back('\0');

  const bgfx::Memory* mem = bgfx::makeRef((uint8_t*)&result[0], result.size());
  ROS_INFO_STREAM(path << " shader size " << mem->size);
  for (size_t i = result.size() - 10; i < result.size(); ++i)
  {
    ROS_DEBUG_STREAM("0x" << std::setfill('0') << std::setw(2)
      << std::hex << static_cast<int>(result[i]) << std::dec);
  }
  // TODO(lucasw) check if this worked
  handle = bgfx::createShader(mem);
}

class BgfxRos
{
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;

  // need two, one to write to and the other to display?
  cv::Mat image_[2];

  SDL_Window* window_;

  uint32_t width_;
  uint32_t height_;
  uint32_t reset_;

  std::vector<PosColorVertex> vertices_;
  std::vector<uint16_t> triangle_list_;

  std::vector<uint8_t> vresult_;
  std::vector<uint8_t> fresult_;
  bgfx::ShaderHandle vhandle_;
  bgfx::ShaderHandle fhandle_;

  bgfx::IndexBufferHandle ibh_;
  bgfx::VertexBufferHandle vbh_;
  bgfx::ProgramHandle program_;

  bgfx::FrameBufferHandle frame_buffer_handle_;
  bgfx::TextureHandle frame_buffer_texture_[2];
  bgfx::TextureHandle read_back_texture_;

  float at_[3];
  float eye_[3];

public:
  bool initted_;
  bool bgfx_initted_;

  bool init()
  {
    pose_sub_ = nh_.subscribe("pose", 5, &BgfxRos::poseCallback, this);
    bgfx_initted_ = bgfxInit();

    image_[0] = cv::Mat(cv::Size(width_, height_), CV_8UC4);
    image_[1] = cv::Mat(cv::Size(width_, height_), CV_8UC4);
    return true;
  }

  bool bgfxInit()
  {
    window_ = SDL_CreateWindow("bgfx_ros", SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED, width_, height_,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

    SDL_SysWMinfo wmi;
    SDL_VERSION(&wmi.version);
    if (!SDL_GetWindowWMInfo(window_, &wmi))
    {
      ROS_ERROR_STREAM("couldn't get wm info");
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
      ROS_ERROR_STREAM("bgfx init failed");
      return false;
    }

    bgfx::reset(width_, height_, BGFX_RESET_VSYNC);

    bgfx::setDebug(BGFX_DEBUG_TEXT);
    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
        0x303030ff, 1.0f, 0);

    ROS_INFO_STREAM(width_ << " " << height_ << " " << reset_);
    ROS_INFO_STREAM(bgfx::RendererType::Count << " "
        << BGFX_PCI_ID_NONE);

    ///////////

    std::string path = ros::package::getPath("bgfx_ros");
    path += "/src/";
    ROS_INFO_STREAM("loading shaders from " << path);

    createShaderFromFile(path + "vs_cubes.bin", vresult_, vhandle_);

    createShaderFromFile(path + "fs_cubes.bin", fresult_, fhandle_);

    const bool destroy_shader = true;
    program_ = bgfx::createProgram(vhandle_, fhandle_, destroy_shader);
    if (!isValid(program_))
    {
      ROS_ERROR_STREAM("creating shader program failed");
      return false;
    }

    ///////////////

    // Make geometry
    PosColorVertex::init();
    int i = 0;
    for (size_t zi = 0; zi < 2; ++zi)
    {
      for (size_t yi = 0; yi < 2; ++yi)
      {
        for (size_t xi = 0; xi < 2; ++xi)
        {
          PosColorVertex pcv;
          pcv.x_ = xi * 2.0 - 1.0;
          pcv.y_ = yi * 2.0 - 1.0;
          pcv.z_ = zi * 2.0 - 1.0;
          pcv.abgr_ = 0xff000000 + ((i * 16) << 16) + (((8 - i) * 31) << 8) + (128 + i * 8);
          ROS_DEBUG_STREAM("0x" << std::setfill('0') << std::setw(8)
              << std::hex << static_cast<long int>(pcv.abgr_) << std::dec);
          vertices_.push_back(pcv);
          ++i;
        }
      }
    }

    triangle_list_.push_back(0);
    triangle_list_.push_back(1);
    triangle_list_.push_back(2);

    triangle_list_.push_back(3);
    triangle_list_.push_back(7);
    triangle_list_.push_back(1);
    triangle_list_.push_back(5);
    triangle_list_.push_back(0);
    triangle_list_.push_back(4);
    triangle_list_.push_back(2);
    triangle_list_.push_back(6);
    triangle_list_.push_back(7);
    triangle_list_.push_back(4);
    triangle_list_.push_back(5);

    {
      const bgfx::Memory* mem = bgfx::makeRef((uint8_t*)&vertices_[0],
          vertices_.size() * sizeof(PosColorVertex));
      vbh_ = bgfx::createVertexBuffer(mem, PosColorVertex::decl_);
    }

    {
      const bgfx::Memory* mem = bgfx::makeRef((uint8_t*)&triangle_list_[0],
          triangle_list_.size() * sizeof(uint16_t));
      ibh_ = bgfx::createIndexBuffer(mem);
    }

    at_[0] = 0.0f;
    at_[1] = 0.0f;
    at_[2] = 0.0f;
    eye_[0] = 0.0f;
    eye_[1] = 0.0f;
    eye_[2] = -35.0f;

    if ((bgfx::getCaps()->supported & (BGFX_CAPS_TEXTURE_BLIT | BGFX_CAPS_TEXTURE_READ_BACK)) !=
        (BGFX_CAPS_TEXTURE_BLIT|BGFX_CAPS_TEXTURE_READ_BACK))
    {
      ROS_ERROR_STREAM("can't read back texture");
      return false;
    }
    const bool has_mips = false;
    const uint16_t num_layers = 1;
    read_back_texture_ = bgfx::createTexture2D(width_, height_,
        has_mips, num_layers,
        bgfx::TextureFormat::BGRA8, BGFX_TEXTURE_READ_BACK | BGFX_TEXTURE_BLIT_DST);
    if (!bgfx::isValid(read_back_texture_))
    {
      ROS_ERROR_STREAM("couldn't create read back texture");
      return false;
    }

    // the regular image texture
    frame_buffer_texture_[0] = bgfx::createTexture2D(width_, height_,
        has_mips, num_layers,
        bgfx::TextureFormat::BGRA8,
        BGFX_TEXTURE_RT | BGFX_TEXTURE_U_CLAMP | BGFX_TEXTURE_V_CLAMP);
    if (!bgfx::isValid(frame_buffer_texture_[0]))
    {
      ROS_ERROR_STREAM("couldn't create read back image texture");
      return false;
    }
    // depth texture- need to provide this?  Not going to use it
    // ... unless for sim kinect node.
    frame_buffer_texture_[1] = bgfx::createTexture2D(width_, height_,
        has_mips, num_layers,
        bgfx::TextureFormat::D16, BGFX_TEXTURE_RT_WRITE_ONLY);
    if (!bgfx::isValid(frame_buffer_texture_[1]))
    {
      ROS_ERROR_STREAM("couldn't create read back depth texture");
      return false;
    }

    frame_buffer_handle_ = bgfx::createFrameBuffer(2, frame_buffer_texture_);
    if (frame_buffer_handle_.idx == bgfx::invalidHandle)
    {
      ROS_ERROR_STREAM("couldn't create fbh");
      return false;
    }

    return true;
  }

  BgfxRos(uint32_t width = 1280, uint32_t height = 720) :
    width_(width),
    height_(height),
    reset_(BGFX_RESET_VSYNC),
    initted_(false),
    bgfx_initted_(false)
  {
    initted_ = init();
  }

  void shutdown()
  {
    if (!initted_) return;

    if (bgfx_initted_)
    {
      bgfx::destroyIndexBuffer(ibh_);
      bgfx::destroyVertexBuffer(vbh_);
      bgfx::destroyProgram(program_);
      bgfx::shutdown();
    }
    initted_ = false;
  }

  ~BgfxRos()
  {
    shutdown();
  }

  void poseCallback(const geometry_msgs::PoseConstPtr& msg)
  {
    eye_[0] = msg->position.x;
    eye_[1] = msg->position.y;
    eye_[2] = msg->position.z;

    at_[0] = msg->position.x;
    at_[1] = msg->position.y;
    at_[2] = msg->position.z + 49.0;

    // ROS_INFO_STREAM(eye_[0] << " " << eye_[1] << " " << eye_[2]);
    // tf::Pose;
  }

  uint32_t i_;

  void update()
  {
    if (!bgfx_initted_) return;

    bgfx::dbgTextClear();
    std::stringstream ss;
    ss << "test " << i_;
    bgfx::dbgTextPrintf(20, 20,
        0x8f, ss.str().c_str());
    ROS_DEBUG_STREAM(ss.str());

    float view[16];
    // ROS_INFO_STREAM(eye_[0] << " " << eye_[1] << " " << eye_[2]);
    bx::mtxLookAt(view, eye_, at_);

    float proj[16];
    bx::mtxProj(proj, 60.0f, float(width_) / float(height_), 0.1f, 100.0f);
    bgfx::setViewTransform(0, view, proj);

    // Set view 0 default viewport.
    bgfx::setViewName(0, "bgfx_ros");
    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
        0x305030ff, 1.0f, 0);
    bgfx::setViewRect(0, 0, 0, uint16_t(width_), uint16_t(height_));
    // TODO(lucasw)
    // It looks like the clear color is appearing on the copied rendered
    // texture, but nothing else is getting rendered- no cubes.
    // Nothing appears on the regular window, but that is okay.
    // If this line is commented out then the cubes re-appear on the
    // standard window.
    bgfx::setViewFrameBuffer(0, frame_buffer_handle_);
    bgfx::touch(0);

    // draw a single cube
    for (uint32_t yy = 0; yy < 11; ++yy)
    {
    for (uint32_t xx = 0; xx < 11; ++xx)
    {
      const float fr = i_ * 0.05;
      float mtx[16];
      bx::mtxRotateXY(mtx, xx*0.21f, yy*0.37f);
      mtx[12] = -15.0f + float(xx)*3.0f;
      mtx[13] = -15.0f + float(yy)*3.0f;
      mtx[14] = 0.0f;

      // Set model matrix for rendering.
      bgfx::setTransform(mtx);

      // Set vertex and index buffer.
      bgfx::setVertexBuffer(vbh_);
      bgfx::setIndexBuffer(ibh_);

      // Set render states.
      bgfx::setState(0
        | BGFX_STATE_DEFAULT
        // | BGFX_STATE_PT_TRILIST    // this doesn't exist - is it the default?
        | BGFX_STATE_PT_TRISTRIP
        // TODO(lwalter) not sure about these
        | BGFX_STATE_RGB_WRITE
        | BGFX_STATE_ALPHA_WRITE
        );

      // Submit primitive for rendering to view 0.
      bgfx::submit(0, program_);
    }  // draw a cube
    }

    // TODO(lucasw) does there need to be a isValid every update?
    bgfx::blit(0, read_back_texture_, 0, 0,
        frame_buffer_texture_[0], 0, 0, width_, height_);
    uint32_t read_frame = bgfx::readTexture(read_back_texture_, image_[i_ % 2].data);

    uint32_t cur_frame = bgfx::frame();
    ROS_INFO_STREAM("read " << read_frame << ", cur " << cur_frame);
    // update sdl processes
    SDL_Delay(1);
    SDL_PollEvent(NULL);
    cv::imshow("image", image_[(i_ + 1) % 2]);
    cv::waitKey(1);
    ++i_;
  }  // update
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bgfx_ros");

  BgfxRos bgfx_ros;
  if (!bgfx_ros.initted_) return -1;

  ros::Rate rate(10);
  while (ros::ok())
  {
    bgfx_ros.update();

    ros::spinOnce();
    rate.sleep();
  }

  bgfx_ros.shutdown();

  return 0;
}
