/**
  Copyright 2016 Lucas Walter

  https://github.com/lucasw/bgfx_ros
*/

// TODO(lucasw) this cause problems when lower in the include list
#include <tf/transform_listener.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>
// this has to come after SDL.h
#include <bgfx/platform.h>
#include <bgfx/bgfx.h>
#include <bx/fpumath.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sstream>
#include <string>
#include <tf/tf.h>
#include <unistd.h>
#include <vector>
#include <visualization_msgs/Marker.h>


struct PosNormalColorVertex
{
  float x_;
  float y_;
  float z_;
  float nx_;
  float ny_;
  float nz_;
  uint32_t abgr_;

  static void init()
  {
    // TODO(lucasw) I think this has to match varying.def.sc
    decl_
      .begin()
      .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
      .add(bgfx::Attrib::Normal, 3, bgfx::AttribType::Float)
      .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
      .end();
  };

  static bgfx::VertexDecl decl_;
};

bgfx::VertexDecl PosNormalColorVertex::decl_;

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

  const bgfx::Memory* mem = bgfx::makeRef(
      reinterpret_cast<uint8_t*>(&result[0]), result.size());
  ROS_INFO_STREAM(path << " shader size " << mem->size);
  for (size_t i = result.size() - 10; i < result.size(); ++i)
  {
    ROS_DEBUG_STREAM("0x" << std::setfill('0') << std::setw(2)
      << std::hex << static_cast<int>(result[i]) << std::dec);
  }
  // TODO(lucasw) check if this worked
  handle = bgfx::createShader(mem);
}


class Mesh
{
public:
  Mesh(const visualization_msgs::MarkerConstPtr& msg) :
    marker_(msg)
  {
    const uint8_t b = marker_->color.b * 255;
    const uint8_t g = marker_->color.g * 255;
    const uint8_t r = marker_->color.r * 255;
    const uint8_t a = marker_->color.a * 255;
    uint32_t abgr = (a << 24) + (b << 16) + (g << 8) + (r);

    if (marker_->type == visualization_msgs::Marker::CUBE)
    {
      // construct a cube
      for (size_t i = 0; i < 2; ++i)
      {
        for (size_t j = 0; j < 2; ++j)
        {
          for (size_t k = 0; k < 2; ++k)
          {
            const float x = (static_cast<float>(k) - 0.5) * marker_->scale.x;
            const float y = (static_cast<float>(j) - 0.5) * marker_->scale.y;
            const float z = (static_cast<float>(i) - 0.5) * marker_->scale.z;

            // TODO(lucasw) need to transform each point by the provided pose and scale
            PosNormalColorVertex pcv;
            pcv.x_ = x;
            pcv.y_ = y;
            pcv.z_ = z;
            tf::Vector3 normal(x, y, z);
            normal.normalize();
            pcv.nx_ = normal.getX();
            pcv.ny_ = normal.getY();
            pcv.nz_ = normal.getZ();
            pcv.abgr_ = abgr;
            vertices_.push_back(pcv);
          }
        }
      }

      /*  
          y
          |
          |__x
          \
           \
            z

          2-----3
          |\    |\
          | 6-----7
          | |   | |
          | |   | |
          0-|---1 |
           \|    \|
            4-----5
      */

      // top face
      triangle_list_.push_back(7);
      triangle_list_.push_back(3);
      triangle_list_.push_back(6);

      triangle_list_.push_back(6);
      triangle_list_.push_back(3);
      triangle_list_.push_back(2);

      // bottom face
      triangle_list_.push_back(1);
      triangle_list_.push_back(5);
      triangle_list_.push_back(0);

      triangle_list_.push_back(0);
      triangle_list_.push_back(5);
      triangle_list_.push_back(4);

      // back face
      triangle_list_.push_back(0);
      triangle_list_.push_back(2);
      triangle_list_.push_back(1);

      triangle_list_.push_back(1);
      triangle_list_.push_back(2);
      triangle_list_.push_back(3);

      // left face
      triangle_list_.push_back(4);
      triangle_list_.push_back(6);
      triangle_list_.push_back(0);

      triangle_list_.push_back(0);
      triangle_list_.push_back(6);
      triangle_list_.push_back(2);

      // front face
      triangle_list_.push_back(5);
      triangle_list_.push_back(7);
      triangle_list_.push_back(4);

      triangle_list_.push_back(4);
      triangle_list_.push_back(7);
      triangle_list_.push_back(6);

      // right face
      triangle_list_.push_back(1);
      triangle_list_.push_back(3);
      triangle_list_.push_back(5);

      triangle_list_.push_back(5);
      triangle_list_.push_back(3);
      triangle_list_.push_back(7);

    }
    else if (marker_->type == visualization_msgs::Marker::TRIANGLE_LIST)
    {
      // construct a vertex and index buffer from the message
      for (size_t i = 0; i < msg->points.size(); ++i)
      {
        // Make geometry
        // TODO(lucasw) need to transform each point by the provided pose and scale
        PosNormalColorVertex pcv;
        pcv.x_ = msg->points[i].x;
        pcv.y_ = msg->points[i].y;
        pcv.z_ = msg->points[i].z;
        // TODO(lucasw) a new message type for bgfx_ros will have normals provided in it
        // so they won't have to be calculated in this node, but it is required for Markers
        tf::Vector3 normal(pcv.x_, pcv.y_, pcv.z_);
        normal.normalize();
        pcv.nx_ = normal.getX();
        pcv.ny_ = normal.getY();
        pcv.nz_ = normal.getZ();
        if (i < msg->colors.size())
        {
          const uint8_t b = msg->colors[i].b * 255;
          const uint8_t g = msg->colors[i].g * 255;
          const uint8_t r = msg->colors[i].r * 255;
          const uint8_t a = msg->colors[i].a * 255;

          pcv.abgr_ = (a << 24) + (b << 16) + (g << 8) + (r);
        }
        else
        {
          pcv.abgr_ = (0xff << 24) + (50 << 16) + (10 << 8) + (200);
        }
        // ROS_DEBUG_STREAM("0x" << std::setfill('0') << std::setw(8)
        //    << std::hex << static_cast<int32_t>(pcv.abgr_) << std::dec);
        vertices_.push_back(pcv);
        // The Marker message doesn't re-use any triangles
        triangle_list_.push_back(i);
      }
    }

    {
      const bgfx::Memory* mem = bgfx::makeRef(reinterpret_cast<uint8_t*>(&vertices_[0]),
          vertices_.size() * sizeof(PosNormalColorVertex));
      vbh_ = bgfx::createVertexBuffer(mem, PosNormalColorVertex::decl_);
    }

    {
      const bgfx::Memory* mem = bgfx::makeRef(
          reinterpret_cast<uint8_t*>(&triangle_list_[0]),
          triangle_list_.size() * sizeof(uint16_t));
      ibh_ = bgfx::createIndexBuffer(mem);
    }
  }

  ~Mesh()
  {
    bgfx::destroyIndexBuffer(ibh_);
    bgfx::destroyVertexBuffer(vbh_);
  }
  // TODO triangle list or strip mode

  const visualization_msgs::MarkerConstPtr marker_;

  std::vector<PosNormalColorVertex> vertices_;
  bgfx::VertexBufferHandle vbh_;

  std::vector<uint16_t> triangle_list_;
  bgfx::IndexBufferHandle ibh_;
};

class BgfxRos
{
  ros::NodeHandle nh_;
  sensor_msgs::ChannelFloat32 pre_mat_;
  ros::Subscriber pre_mat_sub_;
  ros::Subscriber marker_sub_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher cam_pub_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  // TODO(lucasw) in the future will want any number of cameras,
  // for now just a single one.
  std::string frame_id_;

  tf::TransformListener listener_;

  // need two, one to write to and the other to display?
  std::vector<cv::Mat> image_;

  SDL_Window* window_;

  int width_;
  int height_;
  int reset_;

  // namespace and id keys
  std::map<std::string, std::map<int, Mesh*> > meshes_;

  std::vector<uint8_t> vresult_;
  std::vector<uint8_t> fresult_;
  bgfx::ShaderHandle vhandle_;
  bgfx::ShaderHandle fhandle_;

  bgfx::ProgramHandle program_;

  bgfx::FrameBufferHandle frame_buffer_handle_;
  bgfx::TextureHandle frame_buffer_texture_[2];
  bgfx::TextureHandle read_back_texture_;
  bgfx::UniformHandle uniform_handle_;

  float at_[3];
  float eye_[3];
  uint32_t clear_color_;

public:
  bool initted_;
  bool bgfx_initted_;

  bool init()
  {
    pre_mat_.values.resize(16);
    // identity
    pre_mat_.values[0] = 1.0;
    pre_mat_.values[5] = 1.0;
    pre_mat_.values[10] = 1.0;
    pre_mat_.values[15] = 1.0;
    ros::param::get("~width", width_);
    ros::param::get("~height", height_);

    cam_pub_ = it_.advertiseCamera("image", 1);

    image_.resize(3);
    for (size_t i = 0; i < image_.size(); ++i)
    {
      image_[i] = cv::Mat(cv::Size(width_, height_), CV_8UC4);
    }

    std::string camera_name = "camera";
    ros::param::get("~camera_name", camera_name);
    std::string camera_info_url = "";
    ros::param::get("~camera_info_url", camera_info_url);
    camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(nh_,
          camera_name, camera_info_url));

    ros::param::get("~frame_id", frame_id_);
    if (!camera_info_manager_->isCalibrated())
    {
      // the loading of camera_info_url failed, so make a camera info here
      camera_info_manager_->setCameraName(camera_name);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = frame_id_;
      camera_info.width = width_;
      camera_info.height = height_;
      camera_info.distortion_model = "plumb_bob";
      camera_info.K[0] = 800;
      camera_info.K[2] = width_ / 2;
      camera_info.K[4] = 800;
      camera_info.K[5] = height_ / 2;
      camera_info.K[8] = 1.0;
      // TODO(lucasw) what should these be
      camera_info.P[0] = 800;
      camera_info.P[2] = width_ / 2;
      camera_info.P[5] = 800;
      camera_info.P[6] = height_ / 2;
      camera_info.P[10] = 1.0;
      camera_info_manager_->setCameraInfo(camera_info);
      camera_info_manager_->setCameraInfo(camera_info);
    }

    bgfx_initted_ = bgfxInit();

    pre_mat_sub_ = nh_.subscribe("pre_mat", 5, &BgfxRos::preMatCallback, this);
    marker_sub_ = nh_.subscribe("marker", 5, &BgfxRos::markerCallback, this);

    return true;
  }

  bool bgfxInit()
  {
    window_ = SDL_CreateWindow("bgfx_ros", SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED, 100, 10,
        SDL_WINDOW_HIDDEN);

    SDL_SysWMinfo wmi;
    SDL_VERSION(&wmi.version);
    if (!SDL_GetWindowWMInfo(window_, &wmi))
    {
      ROS_ERROR_STREAM("couldn't get wm info");
      return 1;
    }

    bgfx::PlatformData pd;
    pd.ndt          = wmi.info.x11.display;
    pd.nwh          = reinterpret_cast<void*>(uintptr_t(wmi.info.x11.window));
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

    bgfx::reset(width_, height_, reset_);

    bgfx::setDebug(BGFX_DEBUG_TEXT);
    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
        clear_color_, 1.0f, 0);

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
    PosNormalColorVertex::init();

    at_[0] = 0.0f;
    at_[1] = 0.0f;
    at_[2] = 1.0f;
    // this is where the camera actually is
    eye_[0] = 0.0f;
    eye_[1] = 0.0f;
    eye_[2] = 0.0f;

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
        BGFX_TEXTURE_RT);  // | BGFX_TEXTURE_U_CLAMP | BGFX_TEXTURE_V_CLAMP);
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

    // uniform_handle_ = bgfx::createUniform("uniform_handle_", bgfx::UniformType::Int1);

    frame_buffer_handle_ = bgfx::createFrameBuffer(2, frame_buffer_texture_);
    if (frame_buffer_handle_.idx == bgfx::invalidHandle)
    {
      ROS_ERROR_STREAM("couldn't create fbh");
      return false;
    }

    return true;
  }

  explicit BgfxRos(uint32_t width = 1280, uint32_t height = 720) :
    it_(nh_),
    width_(width),
    height_(height),
    frame_id_("camera_frame"),
    reset_(BGFX_RESET_VSYNC),
    initted_(false),
    bgfx_initted_(false),
    clear_color_(0x000000ff)
  {
    initted_ = init();
  }

  void shutdown()
  {
    if (!initted_) return;

    if (bgfx_initted_)
    {

      for (auto const &ns_id_map : meshes_)
      {
        for (auto const &id_pair : ns_id_map.second)
        {
          const std::string ns = ns_id_map.first;
          const size_t id = id_pair.first;
          delete meshes_[ns][id];
          meshes_[ns].erase(id);
        }
      }
      bgfx::destroyProgram(program_);
      bgfx::shutdown();
    }
    initted_ = false;
  }

  ~BgfxRos()
  {
    shutdown();
  }

  void preMatCallback(const sensor_msgs::ChannelFloat32ConstPtr& msg)
  {
    pre_mat_ = *msg;
  }
  geometry_msgs::Vector3 post_rot_;
 
  void print4x4Mat(const float* mtx, const std::string name)
  {
    std::cout << name << std::endl;
    for (size_t i = 0; i < 4; ++i)
    {
      for (size_t j = 0; j < 4; ++j)
        std::cout << mtx[i * 4 + j] << " ";
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }

  void markerCallback(const visualization_msgs::MarkerConstPtr& msg)
  {
    // delete any existing mesh with the same ns and id
    if (meshes_.count(msg->ns) > 0)
    {
      if (meshes_[msg->ns].count(msg->id) > 0)
      {
        delete meshes_[msg->ns][msg->id];
        meshes_[msg->ns].erase(msg->id);
      }
    }
    else
    {
      std::map<int, Mesh*> mesh_by_id;
      meshes_[msg->ns] = mesh_by_id;
    }
    meshes_[msg->ns][msg->id] = new Mesh(msg);
  }

  uint32_t i_;
  uint32_t buffer_ind_;

  void update()
  {
    if (!bgfx_initted_) return;

    {
      // TODO(lucasw) this doesn't appear on the texture image output,
      bgfx::dbgTextClear();
      std::stringstream ss;
      ss << "test " << i_;
      bgfx::dbgTextPrintf(20, 20,
          0x8f, ss.str().c_str());
      ROS_DEBUG_STREAM(ss.str());
    }

    // TODO(lucasw) allow every element of this matrix to be controlled
    // via topic or service call
    float view[16];
    bx::mtxLookAt(view, eye_, at_);

    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(
        camera_info_manager_->getCameraInfo()));
    // TODO(lucasw) need to take the camera info intrinsic matrix
    // and construct parts of the projection matrix from it.
    // The thing to do first is calculate an angle-of-view in degrees
    // for mtxProj.
    // The distortion can be handled downstream using a distortion node,
    // but later a gpu version would be interesting to have.
    float proj[16];
    const float fovy = 2.0 * bx::fatan2(ci->K[5], ci->K[4]);
    // const float fovx = 2.0 * bx::fatan2(ci->K[2], ci->K[0]);
    bx::mtxProj(proj, fovy * 180.0 / bx::pi,
        static_cast<float>(width_) / static_cast<float>(height_), 0.1f, 100.0f);
    // TODO(lucasw) also want ability to set projection matrix
    // entirely from message
    bgfx::setViewTransform(0, view, proj);

    // Set view 0 default viewport.
    bgfx::setViewName(0, "bgfx_ros");
    bgfx::setViewRect(0, 0, 0, bgfx::BackbufferRatio::Equal);
    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
        clear_color_, 1.0f, 0);
    bgfx::setViewFrameBuffer(0, frame_buffer_handle_);
    bgfx::touch(0);

    bgfx::setViewName(1, "backbuffer_render");
    bgfx::setViewRect(1, 0, 0, bgfx::BackbufferRatio::Equal);
    bgfx::FrameBufferHandle invalid = BGFX_INVALID_HANDLE;
    bgfx::setViewFrameBuffer(1, invalid);

    for (auto const &ns_id : meshes_)
    {
      for (auto const &id_mesh : ns_id.second)
      {
        const Mesh* const mesh = id_mesh.second;

        // TODO(lucasw) incorporate tf
        // TODO(lucasw) set this up in advance?
        tf::StampedTransform transform;
        // TODO(lucasw) also need to take the marker pose
        // into account- is there a function for that or will it have to be
        // a manual matrix multiplication?
        // TODO(lucasw) cache the transform to avoid multiple redundant lookups?
        try
        {
          // listener_.lookupTransform(mesh->marker_->header.frame_id, frame_id_,
          listener_.lookupTransform(frame_id_, mesh->marker_->header.frame_id,
              ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
          ROS_ERROR_STREAM(mesh->marker_->ns << " " << mesh->marker_->id);
          ROS_ERROR("%s", ex.what());
          continue;
        }

        float mtx[16];
        {
          double mtxd[16];
          // this flips the models front to back
          transform.getOpenGLMatrix(mtxd);
          std::copy(mtxd, mtxd + 16, mtx);
          // print4x4Mat(mtx, "opengl");
        }

        /*
        0  1  2  3
        4  5  6  7
        8  9  10 11
        12 13 14 15
        
        0  1  2  0
        4  5  6  0
        8  9  10 0
        x  y  z  1.0
        */

        float mat[16];
        for (size_t i = 0; i < 16 && i < pre_mat_.values.size(); ++i)
        {
          mat[i] = pre_mat_.values[i];
        }
        // print4x4Mat(mat, "pre mat");
        float res[16];
        bx::mtxMul(res, mat, mtx);

        // Set model matrix for rendering.
        bgfx::setTransform(res);

        bgfx::setVertexBuffer(mesh->vbh_);
        bgfx::setIndexBuffer(mesh->ibh_);

        // Set render states.
        bgfx::setState(0
            | BGFX_STATE_RGB_WRITE
            | BGFX_STATE_ALPHA_WRITE
            | BGFX_STATE_DEPTH_TEST_LESS
            | BGFX_STATE_DEPTH_WRITE
            | BGFX_STATE_CULL_CW
            | BGFX_STATE_MSAA);
          // | BGFX_STATE_PT_TRILIST    // this doesn't exist - is it the default?
        // Submit primitive for rendering to view 0.
        bgfx::submit(0, program_);
      }
    }

    // TODO(lucasw) does there need to be a isValid every update?
    if (bgfx::isValid(read_back_texture_))
    {
      bgfx::touch(1);
      bgfx::blit(1, read_back_texture_, 0, 0,
          // bgfx::getTexture(frame_buffer_handle_), 0, 0, width_, height_);
          frame_buffer_texture_[0], 0, 0, width_, height_);
      // toggle between the two image buffers
      const size_t image_ind = (buffer_ind_) % image_.size();
      const uint32_t read_frame = bgfx::readTexture(read_back_texture_,
          image_[image_ind].data);
      // Display the oldest image for debug, later publish it on a topic instead
      // Surely the oldest image is done being written to by now?
      // Could actually track the expected frame it will be done.
      // cv::imshow("image", image_[(image_ind + 1) % image_.size()]);
      // cv::waitKey(1);

      ci->header.stamp = ros::Time::now();
      ci->header.frame_id = frame_id_;
      // TODO(lwalter) later this will be have to determined earlier, has to be
      // slightly old in order for tf lookups to work
      cv_bridge::CvImage cv_image;
      cv_image.header.stamp = ci->header.stamp;
      cv_image.header.frame_id = ci->header.frame_id;
      cv_image.encoding = "bgra8";
      cv_image.image = image_[image_ind];
      sensor_msgs::ImagePtr msg = cv_image.toImageMsg();
      cam_pub_.publish(*msg, *ci);
      ++buffer_ind_;
    }

    const uint32_t cur_frame = bgfx::frame();
    // ROS_DEBUG_STREAM("read " << read_frame << ", cur " << cur_frame << " " << image_ind);
    // update sdl processes
    // SDL_Delay(1);
    SDL_PollEvent(NULL);
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
