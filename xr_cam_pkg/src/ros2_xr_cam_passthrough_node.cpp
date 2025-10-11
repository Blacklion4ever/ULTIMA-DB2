#include <stdio.h>
#include <string.h>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <GL/glew.h>
#define GLFW_EXPOSE_NATIVE_X11
#define GLFW_EXPOSE_NATIVE_GLX
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#ifdef None
#undef None
#endif


#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>



#define ROTATE_90 0
#define PARALLAX  0.015f

// ---------- Shaders (identiques en sémantique) ----------
static const char* VS_SRC =
"#version 330 core\n"
"out vec2 vUV;\n"
"void main(){\n"
"  vec2 p = vec2((gl_VertexID<<1)&2, gl_VertexID&2);\n"
"  vUV = p;\n"
"  gl_Position = vec4(p*2.0-1.0, 0.0, 1.0);\n"
"}\n";

#if ROTATE_90
static const char* FS_SRC =
"#version 330 core\n"
"in vec2 vUV;\n"
"out vec4 FragColor;\n"
"uniform sampler2D cam;\n"
"uniform float eyeOffset;\n"
"uniform vec2  fbSize;\n"
"void main(){\n"
"  vec2 p = gl_FragCoord.xy / fbSize;\n"
"  vec2 uv = vec2(p.y, 1.0 - p.x);\n"
"  uv.x = 1.0 - uv.x + eyeOffset;\n"
"  uv = clamp(uv, vec2(0.0), vec2(1.0));\n"
"  FragColor = texture(cam, uv);\n"
"}\n";
#else
static const char* FS_SRC =
"#version 330 core\n"
"in vec2 vUV;\n"
"out vec4 FragColor;\n"
"uniform sampler2D cam;\n"
"uniform float eyeOffset;\n"
"uniform vec2  fbSize;\n"
"void main(){\n"
"  vec2 uv = vUV;\n"
"  uv.x = uv.x + eyeOffset;\n"
"  uv.y = 1.0 - uv.y;\n"
"  uv = clamp(uv, vec2(0.0), vec2(1.0));\n"
"  FragColor = texture(cam, uv);\n"
"}\n";
#endif

static GLuint make_shader(GLenum type, const char* src){
  GLuint sh = glCreateShader(type);
  glShaderSource(sh, 1, &src, nullptr);
  glCompileShader(sh);
  GLint ok = GL_FALSE;
  glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
  if(!ok){
    char log[4096]; GLsizei n=0;
    glGetShaderInfoLog(sh, sizeof(log), &n, log);
    fprintf(stderr, "[shader] compile error: %.*s\n", n, log);
    exit(1);
  }
  return sh;
}

static int64_t pick_fmt(XrSession s){
  uint32_t n = 0;
  xrEnumerateSwapchainFormats(s, 0, &n, nullptr);
  std::vector<int64_t> f(n);
  xrEnumerateSwapchainFormats(s, n, &n, f.data());
  for(auto v : f){
    if(v == (int64_t)GL_SRGB8_ALPHA8){ fprintf(stderr, "[xr] swapchain format GL_SRGB8_ALPHA8\n"); return GL_SRGB8_ALPHA8; }
    if(v == (int64_t)GL_RGBA8){ fprintf(stderr, "[xr] swapchain format GL_RGBA8\n"); return GL_RGBA8; }
  }
  int64_t ret = n ? f[0] : GL_RGBA8;
  fprintf(stderr, "[xr] swapchain format fallback 0x%llx\n", (long long)ret);
  return ret;
}

// ---------- Buffer caméra (ROS) ----------
struct Cam { int w=0, h=0; GLuint tex=0; };

static std::mutex frame_mtx;
static std::vector<uint8_t> frame_cpu; // RGBA
static std::atomic<bool> frame_new{false};
static std::atomic<bool> have_dims{false};
static std::atomic<int> g_w{0}, g_h{0};

static void bgr8_to_rgba(const uint8_t* src, int w, int h, int step, std::vector<uint8_t>& dst){
  dst.resize((size_t)w*h*4);
  for(int y=0;y<h;++y){ const uint8_t* row=src+(size_t)y*step;
    for(int x=0;x<w;++x){ size_t i=4*(y*w+x); dst[i+0]=row[3*x+2]; dst[i+1]=row[3*x+1]; dst[i+2]=row[3*x+0]; dst[i+3]=255; } }
}
static void rgb8_to_rgba(const uint8_t* src, int w, int h, int step, std::vector<uint8_t>& dst){
  dst.resize((size_t)w*h*4);
  for(int y=0;y<h;++y){ const uint8_t* row=src+(size_t)y*step;
    for(int x=0;x<w;++x){ size_t i=4*(y*w+x); dst[i+0]=row[3*x+0]; dst[i+1]=row[3*x+1]; dst[i+2]=row[3*x+2]; dst[i+3]=255; } }
}
static void bgra8_to_rgba(const uint8_t* src, int w, int h, int step, std::vector<uint8_t>& dst){
  dst.resize((size_t)w*h*4);
  for(int y=0;y<h;++y){ const uint8_t* row=src+(size_t)y*step;
    for(int x=0;x<w;++x){ size_t i=4*(y*w+x); dst[i+0]=row[4*x+2]; dst[i+1]=row[4*x+1]; dst[i+2]=row[4*x+0]; dst[i+3]=row[4*x+3]; } }
}
static void rgba8_copy(const uint8_t* src, int w, int h, int step, std::vector<uint8_t>& dst){
  dst.resize((size_t)w*h*4);
  for(int y=0;y<h;++y){ memcpy(dst.data()+(size_t)y*w*4, src+(size_t)y*step, (size_t)w*4); }
}

class RosCamNode : public rclcpp::Node{
public:
  RosCamNode(const std::string& topic): Node("xr_cam_passthrough_node"){
    auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic, qos, std::bind(&RosCamNode::on_img, this, std::placeholders::_1));
  }
private:
  void on_img(const sensor_msgs::msg::Image::ConstSharedPtr msg){
    int w=(int)msg->width, h=(int)msg->height, step=(int)msg->step;
    const uint8_t* data = msg->data.data();
    std::vector<uint8_t> rgba;
    const std::string& enc = msg->encoding;
    if(enc=="bgr8") bgr8_to_rgba(data,w,h,step,rgba);
    else if(enc=="rgb8") rgb8_to_rgba(data,w,h,step,rgba);
    else if(enc=="bgra8") bgra8_to_rgba(data,w,h,step,rgba);
    else if(enc=="rgba8") rgba8_copy(data,w,h,step,rgba);
    else return;

    {
      std::lock_guard<std::mutex> lk(frame_mtx);
      frame_cpu.swap(rgba);
      frame_new.store(true, std::memory_order_release);
      g_w.store(w, std::memory_order_release);
      g_h.store(h, std::memory_order_release);
      have_dims.store(true, std::memory_order_release);
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

// ---------- Main ----------
int main(int argc, char** argv){
  // ROS 2
  rclcpp::init(argc, argv);
  std::string topic = "camera/image_raw";
  for(int i=1;i<argc;i++){ if(!strcmp(argv[i],"--topic") && i+1<argc){ topic=argv[i+1]; i++; } }
  auto node = std::make_shared<RosCamNode>(topic);
  std::thread spin_thr([&](){ rclcpp::spin(node); });

  // GL caché
  if(!glfwInit()){ fprintf(stderr,"[glfw] init failed\n"); return 1; }
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  GLFWwindow* win = glfwCreateWindow(640,480,"hidden",NULL,NULL);
  glfwMakeContextCurrent(win);
  if(glewInit()!=GLEW_OK){ fprintf(stderr,"[glew] init failed\n"); return 1; }
  Display* dpy = glfwGetX11Display();
  Window xw = glfwGetX11Window(win);
  GLXContext glx = glXGetCurrentContext();
  fprintf(stderr,"[gl] context ok\n");

  // OpenXR
  const char* exts[] = { XR_KHR_OPENGL_ENABLE_EXTENSION_NAME };
  XrInstanceCreateInfo ici{XR_TYPE_INSTANCE_CREATE_INFO};
  ici.next = nullptr;
  ici.applicationInfo = {};
  std::snprintf(ici.applicationInfo.applicationName, XR_MAX_APPLICATION_NAME_SIZE, "XR Cam");
  ici.applicationInfo.apiVersion = XR_MAKE_VERSION(1,0,0);
  ici.enabledExtensionCount = 1;
  ici.enabledExtensionNames = exts;
  XrInstance inst = XR_NULL_HANDLE;
  if(xrCreateInstance(&ici,&inst)!=XR_SUCCESS){ fprintf(stderr,"[xr] xrCreateInstance failed\n"); return 1; }
  fprintf(stderr,"[xr] instance ok\n");





  XrSystemId sys = XR_NULL_SYSTEM_ID;
  XrSystemGetInfo sgi{XR_TYPE_SYSTEM_GET_INFO};
  sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
  if(xrGetSystem(inst,&sgi,&sys)!=XR_SUCCESS){ fprintf(stderr,"[xr] xrGetSystem failed\n"); return 1; }
  fprintf(stderr,"[xr] system ok\n");

  PFN_xrGetOpenGLGraphicsRequirementsKHR pReq=nullptr;
  xrGetInstanceProcAddr(inst,"xrGetOpenGLGraphicsRequirementsKHR",(PFN_xrVoidFunction*)&pReq);
  XrGraphicsRequirementsOpenGLKHR req{XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR};
  if(pReq){ pReq(inst,sys,&req); }
  fprintf(stderr,"[xr] graphics requirements ok\n");

  XrGraphicsBindingOpenGLXlibKHR bind{XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR};
  bind.xDisplay=dpy; bind.glxContext=glx; bind.glxDrawable=(GLXDrawable)xw;
  XrSession sess=XR_NULL_HANDLE;
  XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO};
  sci.next=&bind; sci.systemId=sys;
  if(xrCreateSession(inst,&sci,&sess)!=XR_SUCCESS){ fprintf(stderr,"[xr] xrCreateSession failed\n"); return 1; }
  fprintf(stderr,"[xr] session ok\n");

  XrSpace space=XR_NULL_HANDLE;
  XrReferenceSpaceCreateInfo rci{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
  rci.referenceSpaceType=XR_REFERENCE_SPACE_TYPE_STAGE;
  rci.poseInReferenceSpace.orientation.w=1.0f;
  xrCreateReferenceSpace(sess,&rci,&space);
  fprintf(stderr,"[xr] reference space ok\n");

  uint32_t viewCount=0;
  xrEnumerateViewConfigurationViews(inst,sys,XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,0,&viewCount,nullptr);
  std::vector<XrViewConfigurationView> vcfg(viewCount,{XR_TYPE_VIEW_CONFIGURATION_VIEW});
  xrEnumerateViewConfigurationViews(inst,sys,XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,viewCount,&viewCount,vcfg.data());
  fprintf(stderr,"[xr] view cfg ok, views=%u\n",viewCount);

  XrSessionBeginInfo sbi{XR_TYPE_SESSION_BEGIN_INFO};
  sbi.primaryViewConfigurationType=XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
  xrBeginSession(sess,&sbi);
  fprintf(stderr,"[xr] session begun\n");

  int64_t fmt = pick_fmt(sess);
  XrSwapchainCreateInfo scInfo{XR_TYPE_SWAPCHAIN_CREATE_INFO};
  scInfo.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
  scInfo.format=fmt;
  scInfo.width = vcfg[0].recommendedImageRectWidth;
  scInfo.height= vcfg[0].recommendedImageRectHeight;
  scInfo.faceCount=1; scInfo.arraySize=2; scInfo.mipCount=1; scInfo.sampleCount=1;
  XrSwapchain sc=XR_NULL_HANDLE;
  xrCreateSwapchain(sess,&scInfo,&sc);
  uint32_t nImg=0; xrEnumerateSwapchainImages(sc,0,&nImg,nullptr);
  std::vector<XrSwapchainImageOpenGLKHR> imgs(nImg,{XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR});
  xrEnumerateSwapchainImages(sc,nImg,&nImg,(XrSwapchainImageBaseHeader*)imgs.data());
  fprintf(stderr,"[xr] swapchain %dx%d layers=2\n",scInfo.width,scInfo.height);

  GLuint vs=make_shader(GL_VERTEX_SHADER,VS_SRC);
  GLuint fs=make_shader(GL_FRAGMENT_SHADER,FS_SRC);
  GLuint prog=glCreateProgram(); glAttachShader(prog,vs); glAttachShader(prog,fs); glLinkProgram(prog);
  glDeleteShader(vs); glDeleteShader(fs);
  GLint ok=0; glGetProgramiv(prog,GL_LINK_STATUS,&ok); if(!ok){ char log[4096]; GLsizei n=0; glGetProgramInfoLog(prog,sizeof(log),&n,log); fprintf(stderr,"[prog] link error: %.*s\n",n,log); return 1; }
  GLint uCam=glGetUniformLocation(prog,"cam");
  GLint uEye=glGetUniformLocation(prog,"eyeOffset");
  GLint uFB =glGetUniformLocation(prog,"fbSize");

  GLuint fbo=0; glGenFramebuffers(1,&fbo);
  fprintf(stderr,"[gl] fbo ok\n");

  // Dimensionnement à la première image
  fprintf(stderr,"[ros] waiting first frame on %s\n", topic.c_str());
  while(!have_dims.load(std::memory_order_acquire)){ if(!rclcpp::ok()) return 0; std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
  Cam cam; cam.w=g_w.load(std::memory_order_acquire); cam.h=g_h.load(std::memory_order_acquire);
  glGenTextures(1,&cam.tex);
  glBindTexture(GL_TEXTURE_2D,cam.tex);
  glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA8,cam.w,cam.h,0,GL_RGBA,GL_UNSIGNED_BYTE,nullptr);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);

  // Boucle XR
  while(rclcpp::ok()){
    XrFrameState fsr{XR_TYPE_FRAME_STATE};
    xrWaitFrame(sess,nullptr,&fsr);
    XrFrameBeginInfo fbi{XR_TYPE_FRAME_BEGIN_INFO};
    xrBeginFrame(sess,&fbi);

    uint32_t idx=0;
    XrSwapchainImageAcquireInfo acq{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    xrAcquireSwapchainImage(sc,&acq,&idx);
    XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO}; wi.timeout=XR_INFINITE_DURATION;
    xrWaitSwapchainImage(sc,&wi);

    glBindFramebuffer(GL_FRAMEBUFFER,fbo);
    GLuint arrayTex=imgs[idx].image;
    glViewport(0,0,scInfo.width,scInfo.height);
    glUseProgram(prog);
    glUniform1i(uCam,0);
    glUniform2f(uFB,(float)scInfo.width,(float)scInfo.height);

    // upload dernière frame si nouvelle
    if(frame_new.load(std::memory_order_acquire)){
      std::lock_guard<std::mutex> lk(frame_mtx);
      if(!frame_cpu.empty()){
        glBindTexture(GL_TEXTURE_2D,cam.tex);
        glPixelStorei(GL_UNPACK_ALIGNMENT,1);
        glTexSubImage2D(GL_TEXTURE_2D,0,0,0,cam.w,cam.h,GL_RGBA,GL_UNSIGNED_BYTE,frame_cpu.data());
        glFinish();
        frame_new.store(false,std::memory_order_release);
      }
    }

    for(int eye=0; eye<2; ++eye){
      glFramebufferTextureLayer(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,arrayTex,0,eye);
      glClearColor(0.0f,0.0f,0.0f,1.0f);
      glClear(GL_COLOR_BUFFER_BIT);
      glUniform1f(uEye, eye==0 ? -PARALLAX : +PARALLAX);
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D,cam.tex);
      glDrawArrays(GL_TRIANGLES,0,3);
    }

    glBindFramebuffer(GL_FRAMEBUFFER,0);
    XrSwapchainImageReleaseInfo rel{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
    xrReleaseSwapchainImage(sc,&rel);

    std::vector<XrView> views(viewCount,{XR_TYPE_VIEW});
    XrViewLocateInfo vli{XR_TYPE_VIEW_LOCATE_INFO};
    vli.viewConfigurationType=XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    vli.displayTime=fsr.predictedDisplayTime;
    vli.space=space;
    XrViewState vs{XR_TYPE_VIEW_STATE};
    uint32_t nviews=viewCount;
    xrLocateViews(sess,&vli,&vs,viewCount,&nviews,views.data());

    std::vector<XrCompositionLayerProjectionView> pviews(viewCount);
    for(uint32_t eye=0; eye<viewCount; ++eye){
      pviews[eye]={XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW};
      pviews[eye].pose=views[eye].pose;
      pviews[eye].fov =views[eye].fov;
      pviews[eye].subImage.swapchain=sc;
      pviews[eye].subImage.imageArrayIndex=eye;
      pviews[eye].subImage.imageRect.offset={0,0};
      pviews[eye].subImage.imageRect.extent={(int)scInfo.width,(int)scInfo.height};
    }

    XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    layer.space=space; layer.viewCount=viewCount; layer.views=pviews.data();
    const XrCompositionLayerBaseHeader* layers[]={(XrCompositionLayerBaseHeader*)&layer};
    XrFrameEndInfo fe{XR_TYPE_FRAME_END_INFO};
    fe.displayTime=fsr.predictedDisplayTime;
    fe.environmentBlendMode=XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    fe.layerCount=1; fe.layers=layers;
    xrEndFrame(sess,&fe);
  }

  rclcpp::shutdown();
  spin_thr.join();
  return 0;
}
