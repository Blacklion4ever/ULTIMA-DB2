cmake_minimum_required(VERSION 3.8)
project(xr_cam_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(X11 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenXR REQUIRED)

pkg_check_modules(GLFW3 REQUIRED glfw3)
pkg_check_modules(OPENXR REQUIRED openxr)



add_executable(xr_cam_passthrough_node src/ros2_xr_cam_passthrough_node.cpp)
target_compile_features(xr_cam_passthrough_node PRIVATE cxx_std_17)
target_compile_definitions(xr_cam_passthrough_node PRIVATE
  XR_USE_PLATFORM_XLIB XR_USE_GRAPHICS_API_OPENGL)

target_include_directories(xr_cam_passthrough_node PRIVATE
  ${GLFW3_INCLUDE_DIRS} ${OPENXR_INCLUDE_DIRS})

ament_target_dependencies(xr_cam_passthrough_node rclcpp sensor_msgs)


target_link_libraries(xr_cam_passthrough_node OpenXR::openxr_loader OpenGL::GL GLEW::GLEW X11::X11 ${GLFW3_LIBRARIES})

install(TARGETS xr_cam_passthrough_node
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
