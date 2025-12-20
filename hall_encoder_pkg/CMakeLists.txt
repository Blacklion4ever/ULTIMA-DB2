cmake_minimum_required(VERSION 3.8)
project(hall_encoder_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ================= CORE LIBRARY =================
add_library(hall_encoder_core
  src/core/Encoder.cpp
  src/core/EncoderManager.cpp
  src/hal/GpioLibgpiod.cpp
)

target_include_directories(hall_encoder_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(hall_encoder_core
  gpiod
  pthread
)

# ================= ROS 2 NODE =================
add_executable(encoder_node
  src/encoder_node.cpp
)

target_link_libraries(encoder_node hall_encoder_core)
ament_target_dependencies(encoder_node rclcpp)

# ================= STANDALONE APP =================
add_executable(encoder_monitor
  apps/encoder_monitor.cpp
)

target_link_libraries(encoder_monitor hall_encoder_core)

# ================= INSTALL =================
install(TARGETS
  hall_encoder_core
  encoder_node
  encoder_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
