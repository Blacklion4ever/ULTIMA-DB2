cmake_minimum_required(VERSION 3.8)
project(dk2_driver_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dépendances ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)

# OpenHMD
find_path(OPENHMD_INCLUDE_DIR openhmd.h)
find_library(OPENHMD_LIBRARY openhmd)

if(NOT OPENHMD_INCLUDE_DIR OR NOT OPENHMD_LIBRARY)
  message(FATAL_ERROR "OpenHMD not found. Install or set OPENHMD_INCLUDE_DIR and OPENHMD_LIBRARY.")
endif()

# hidapi
find_library(HIDAPI_LIBRARY hidapi-hidraw)
if(NOT HIDAPI_LIBRARY)
  message(FATAL_ERROR "hidapi not found. Install with: sudo apt install libhidapi-dev")
endif()

# ---- CIBLE ----
add_executable(dk2_node src/dk2_node.cpp)

# Includes
target_include_directories(dk2_node PRIVATE ${OPENHMD_INCLUDE_DIR})

# Linkage
target_link_libraries(dk2_node ${OPENHMD_LIBRARY} ${HIDAPI_LIBRARY})

# ROS2 deps
ament_target_dependencies(dk2_node rclcpp sensor_msgs geometry_msgs tf2 std_srvs)

# Installation
install(TARGETS
  dk2_node
  DESTINATION lib/${PROJECT_NAME}
)

# Tests optionnels
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

