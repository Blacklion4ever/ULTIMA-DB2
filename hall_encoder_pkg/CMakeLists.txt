cmake_minimum_required(VERSION 3.8)
project(hall_encoder_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ================= ROS 2 NODE =================

add_executable(wheel_encoder_node src/wheel_encoder_node.cpp)
ament_target_dependencies(wheel_encoder_node rclcpp std_msgs ament_index_cpp)
target_link_libraries(wheel_encoder_node gpiod pthread)

add_executable(motor_encoder_node src/motor_encoder_node.cpp)
ament_target_dependencies(motor_encoder_node rclcpp std_msgs ament_index_cpp)
target_link_libraries(motor_encoder_node gpiod pthread)

# ================= INSTALL =================
install(TARGETS
  wheel_encoder_node
  motor_encoder_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  FILES
    config/motor_lut.csv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
