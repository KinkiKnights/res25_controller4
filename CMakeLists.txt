cmake_minimum_required(VERSION 3.8)
project(res25_controller4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kk_driver_msg REQUIRED)

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp std_msgs sensor_msgs kk_driver_msg)
install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
