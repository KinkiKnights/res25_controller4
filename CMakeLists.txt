cmake_minimum_required(VERSION 3.8)
project(res25_test_controller4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存パッケージ
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kk_driver_msg REQUIRED)

# 実行ファイルの追加
add_executable(controller src/controller.cpp)

# 実行ファイルに必要な依存ライブラリをリンク
ament_target_dependencies(controller
  rclcpp
  std_msgs
  sensor_msgs
  kk_driver_msg
)

# ビルド成果物のインストール先を指定
install(TARGETS controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
