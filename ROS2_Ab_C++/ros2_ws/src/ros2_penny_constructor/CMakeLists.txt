cmake_minimum_required(VERSION 3.5)
project(ros2_penny_constructor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(penny_tray_constructor src/penny_constructor.cpp)
ament_target_dependencies(penny_tray_constructor rclcpp std_msgs)

install(TARGETS
  penny_tray_constructor
  DESTINATION lib/${PROJECT_NAME})

ament_package()
