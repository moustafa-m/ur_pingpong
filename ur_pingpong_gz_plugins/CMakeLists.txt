cmake_minimum_required(VERSION 3.8)
project(ur_pingpong_gz_plugins)

add_compile_options(-std=c++17)

# 1. Find Fortress-specific Ignition packages
# Use 'ignition-gazebo6' for the simulation engine
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
# Use 'ignition-plugin1' for the plugin registration system
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)
add_library(ball_spawner SHARED
  src/ball_spawner.cpp
)

target_link_libraries(ball_spawner
  ignition-transport11::core
  ignition-gazebo6::core
  ignition-plugin1::register
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

install(TARGETS ball_spawner
  EXPORT ball_spawner
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
