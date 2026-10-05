cmake_minimum_required(VERSION 3.8)
project(ur_pingpong_ball_tracker)

add_compile_options(-std=c++17)
# set(CMAKE_BUILD_TYPE "Debug")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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

include_directories(
  include
)

add_executable(ball_tracker_node
  src/main.cpp
  src/ball_tracker_node.cpp
  src/ball_tracker.cpp
)
target_link_libraries(ball_tracker_node
  Eigen3::Eigen
)
ament_target_dependencies(ball_tracker_node
  rclcpp
  sensor_msgs
  geometry_msgs
  message_filters
  OpenCV
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(ball_tracker_test
  src/ball_tracker.cpp
  test/ball_tracker_test.cpp
)
ament_target_dependencies(ball_tracker_test
  OpenCV
)

install(
  TARGETS
    ball_tracker_node
    ball_tracker_test
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
