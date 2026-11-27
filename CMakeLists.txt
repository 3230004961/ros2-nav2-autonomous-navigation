cmake_minimum_required(VERSION 3.8)
project(laser_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)
target_include_directories(obstacle_avoidance_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(obstacle_avoidance_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  obstacle_avoidance_node
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"

  nav_msgs
)

install(TARGETS obstacle_avoidance_node
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


# 安装 Gazebo 启动文件和仿真世界文件
install(
  DIRECTORY launch worlds config maps
  DESTINATION share/${PROJECT_NAME}
)


# C++ Nav2 multi-goal mission client
add_executable(nav2_mission_node src/nav2_mission_node.cpp)

target_compile_features(nav2_mission_node PUBLIC cxx_std_17)

ament_target_dependencies(nav2_mission_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  nav_msgs
)

install(TARGETS nav2_mission_node
  DESTINATION lib/${PROJECT_NAME}
)


install(
  PROGRAMS scripts/frontier_detector.py
  DESTINATION lib/${PROJECT_NAME}
)


install(
  PROGRAMS scripts/frontier_explorer.py
  DESTINATION lib/${PROJECT_NAME}
)


install(
  PROGRAMS scripts/exploration_boundary_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
