cmake_minimum_required(VERSION 3.8)
project(selfie_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs  REQUIRED)

add_executable(raw_goals_publisher src/raw_goal_node.cpp)
ament_target_dependencies(raw_goals_publisher rclcpp geometry_msgs std_srvs)

add_executable(path_planning_node src/path_planning_node.cpp)
ament_target_dependencies(path_planning_node rclcpp geometry_msgs std_srvs visualization_msgs)

add_executable(ur3_driver_node src/ur3_driver_node.cpp)
ament_target_dependencies(ur3_driver_node rclcpp geometry_msgs moveit_ros_planning_interface)

install(TARGETS
  raw_goals_publisher
  path_planning_node
  ur3_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

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
