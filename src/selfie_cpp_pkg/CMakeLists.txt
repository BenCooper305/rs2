cmake_minimum_required(VERSION 3.8)
project(selfie_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(moveit_sim_move src/moveit_sim_move.cpp)
ament_target_dependencies(moveit_sim_move rclcpp moveit_ros_planning_interface)

# add_executable(moveit_sim2 src/moveit2_sim.cpp)
# ament_target_dependencies(moveit_sim2 rclcpp moveit_ros_planning_interface)

install(TARGETS
  moveit_sim_move
  #moveit_sim2
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
