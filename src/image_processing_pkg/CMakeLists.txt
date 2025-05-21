cmake_minimum_required(VERSION 3.8)
project(image_processing_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(OpenCV REQUIRED) 
find_package(BLAS REQUIRED) 
find_package(LAPACK REQUIRED)
find_package(std_srvs REQUIRED)

# Find dlib package
find_package(dlib REQUIRED)  # This finds the dlib package if installed globally

# Add compiler options for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include dlib's headers (this should be already covered by find_package)
include_directories(${dlib_INCLUDE_DIRS})

# Add executable
add_executable(image_processing_node src/image_processing_node.cpp)

# Link dependencies and dlib
ament_target_dependencies(image_processing_node rclcpp geometry_msgs std_srvs OpenCV BLAS LAPACK)

target_link_libraries(image_processing_node ${OpenCV_LIBS} ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} ${dlib_LIBRARIES})

# Install the executable
install(TARGETS image_processing_node DESTINATION lib/${PROJECT_NAME})

# Testing support (if enabled)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()