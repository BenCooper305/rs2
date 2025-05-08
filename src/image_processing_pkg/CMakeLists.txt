cmake_minimum_required(VERSION 3.8)
project(image_processing_pkg)

# Add Dlib include path
include_directories("/usr/local/include")
# Link to Dlib library
link_directories("/usr/local/lib")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Specify the location of the Dlib CMake config file
set(CMAKE_PREFIX_PATH "/home/milarb/dlib/cmake" ${CMAKE_PREFIX_PATH})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED) # ADDED
find_package(OpenCV REQUIRED) # ADDED
find_package(BLAS REQUIRED) # ADDED
# find_package(DLIB REQUIRED) # ADDED
find_package(LAPACK REQUIRED)


add_executable(image_processing_node src/image_processing_node.cpp)
ament_target_dependencies(image_processing_node rclcpp geometry_msgs)  # ADDED geometry_msgs
# target_link_libraries(test_node ${OpenCV_LIBS} )
# target_link_libraries(test_node ${OpenCV_LIBS} ${BLAS_LIBRARIES} ${DLIB_LIBRARIES})
target_link_libraries(image_processing_node ${OpenCV_LIBS} ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} dlib)
install(TARGETS image_processing_node DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()




