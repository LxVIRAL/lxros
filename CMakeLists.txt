cmake_minimum_required(VERSION 3.8)
project(lxros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(BUILD_TESTING ON)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR python/${PROJECT_NAME}
)

# If later you add Python entry-point scripts / CLIs, you can install them, e.g.:
# install(
#   PROGRAMS scripts/lxros-tool
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_add_gtest(test_lxros_cpp test/test_lxros_cpp.cpp)
target_include_directories(test_lxros_cpp PRIVATE include)
ament_target_dependencies(test_lxros_cpp rclcpp std_msgs)



add_executable(lx_talker_cpp src/lx_talker_cpp.cpp)
target_include_directories(lx_talker_cpp PRIVATE include)
ament_target_dependencies(lx_talker_cpp rclcpp std_msgs)

add_executable(lx_listener_cpp src/lx_listener_cpp.cpp)
target_include_directories(lx_listener_cpp PRIVATE include)
ament_target_dependencies(lx_listener_cpp rclcpp std_msgs)




ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

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
