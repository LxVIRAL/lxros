cmake_minimum_required(VERSION 3.8)
project(lxros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

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
