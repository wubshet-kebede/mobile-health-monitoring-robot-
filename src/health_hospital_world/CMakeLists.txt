cmake_minimum_required(VERSION 3.8)
project(health_hospital_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install everything so ROS 2 Jazzy can find it
install(DIRECTORY launch models worlds photos docs
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # skip copyright check for now
  set(ament_cmake_copyright_FOUND TRUE)
  # skip cpplint if not in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
