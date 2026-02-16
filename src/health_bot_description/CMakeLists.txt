cmake_minimum_required(VERSION 3.8)
project(health_bot_description)

# 1. Explicitly find the tools your project uses
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(xacro REQUIRED)

# 2. Compiler flags (Keep these for safety)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 3. The Installation (Professional Layout)
install(DIRECTORY
  launch
  urdf
  worlds
  config  # Added config for your future SLAM/Nav2 parameters
  DESTINATION share/${PROJECT_NAME}
)

# 4. Testing (You can keep this as is)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
