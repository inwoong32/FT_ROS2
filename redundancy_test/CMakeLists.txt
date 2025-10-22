cmake_minimum_required(VERSION 3.8)
project(redundancy_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# C++ executables
add_executable(redundancy_subscriber src/redundancy_subscriber.cpp)
add_executable(redundancy_publisher src/redundancy_publisher.cpp)
ament_target_dependencies(redundancy_subscriber rclcpp std_msgs)
ament_target_dependencies(redundancy_publisher rclcpp std_msgs)

# Install C++ executables
install(TARGETS
  redundancy_subscriber
  redundancy_publisher
  DESTINATION lib/${PROJECT_NAME})

# # Install Python scripts
# install(PROGRAMS
#   scripts/redundancy_publisher.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()