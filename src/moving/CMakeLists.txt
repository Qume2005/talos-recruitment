cmake_minimum_required(VERSION 3.8)
project(moving)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

add_executable(turtle_controller src/main.cpp src/turtle_controller.cpp)

# Specify the dependencies for the target
ament_target_dependencies(
    turtle_controller
    rclcpp
    std_msgs
    geometry_msgs
    turtlesim
)

# Install the executable
install(TARGETS
  turtle_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Specify that this is an ament package
ament_package()
