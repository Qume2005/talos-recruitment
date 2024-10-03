cmake_minimum_required(VERSION 3.8)
project(moving)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# 创建可执行文件
add_executable(task1 src/task1.cpp)

# 指定目标的依赖项
ament_target_dependencies(
    task1
    rclcpp
    std_msgs
    geometry_msgs
    turtlesim
)

# 指定包含头文件目录
target_include_directories(task1 PRIVATE
  include  # 包含根目录下的include文件夹
)

# 安装可执行文件
install(TARGETS
  task1
  DESTINATION lib/${PROJECT_NAME}
)

# 指定这是一个 ament 包
ament_package()
