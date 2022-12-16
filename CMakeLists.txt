cmake_minimum_required(VERSION 3.8)
project(ros2_gazebo)

# clangd support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
execute_process(COMMAND "rm" "-f" "${CMAKE_SOURCE_DIR}/compile_commands.json")
execute_process(COMMAND "ln" "-s" "-r" "${CMAKE_BINARY_DIR}/compile_commands.json" "${CMAKE_SOURCE_DIR}/")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(Walker_Algo src/Walker_Algo.cpp)
ament_target_dependencies(Walker_Algo rclcpp geometry_msgs sensor_msgs)

install(TARGETS
  Walker_Algo
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

ament_package()