cmake_minimum_required(VERSION 3.5)
project(exomy_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # Ensure std_msgs is found
find_package(rosidl_default_runtime REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommands.msg"
  "msg/RoverCommand.msg"
  "msg/Screen.msg"
  DEPENDENCIES std_msgs  # Link std_msgs as a dependency
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()