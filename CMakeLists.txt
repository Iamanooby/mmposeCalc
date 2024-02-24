cmake_minimum_required(VERSION 3.8)
project(mmposeCalc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tutorial_interfaces REQUIRED)     
find_package(geometry_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)



# Create Cpp executable
add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp geometry_msgs tutorial_interfaces)

# Install Cpp executables
install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/webcam2mmpose.py
  scripts/webcam2mmpose_front.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()