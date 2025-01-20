cmake_minimum_required(VERSION 3.8)
project(my_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rosidl_default_runtime REQUIRED)


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

# Add the executable
add_executable(elevator_mover_node src/elevator_mover.cpp)

# Link dependencies
ament_target_dependencies(elevator_mover_node rclcpp std_msgs sensor_msgs rosidl_default_runtime)

#This is to install executable
install(TARGETS elevator_mover_node
	DESTINATION lib/${PROJECT_NAME}
)

# This is to install other stuff
install(
  DIRECTORY config description launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
