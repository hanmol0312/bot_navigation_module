cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)   
find_package(yaml-cpp REQUIRED)   
find_package(tf2_ros REQUIRED)   
find_package(tf2_geometry_msgs REQUIRED)   
find_package(rclcpp_action REQUIRED)  
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED) 



add_executable(rpm_vel_controller src/rpm_vel_controller.cpp)
ament_target_dependencies(rpm_vel_controller rclcpp  nav2_msgs gazebo geometry_msgs)


install(DIRECTORY urdf
DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)



add_library(gazebo_rpm_plugin SHARED src/gazebo_rpm_plugin.cpp)
ament_target_dependencies(gazebo_rpm_plugin rclcpp std_msgs gazebo_ros)
install(TARGETS gazebo_rpm_plugin DESTINATION lib)


install(TARGETS
rpm_vel_controller
DESTINATION lib/${PROJECT_NAME})


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
