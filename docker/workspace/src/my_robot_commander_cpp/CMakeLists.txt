cmake_minimum_required(VERSION 3.8)
project(my_robot_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface)

add_executable(preset_path src/preset_path.cpp)
ament_target_dependencies(preset_path rclcpp moveit_ros_planning_interface)

add_executable(joint_goal src/joint_goal.cpp)
ament_target_dependencies(joint_goal rclcpp moveit_ros_planning_interface)

add_executable(pose_goal src/pose_goal.cpp)
ament_target_dependencies(pose_goal rclcpp moveit_ros_planning_interface)

add_executable(cartesian_path src/cartesian_path.cpp)
ament_target_dependencies(cartesian_path rclcpp moveit_ros_planning_interface)

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(obstacle_avoidance rclcpp moveit_ros_planning_interface)

add_executable(commander src/commander.cpp)
ament_target_dependencies(commander rclcpp moveit_ros_planning_interface std_msgs my_robot_interfaces)

install(TARGETS
  test_moveit
  preset_path
  joint_goal
  pose_goal
  cartesian_path
  obstacle_avoidance
  commander
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
