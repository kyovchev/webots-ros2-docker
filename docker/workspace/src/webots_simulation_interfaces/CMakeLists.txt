cmake_minimum_required(VERSION 3.8)
project(webots_simulation_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetObjectPose.srv"
  "srv/GetObjectPose.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()