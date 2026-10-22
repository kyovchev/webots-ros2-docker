cmake_minimum_required(VERSION 3.8)
project(my_robot_rviz_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(my_robot_interfaces REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/my_robot_rviz_panel/panel.hpp
)

add_library(my_robot_rviz_panel src/panel.cpp ${MOC_FILES})
target_include_directories(my_robot_rviz_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(my_robot_rviz_panel
  pluginlib
  rviz_common
  my_robot_interfaces
)
install(TARGETS my_robot_rviz_panel
        EXPORT export_my_robot_rviz_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES icons/classes/MyRobotPanel.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_my_robot_rviz_panel)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()
