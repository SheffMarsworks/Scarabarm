cmake_minimum_required(VERSION 3.22)
project(scarabarm_moveit)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Include headers if any
include_directories(
  include
)

# Build the ODrive hardware plugin
add_library(odrive_system SHARED
  src/odrive_system.cpp  # Adjust if needed
)

ament_target_dependencies(odrive_system
  hardware_interface
  pluginlib
  rclcpp
)

# Export the plugin description
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

# Install the plugin library and headers
install(TARGETS odrive_system
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin description
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch, config, and setup files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
