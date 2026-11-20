cmake_minimum_required(VERSION 3.8)
project(tins8_gazebo)
find_package(ament_cmake REQUIRED)
install(DIRECTORY models worlds config data scripts licenses DESTINATION share/${PROJECT_NAME})
# Gazebo finds model://<name> under these directories after `source install/setup.bash`.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/tins8_gazebo.dsv.in")
ament_package()
