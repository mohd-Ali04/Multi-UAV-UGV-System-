cmake_minimum_required(VERSION 3.8)
project(tins8_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CommandStatus.msg msg/UavStatus.msg msg/FleetSlot.msg msg/FleetState.msg
  msg/CarouselState.msg msg/TrackStatus.msg
  srv/UavCommand.srv srv/CarouselCommand.srv srv/SelectTarget.srv srv/PursuitCommand.srv srv/ActorCommand.srv
  action/RunMission.action
  DEPENDENCIES builtin_interfaces geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
