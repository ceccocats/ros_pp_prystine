cmake_minimum_required(VERSION 3.5)
project(ros_pp_prystine)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlInputData.msg"  
  "msg/ControlOutputData.msg"
  "msg/Dms.msg"
  "msg/TrafficLight.msg" 
  "msg/VehicleData.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()