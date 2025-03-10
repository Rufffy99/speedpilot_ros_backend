cmake_minimum_required(VERSION 3.28)
project(custom_msgs)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleCommand.msg"
)