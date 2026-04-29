cmake_minimum_required(VERSION 3.8)
project(dodgeball)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallLocation.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_python_install_package(${PROJECT_NAME})

add_executable(dodgeball_fsm src/Dodgeball_FSM.cpp)
ament_target_dependencies(dodgeball_fsm
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  cv_bridge
)

target_include_directories(dodgeball_fsm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

rosidl_target_interfaces(dodgeball_fsm
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

install(TARGETS dodgeball_fsm
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/BallFinder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME BallFinder
)

ament_package()