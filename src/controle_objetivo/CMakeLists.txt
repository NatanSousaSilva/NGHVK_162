cmake_minimum_required(VERSION 3.8)
project(controle_objetivo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(brain_node
  src/brain_node.cpp  
)

ament_target_dependencies(brain_node
  rclcpp
  std_msgs
)

target_include_directories(brain_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  brain_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
