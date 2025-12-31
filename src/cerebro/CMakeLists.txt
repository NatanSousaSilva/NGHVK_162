cmake_minimum_required(VERSION 3.8)
project(cerebro)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cerebro
  src/cerebro_node.cpp
)

ament_target_dependencies(cerebro
  rclcpp
  std_msgs
)

target_include_directories(cerebro PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  cerebro
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
