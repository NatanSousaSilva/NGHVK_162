cmake_minimum_required(VERSION 3.8)
project(boca)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(CURL REQUIRED)

add_executable(boca_node src/boca_node.cpp)

ament_target_dependencies(boca_node
  rclcpp
  std_msgs
)

target_link_libraries(boca_node
  CURL::libcurl
)

install(TARGETS
  boca_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
