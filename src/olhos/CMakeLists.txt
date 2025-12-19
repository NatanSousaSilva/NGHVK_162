cmake_minimum_required(VERSION 3.8)
project(olhos)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(olhos_node src/olhos_node.cpp)

ament_target_dependencies(olhos_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

target_link_libraries(olhos_node
  ${OpenCV_LIBS}
)

target_include_directories(olhos_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS olhos_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
