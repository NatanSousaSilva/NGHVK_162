cmake_minimum_required(VERSION 3.8)
project(memoria)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(memoria
  src/memoria_node.cpp
)

ament_target_dependencies(memoria
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(memoria
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  memoria
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
