cmake_minimum_required(VERSION 3.8)
project(localizacao)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(localizacao_node src/localizacao_node.cpp)

ament_target_dependencies(localizacao_node
  rclcpp
  std_msgs
)

install(TARGETS
  localizacao_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
