cmake_minimum_required(VERSION 3.8)
project(mapeamento)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mapeamento_node src/mapeamento_node.cpp)

ament_target_dependencies(mapeamento_node
  rclcpp
  std_msgs
)

install(TARGETS
  mapeamento_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
