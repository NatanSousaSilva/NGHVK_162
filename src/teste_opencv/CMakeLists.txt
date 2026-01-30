cmake_minimum_required(VERSION 3.8)
project(teste_opencv)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(teste_opencv src/teste_opencv_node.cpp)

ament_target_dependencies(
  teste_opencv
  rclcpp
)

target_link_libraries(
  teste_opencv
  ${OpenCV_LIBS}
)

install(TARGETS
  teste_opencv
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
