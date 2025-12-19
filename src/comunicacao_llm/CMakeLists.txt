cmake_minimum_required(VERSION 3.5)
project(comunicacao_llm)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependências do ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Dependências externas
find_package(nlohmann_json REQUIRED)
find_package(CURL REQUIRED)

include_directories(
    include
)

# Criar o executável
add_executable(llm_ia_node src/llm_ia_node.cpp)

# Vincular bibliotecas externas
target_link_libraries(llm_ia_node
    nlohmann_json::nlohmann_json
    CURL::libcurl
)

# Dependências ROS2
ament_target_dependencies(llm_ia_node
    rclcpp
    std_msgs
)

# Instalação
install(TARGETS
    llm_ia_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
