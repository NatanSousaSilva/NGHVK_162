cmake_minimum_required(VERSION 3.8)
project(NGHVK_160)

# Define o padrão C++
if(NOT CMAKE_CXX_STANDARD)
  	set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  	set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# -Dependências 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CURL REQUIRED) # ---biblioteca 1
find_package(nlohmann_json REQUIRED) # ---biblioteca 2

include_directories(${CURL_INCLUDE_DIRS}) # ---biblioteca 1

# -Adicionar Executáveis 
# 1. Cria o executável
add_executable(llm_ia_node src/llm_ia_node.cpp)

# 2. Define as dependências de compilação
ament_target_dependencies(llm_ia_node 
  	rclcpp 
	std_msgs
	nlohmann_json # ---biblioteca 2
)

# 3. Linka as bibliotecas necessárias
target_link_libraries(llm_ia_node
  	${CURL_LIBRARIES} # ---biblioteca 1
)

# 4. Instala o executável para ser encontrado pelo ros2 run
install(TARGETS llm_ia_node
  	DESTINATION lib/${PROJECT_NAME}
)

# Finalização 
ament_package()
