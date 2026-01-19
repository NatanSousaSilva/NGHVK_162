cmake_minimum_required(VERSION 3.8)
project(localizacao)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) # Adicione outras mensagens se necessário

# 2. Define o executável e o arquivo fonte
add_executable(localizacao src/localizacao_node.cpp)

# 3. Especifica as dependências de compilação (include headers)
ament_target_dependencies(localizacao
  rclcpp
  std_msgs
)

# 4. Instalação do executável (importante para o 'colcon build')
install(TARGETS
  localizacao
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
