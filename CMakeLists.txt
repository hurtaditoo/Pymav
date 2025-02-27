cmake_minimum_required(VERSION 3.8)
project(pymav)

# Buscar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Instalar los nodos Python (sin usar ament_python_install_package)
install(PROGRAMS
  src/mission_control_node.py
  src/battery_gps_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# No es necesario ament_python_install_package en este caso
# porque estamos instalando los programas directamente.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()