cmake_minimum_required(VERSION 3.8)
project(ntarobot_execpkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Seccion de paquetes utilizados
find_package(ament_cmake REQUIRED)

# Incluir paquetes de "include"
include_directories(include)

# Seccion de ejecutables en caso de se creen nodos en este paquete


# Instalacion de directorios necesarios
install(
  DIRECTORY include
  DESTINATION include
)

# Instalacion de launch y config
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Seccion de construccion de extensiones necesarias
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
