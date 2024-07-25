cmake_minimum_required(VERSION 3.8)
project(ntarobot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Seccion de importe de librerias requeridas
find_package(ament_cmake REQUIRED)

# Inclusion de paquetes "include"
include_directories(include)

# Seccion de creacion de ejecutables

# Instalacion de directorios, primero por defecto es necesario incluir el include
install(
  DIRECTORY include
  DESTINATION include
)

# Instalacion de carpetas extra, entre estas es la carpetas completmentarias de configuracion
# launch y parametros
install(
  DIRECTORY launch config params maps
  DESTINATION share/${PROJECT_NAME}
)

# Seccion de importe de librerias finales necesarias
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
