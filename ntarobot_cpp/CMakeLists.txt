cmake_minimum_required(VERSION 3.8)
project(ntarobot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  
find_package(Eigen3 REQUIRED)
find_package(solver_untils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(fcntl REQUIRED)
# find_package(termios REQUIRED)
# find_package(unistd REQUIRED)
# find_package(stdlib REQUIRED)

# install include
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Seccion de ejecutables
add_executable(nodo_primera_prueba src/nodo_primera_prueba.cpp)
target_include_directories(nodo_primera_prueba PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nodo_primera_prueba PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# incluir cada nodo creado
add_executable(ntarobot_controller src/ntarobot_controller.cpp src/md49_controller.cpp)
ament_target_dependencies(ntarobot_controller rclcpp solver_untils std_msgs geometry_msgs ${Eigen_LIBRARIES})

# incluir nodo de calculo de odometria
add_executable(ntarobot_odom src/ntarobot_odom.cpp)
ament_target_dependencies(ntarobot_odom 
  rclcpp
  tf2_ros
  tf2
  solver_untils
  nav_msgs
  geometry_msgs)

# Install include package
install(
  DIRECTORY include
  DESTINATION include
)

# Install targets 
install(TARGETS nodo_primera_prueba
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ntarobot_controller
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ntarobot_odom
  DESTINATION lib/${PROJECT_NAME})

# Install launch and config files
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
