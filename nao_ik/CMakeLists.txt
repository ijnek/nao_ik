cmake_minimum_required(VERSION 3.8)
project(nao_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)
find_package(biped_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(  # (Deprecated as of J-turtle)
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Build ik_node (Deprecated as of J-turtle)
add_executable(ik_node
  src/ik_node.cpp
  src/ik.cpp
  src/bhuman/ik_bhuman.cpp
  src/bhuman/RotationMatrix.cpp)

ament_target_dependencies(ik_node
  rclcpp
  nao_lola_command_msgs
  biped_interfaces
  Eigen3)

# Build nao_ik_node library, and nao_ik executable
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  nao_lola_command_msgs
  biped_interfaces
  Eigen3)

add_library(nao_ik_node SHARED
  src/nao_ik.cpp
  src/bhuman/ik_bhuman.cpp
  src/bhuman/RotationMatrix.cpp)

target_include_directories(nao_ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})

rclcpp_components_register_node(nao_ik_node
  PLUGIN "nao_ik::NaoIK"
  EXECUTABLE nao_ik)

ament_target_dependencies(nao_ik_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_targets(export_nao_ik HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
install(TARGETS  # (Deprecated as of J-turtle)
  ik_node
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS nao_ik_node
  EXPORT export_nao_ik
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_package()
