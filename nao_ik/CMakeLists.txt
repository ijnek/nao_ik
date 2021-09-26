cmake_minimum_required(VERSION 3.8)
project(nao_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nao_command_msgs REQUIRED)
find_package(biped_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Build ik
add_executable(ik_node
  src/ik_node.cpp
  src/ik.cpp
  src/bhuman/ik_bhuman.cpp
  src/bhuman/RotationMatrix.cpp)

ament_target_dependencies(ik_node
  rclcpp
  nao_command_msgs
  biped_interfaces
  Eigen3)

# Install
install(TARGETS
  ik_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
