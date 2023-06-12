cmake_minimum_required(VERSION 3.8)
project(nao_ik_ims)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(biped_interfaces REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp_components REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  biped_interfaces
  interactive_markers
  rclcpp_components)

# Build
add_library(${PROJECT_NAME}_node SHARED
  src/nao_ik_ims.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "nao_ik_ims::NaoIkIms"
  EXECUTABLE ${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
