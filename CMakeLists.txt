cmake_minimum_required(VERSION 3.8)
project(ros2_task_set)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_component_container_callback_isolated REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_add_library(dummy_node SHARED src/dummy_node.cpp)
ament_target_dependencies(dummy_node rclcpp rclcpp_components std_msgs rclcpp_component_container_callback_isolated)
rclcpp_components_register_nodes(dummy_node "DummyNode")

install(TARGETS dummy_node DESTINATION lib)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
