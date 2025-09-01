cmake_minimum_required(VERSION 3.8)
project(flocking_algorithm_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)


add_library(flocking_node_lib
  src/flocking_node.cpp
)


target_include_directories(flocking_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


target_link_libraries(flocking_node_lib
)


# autowrite: target_compile_definitions(flocking_node_lib PRIVATE "EIGEN_NO_DEBUG")


ament_target_dependencies(flocking_node_lib
rclcpp
geometry_msgs
nav_msgs
tf2
tf2_geometry_msgs
)


add_executable(flocking_node src/main.cpp)


target_link_libraries(flocking_node
flocking_node_lib
)


ament_target_dependencies(flocking_node
rclcpp
geometry_msgs
nav_msgs
tf2
tf2_geometry_msgs
)


install(TARGETS
flocking_node
flocking_node_lib
DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
DESTINATION include/
)


ament_package()