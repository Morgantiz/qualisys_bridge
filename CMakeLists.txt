cmake_minimum_required(VERSION 3.5)
project(qualisys_bridge)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  src/qualisys_cpp_sdk/
)
add_library(qualisys_bridge_core 
  src/qualisys_cpp_sdk/Markup.cpp
  src/qualisys_cpp_sdk/Network.cpp
  src/qualisys_cpp_sdk/RTPacket.cpp
  src/qualisys_cpp_sdk/RTProtocol.cpp
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS qualisys_bridge_core
  EXPORT qualisys_bridge_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include 
)

# qualisys bridge node
add_executable(qualisys_bridge_node src/qualisys_bridge_node.cpp)
target_link_libraries(qualisys_bridge_node qualisys_bridge_core)
ament_target_dependencies(qualisys_bridge_node 
  rclcpp 
  geometry_msgs
  tf2
  tf2_ros 
)
install(TARGETS
  qualisys_bridge_node
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()