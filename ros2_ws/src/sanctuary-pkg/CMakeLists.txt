cmake_minimum_required(VERSION 3.5)
project(sanctuary-pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(fk_node src/publisher_member_function.cpp src/three_dof_arm.cpp)
ament_target_dependencies(fk_node rclcpp std_msgs geometry_msgs)


add_executable(ik_node src/subscriber_member_function.cpp src/three_dof_arm.cpp)
ament_target_dependencies(ik_node rclcpp std_msgs geometry_msgs)



target_include_directories(fk_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS
  fk_node
  ik_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/ DESTINATION include/)

ament_package()
