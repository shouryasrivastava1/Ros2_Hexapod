cmake_minimum_required(VERSION 3.8)
project(hexapod_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(hexapod_msgs REQUIRED)
find_package(hexapod_kinematics REQUIRED)

add_executable(test_node src/test.cpp)
include_directories(include)


ament_target_dependencies(test_node
  rclcpp
  hexapod_msgs
  hexapod_kinematics
)


install(TARGETS test_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
