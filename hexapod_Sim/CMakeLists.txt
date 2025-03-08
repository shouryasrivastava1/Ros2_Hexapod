cmake_minimum_required(VERSION 3.8)
project(hexapod_Sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hexapod_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
include_directories(include)
add_executable(HexaTopicConverter src/ControlTopicConverter.cpp)
add_executable(HexaTopicConverterFull src/FullTopicConverter.cpp)

ament_target_dependencies(HexaTopicConverter
  rclcpp
  hexapod_msgs
  sensor_msgs
)

ament_target_dependencies(HexaTopicConverterFull
  rclcpp
  hexapod_msgs
  sensor_msgs
)

install(TARGETS HexaTopicConverter
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS HexaTopicConverterFull
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch/ meshes/ config/
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)



ament_package()
