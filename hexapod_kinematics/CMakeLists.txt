cmake_minimum_required(VERSION 3.8)
project(hexapod_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hexapod_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(HexaKinematicsSolver src/HexaKinematics.cpp)

target_link_libraries(HexaKinematicsSolver)

ament_target_dependencies(HexaKinematicsSolver
  geometry_msgs
  hexapod_msgs
)


target_include_directories(HexaKinematicsSolver PUBLIC
  $<BUILD_INTERFACE:${CAMKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>

)


install(TARGETS HexaKinematicsSolver
  EXPORT export_HexaKinematicsSolver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin

)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_HexaKinematicsSolver HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(HexaKinematicsSolver)

ament_package()
