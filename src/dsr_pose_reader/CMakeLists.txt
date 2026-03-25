cmake_minimum_required(VERSION 3.8)
project(dsr_pose_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Doosan Robot API (DRFL) — imported as SYSTEM to suppress vendor warnings
set(DRFL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/doosan_api/include")
set(DRFL_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/doosan_api/lib")

add_library(drfl STATIC IMPORTED)
set_target_properties(drfl PROPERTIES
  IMPORTED_LOCATION "${DRFL_LIB_DIR}/libDRFL.a"
  INTERFACE_INCLUDE_DIRECTORIES "${DRFL_INCLUDE_DIR}"
)
# Mark DRFL headers as SYSTEM (suppresses -Wpedantic / -Wunused-variable from vendor code)
target_include_directories(drfl SYSTEM INTERFACE "${DRFL_INCLUDE_DIR}")

add_executable(pose_reader_node
  src/pose_reader_node.cpp
)

target_include_directories(pose_reader_node PRIVATE include)

target_link_libraries(pose_reader_node
  drfl
  ${DRFL_LIB_DIR}/libPocoFoundation.so
  ${DRFL_LIB_DIR}/libPocoNet.so
  pthread
)

ament_target_dependencies(pose_reader_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS pose_reader_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
