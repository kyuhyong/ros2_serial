cmake_minimum_required(VERSION 3.5)
project(serial_com)

# Default to C99
#if(NOT CMAKE_C_STANDARD)
#  set(CMAKE_C_STANDARD 99)
#endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# find dependencies
find_package(ament_cmake_ros REQUIRED)

find_package(asio         REQUIRED)
find_package(Boost         REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(std_msgs     REQUIRED)
find_package(sensor_msgs  REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

#Added
#pkg_check_modules(SERIAL libserial REQUIRED)


###########
## Build ##
###########

add_executable(${PROJECT_NAME}_node 
  src/serial_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  asio
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

#Added 
#target_link_libraries(${PROJECT_NAME} core supervisor serial utm)

#############
## Install ##
#############

install(TARGETS 
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
