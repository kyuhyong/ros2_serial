cmake_minimum_required(VERSION 3.5)
project(serial_com)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)

#find_package(AsioSerial CONFIG REQUIRED)
#find_package(Boost         REQUIRED COMPONENTS thread)
find_package(Boost 1.58.0 COMPONENTS system filesystem thread REQUIRED)

find_package(fmt         REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(std_msgs     REQUIRED)
find_package(sensor_msgs  REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(spdlog REQUIRED)


###########
## Build ##
###########
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  spdlog
)

# Build library
add_library(AsioSerial SHARED 
  src/AsioSerial.cpp
)

target_link_libraries(AsioSerial Boost::thread ${BOOST_LIBRARIES})
ament_target_dependencies(AsioSerial ${THIS_PACKAGE_INCLUDE_DEPENDS})

include_directories(include)

add_executable(${PROJECT_NAME}_node src/serial_node.cpp)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

#Added 
target_link_libraries(${PROJECT_NAME}_node AsioSerial fmt ${BOOST_LIBRARIES})

#############
## Install ##
#############

install(TARGETS 
  AsioSerial
  ${PROJECT_NAME}_node
  DESTINATION lib/
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
