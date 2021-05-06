cmake_minimum_required(VERSION 3.5)
project(realm_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


set(msg_files 
      msg/Frame.msg 
      msg/Georeference.msg 
      msg/GroundImageCompressed.msg 
      msg/Pinhole.msg
      msg/CvGridMap.msg 
      msg/Depthmap.msg
      msg/PoseStamped.msg
      )
set(srv_files srv/ParameterChange.srv)

rosidl_generate_interfaces(${PROJECT_NAME} 
    ${msg_files} 
    ${srv_files}
    DEPENDENCIES 
    builtin_interfaces
    rcl_interfaces
    geometry_msgs
    sensor_msgs
    std_msgs
    )

ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

ament_package()
