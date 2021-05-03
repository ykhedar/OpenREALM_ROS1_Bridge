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
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

set(msg_files 
      msg/Frame.msg 
      msg/Georeference.msg 
      msg/GroundImageCompressed.msg 
      msg/Pinhole.msg
      msg/CvGridMap.msg 
      msg/Depthmap.msg
      )
set(srv_files srv/ParameterChange.srv)

rosidl_generate_interfaces(${PROJECT_NAME} 
    ${msg_files} 
    ${srv_files}
    DEPENDENCIES builtin_interfaces
    )
                
set(INCLUDE_DIRS 
  ${ament_cmake_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS} 
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS} 
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS})

include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS 
  ${ament_cmake_LIBRARY_DIRS}
  ${rosidl_default_generators_LIBRARY_DIRS} 
  ${rclcpp_LIBRARY_DIRS}
  ${std_msgs_LIBRARY_DIRS} 
  ${geometry_msgs_LIBRARY_DIRS}
  ${sensor_msgs_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS 
    ${ament_cmake_LIBRARIES} 
    ${rosidl_default_generators_LIBRARIES}
    ${rclcpp_LIBRARIES} 
    ${std_msgs_LIBRARIES} 
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES})


ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
