cmake_minimum_required(VERSION 3.5)
project(realm_ros)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

################################################################################
# Dependencies
################################################################################
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)
find_package(message_filters REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(PCL 1.8 REQUIRED QUIET COMPONENTS common io)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(CGAL)
find_package (Eigen3 3.3 REQUIRED)
find_package(FLANN 1.7.0 REQUIRED)

find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)


find_package(OpenCV 4 REQUIRED)
find_package(OpenREALM REQUIRED)
find_package(openvslam REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(realm_msgs REQUIRED)
find_package(geodesy REQUIRED)

include_directories(
  include
  ${PCL_COMMON_INCLUDE_DIRS}
)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


set(dependencies
      rclcpp
      rclcpp_components
      rcpputils
      message_filters
      tf2_ros
      cv_bridge
      std_msgs
      nav_msgs
      sensor_msgs
      geographic_msgs
      realm_msgs
      geodesy
      pcl_conversions
)

set(INCLUDE_DIRS include 
      ${ament_cmake_INCLUDE_DIRS} 
      ${tf2_ros_INCLUDE_DIRS}
      ${std_msgs_INCLUDE_DIRS} 
      ${nav_msgs_INCLUDE_DIRS} 
      ${rclcpp_INCLUDE_DIRS}
      ${cv_bridge_INCLUDE_DIRS} 
      ${sensor_msgs_INCLUDE_DIRS}
      ${geographic_msgs_INCLUDE_DIRS} 
      ${realm_msgs_INCLUDE_DIRS}
      ${geodesy_INCLUDE_DIRS}
      ${pcl_conversions_INCLUDE_DIRS})

include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS 
      ${ament_cmake_LIBRARY_DIRS} 
      ${tf2_ros_LIBRARY_DIRS}
      ${std_msgs_LIBRARY_DIRS} 
      ${nav_msgs_LIBRARY_DIRS} 
      ${rclcpp_LIBRARY_DIRS}
      ${cv_bridge_LIBRARY_DIRS} 
      ${sensor_msgs_LIBRARY_DIRS}
      ${geographic_msgs_LIBRARY_DIRS} 
      ${realm_msgs_LIBRARY_DIRS}
      ${geodesy_LIBRARY_DIRS}
      ${pcl_conversions_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS 
      ${ament_cmake_LIBRARIES} 
      ${tf2_LIBRARIES} 
      ${std_msgs_LIBRARIES}
      ${nav_msgs_LIBRARIES} 
      ${rclcpp_LIBRARIES} 
      ${cv_bridge_LIBRARIES}
      ${sensor_msgs_LIBRARIES} 
      ${geographic_msgs_LIBRARIES} 
      ${realm_msgs_LIBRARIES}
      ${geodesy_LIBRARIES}
      ${pcl_conversions_LIBRARIES})


################################################################################
# Build
################################################################################
add_library(${PROJECT_NAME} SHARED 
            src/realm_ros_lib/grabber_ros_node.cpp
            src/realm_ros_lib/grabber_exiv2_node.cpp 
            src/realm_ros_lib/stage_node.cpp
            src/realm_ros_lib/conversions.cpp)

target_include_directories(${PROJECT_NAME} 
            PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} 
            PUBLIC 
            ${LIBS} 
            ${OpenCV_LIBRARIES} 
            realm_stages)

# add_dependencies(${PROJECT_NAME} ${dependencies})

################################################################################
# Executables
################################################################################

add_executable(realm_ros_grabber src/grabber_ros_node_main.cpp)
target_link_libraries(realm_ros_grabber PUBLIC ${PROJECT_NAME})

add_executable(realm_exiv2_grabber src/grabber_exiv2_node_main.cpp)
target_link_libraries(realm_exiv2_grabber PUBLIC ${PROJECT_NAME})

add_executable(realm_stage_node src/stage_node_main.cpp)
target_link_libraries(realm_stage_node PUBLIC ${PROJECT_NAME})

################################################################################
# Install
################################################################################

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

ament_export_include_directories(${INCLUDE_DIRS})
#ament_export_libraries(${LIBS})


ament_export_dependencies(ament_cmake)
#ament_export_dependencies(ament_cmake_ros)
ament_export_dependencies(${dependencies})


ament_package()