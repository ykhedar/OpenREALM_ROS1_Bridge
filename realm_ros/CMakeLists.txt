cmake_minimum_required(VERSION 3.5)

project(realm_ros)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()




find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(realm_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geodesy REQUIRED)

ament_export_dependencies(rosidl_default_runtime)


set(INCLUDE_DIRS include 
      ${ament_cmake_INCLUDE_DIRS} 
      ${tf2_INCLUDE_DIRS}
      ${std_msgs_INCLUDE_DIRS} 
      ${nav_msgs_INCLUDE_DIRS} 
      ${rclcpp_INCLUDE_DIRS}
      ${cv_bridge_INCLUDE_DIRS} 
      ${sensor_msgs_INCLUDE_DIRS}
      ${geographic_msgs_INCLUDE_DIRS} 
      ${realm_msgs_INCLUDE_DIRS}
      ${pcl_ros_INCLUDE_DIRS} 
      ${geodesy_INCLUDE_DIRS})

include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS 
      ${ament_cmake_LIBRARY_DIRS} 
      ${tf2_LIBRARY_DIRS}
      ${std_msgs_LIBRARY_DIRS} 
      ${nav_msgs_LIBRARY_DIRS} 
      ${rclcpp_LIBRARY_DIRS}
      ${cv_bridge_LIBRARY_DIRS} 
      ${sensor_msgs_LIBRARY_DIRS}
      ${geographic_msgs_LIBRARY_DIRS} 
      ${realm_msgs_LIBRARY_DIRS}
      ${pcl_ros_LIBRARY_DIRS} 
      ${geodesy_LIBRARY_DIRS})

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
      ${pcl_ros_LIBRARIES} 
      ${geodesy_LIBRARIES})

################################################################################
# Include CMake dependencies
################################################################################

include(CheckLanguage)

################################################################################
# Compiler specific configuration
################################################################################

check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  message(STATUS "CUDA Support enabled.")

  include(FindCUDA)
  set(CUDA_ARCH_LIST Auto
    CACHE
    STRING
    "List of CUDA architectures (e.g. Pascal, Volta, etc) or \
                                           compute capability versions (6.1, 7.0, etc) to generate code for. \
                                           Set to Auto for automatic detection (default).")
  cuda_select_nvcc_arch_flags(CUDA_ARCH_FLAGS ${CUDA_ARCH_LIST})
  list(APPEND CUDA_NVCC_FLAGS ${CUDA_ARCH_FLAGS})
else()
  message(STATUS "CUDA Support disabled.")
endif()

################################################################################
# Dependencies
################################################################################

find_package(OpenCV 4 REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(OpenREALM REQUIRED)
find_package(openvslam REQUIRED)

################################################################################
# Catkin Package
################################################################################

################################################################################
# Build
################################################################################

add_library(${PROJECT_NAME} SHARED 
            src/realm_ros_lib/grabber_ros_node.cpp
            src/realm_ros_lib/grabber_exiv2_node.cpp 
            src/realm_ros_lib/stage_node.cpp
            src/realm_ros_lib/conversions.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBS} ${cmake_modules_LIBRARIES} ${OpenCV_LIBRARIES} realm_stages)

add_dependencies(${PROJECT_NAME} realm_msgs_generate_messages_cpp)

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

ament_export_dependencies(ament_cmake)
ament_export_dependencies(tf2)
ament_export_dependencies(std_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geographic_msgs)
ament_export_dependencies(realm_msgs)
ament_export_dependencies(pcl_ros)
ament_export_dependencies(geodesy)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${LIBS})

ament_package()
