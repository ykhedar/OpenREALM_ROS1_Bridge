cmake_minimum_required(VERSION 3.5)
project(realm_rviz_plugin)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(cv_bridge REQUIRED)
find_package(rviz REQUIRED)
find_package(realm_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geodesy REQUIRED)

set(INCLUDE_DIRS ${cv_bridge_INCLUDE_DIRS} ${rviz_INCLUDE_DIRS}
  ${realm_msgs_INCLUDE_DIRS} ${geographic_msgs_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS} ${geodesy_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${cv_bridge_LIBRARY_DIRS} ${rviz_LIBRARY_DIRS}
  ${realm_msgs_LIBRARY_DIRS} ${geographic_msgs_LIBRARY_DIRS}
  ${ament_cmake_LIBRARY_DIRS} ${geodesy_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${cv_bridge_LIBRARIES} ${rviz_LIBRARIES} ${realm_msgs_LIBRARIES}
  ${geographic_msgs_LIBRARIES} ${ament_cmake_LIBRARIES} ${geodesy_LIBRARIES})

find_package(OpenCV 3 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 4 REQUIRED)
  message(WARNING "OpenCV 4 Support is experimental, use at your own risk!")
endif()
find_package(PCL 1.7 REQUIRED) ####################
## Catkin Package ##
####################

set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake/Qt5")
find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5Widgets)

add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(MOC_FILES src/ground_image_display.h)

set(SOURCE_FILES src/ground_image_display.cpp ${MOC_FILES})

add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${QT_LIBRARIES}
  ${LIBS})

add_dependencies(${PROJECT_NAME} realm_msgs_generate_messages_cpp)

add_executable(ground_image_test_node src/ground_image_test_publisher.cpp)

add_dependencies(ground_image_test_node realm_msgs_generate_messages_cpp)
target_link_libraries(ground_image_test_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(cv_bridge)
ament_export_dependencies(rviz)
ament_export_dependencies(realm_msgs)
ament_export_dependencies(geographic_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(geodesy)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${LIBS})

ament_package()
