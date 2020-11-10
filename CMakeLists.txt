cmake_minimum_required(VERSION 2.8.3)
project(auto_dock)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  geometry_msgs
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  tf
  apriltag
  apriltag_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES controller
  CATKIN_DEPENDS 
    geometry_msgs
    pluginlib
    roscpp
    sensor_msgs
    std_msgs
    tf
    apriltag_ros
  DEPENDS
   OpenCV
   apriltag

)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include include/
)

add_executable(controller src/controller.cpp)
add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(controller
  ${catkin_LIBRARIES}
)
