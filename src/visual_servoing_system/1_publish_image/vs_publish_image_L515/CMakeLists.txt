cmake_minimum_required(VERSION 3.0.2)
project(vs_publish_image_L515)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nodelet 
  image_transport 
  sensor_msgs 
  cv_bridge
)

find_package(realsense2 REQUIRED)

catkin_package(
CATKIN_DEPENDS cv_bridge image_transport nodelet roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIR}
)

# 编译为库 (Nodelet 的核心)
add_library(vs_publish_image_L515
  src/l515_nodelet.cpp
)

target_link_libraries(vs_publish_image_L515
  ${catkin_LIBRARIES}
  ${realsense2_LIBRARY}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS vs_publish_image_L515
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)