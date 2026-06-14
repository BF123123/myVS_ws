cmake_minimum_required(VERSION 3.0.2)
project(vs_control)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_filters nodelet roscpp sensor_msgs std_msgs std_srvs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 1. 把你的算法编译成一个库 (vs_control_algo)
add_library(vs_control_algo
  src/visual_servoing.cpp
  src/direct_visual_servoing.cpp
  src/depth_visual_servoing.cpp
  src/defocused_visual_servoing.cpp
  src/defocused_visual_servoing_RAL.cpp
)

# 2. 把 Nodelet 包装器编译成库 (vs_control_nodelet)
add_library(vs_control_nodelet
  src/vs_control_nodelet.cpp
)

# 3. 链接库
target_link_libraries(vs_control_algo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(vs_control_nodelet
  vs_control_algo  # 链接算法库
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 4. 安装
install(TARGETS vs_control_algo vs_control_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)