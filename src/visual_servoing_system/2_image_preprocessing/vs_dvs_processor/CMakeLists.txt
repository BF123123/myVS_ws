cmake_minimum_required(VERSION 3.0.2)
project(vs_dvs_processor)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nodelet
  roscpp
  sensor_msgs
  std_srvs
  message_filters
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include  # 【关键】暴露 include 文件夹
  CATKIN_DEPENDS cv_bridge image_transport nodelet roscpp sensor_msgs std_srvs
)

include_directories(
  include               # 【关键】包含本地 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(vs_dvs_processor
  src/dvs_processor_nodelet.cpp
  src/depth_processor_nodelet.cpp 
  src/basler_processor_nodelet.cpp
)

target_link_libraries(vs_dvs_processor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 安装规则（插件必须）
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS vs_dvs_processor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
