cmake_minimum_required(VERSION 3.0.2)
project(vs_velocity_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_srvs
  tf
  eigen_conversions
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs std_srvs tf eigen_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 编译库文件 (类实现)
add_library(${PROJECT_NAME}_lib src/velocity_bridge.cpp)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

# 编译可执行文件 (节点入口)
add_executable(bridge_node src/bridge_node.cpp)
target_link_libraries(bridge_node ${PROJECT_NAME}_lib ${catkin_LIBRARIES})