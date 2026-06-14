cmake_minimum_required(VERSION 3.0.2)
project(vs_robot_manager)

find_package(catkin REQUIRED COMPONENTS rospy std_srvs geometry_msgs)

catkin_package()

# 安装 Python 脚本
catkin_install_python(PROGRAMS scripts/robot_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)