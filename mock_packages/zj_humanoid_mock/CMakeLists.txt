cmake_minimum_required(VERSION 3.0.2)
project(zj_humanoid_mock)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mock_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
