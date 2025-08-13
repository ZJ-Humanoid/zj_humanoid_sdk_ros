cmake_minimum_required(VERSION 3.0.2)
project(navi_demos)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosservice
  std_msgs
  std_srvs
  sensor_msgs
  actionlib
  navi_types
  msg_interface
  control_msgs  # 添加对 control_msgs 的依赖
  trajectory_msgs  # 添加对 trajectory_msgs 的依赖
)

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


catkin_package(
#  INCLUDE_DIRS include
  # LIBRARIES robothand_pkg
  CATKIN_DEPENDS roscpp rospy rosservice std_msgs sensor_msgs std_srvs navi_types actionlib control_msgs trajectory_msgs msg_interface
  # message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIB_DIRS}
)

add_executable(service_task_client src/hand_ctrl/service_task_client.cpp)
add_executable(service_joint_client src/hand_ctrl/service_joint_client.cpp)
add_executable(service_gesture_client src/hand_ctrl/service_gesture_client.cpp)
add_executable(robot_state_get_set src/robot_state/robot_state_get_set.cpp)
add_executable(set_robot_state src/robot_state/set_robot_state.cpp)

add_executable(right_arm_movej_client src/uplimb_ctrl/right_arm_movej_client.cpp)
add_executable(right_arm_movel_client src/uplimb_ctrl/right_arm_movel_client.cpp)
add_executable(teach_mode_client src/uplimb_ctrl/teach_mode_client.cpp)
add_executable(waist_trajectory_action src/uplimb_ctrl/waist_trajectory_action.cpp)
add_executable(neck_trajectory_action src/uplimb_ctrl/neck_trajectory_action.cpp)
add_executable(left_arm_trajectory_action src/uplimb_ctrl/left_arm_trajectory_action.cpp)
add_executable(left_arm_waist_trajectory_action src/uplimb_ctrl/left_arm_waist_trajectory_action.cpp)
add_executable(Wave_Movej src/uplimb_ctrl/Wave_Movej.cpp) # 挥手动作


set(CMAKE_BUILD_TYPE Debug)

set(TARGETS
  service_task_client
  service_joint_client
  service_gesture_client
  robot_state_get_set
  right_arm_movej_client
  right_arm_movel_client
  teach_mode_client
  waist_trajectory_action
  neck_trajectory_action
  left_arm_trajectory_action
  left_arm_waist_trajectory_action
  Wave_Movej
)
target_link_libraries(set_robot_state ${catkin_LIBRARIES})
foreach(TARGET ${TARGETS})
    target_link_libraries(${TARGET} ${catkin_LIBRARIES})
    add_dependencies(${TARGET} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endforeach()

foreach(TARGET ${TARGETS})
    add_dependencies(${TARGET} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endforeach()
