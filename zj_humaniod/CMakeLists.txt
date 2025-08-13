cmake_minimum_required(VERSION 3.0.2)
project(navi_types)

# 找到catkin并设置依赖项
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  trajectory_msgs  
  message_generation
  sensor_msgs
)


# 定义消息和服务文件
add_message_files(
  FILES
  Robot_StateMsg.msg
  Robot_WorkStatus.msg
  Robot_BatteryInfo.msg
  nlu_result.msg
  entity.msg
  Audio_Asr.msg
  Audio_Data.msg
  Uplimb_ServoJ.msg
  Uplimb_ServoL.msg
    # 你定义的消息文件
)

add_service_files(
  FILES
  Hand_Gesture.srv
  Hand_Joint.srv
  Hand_Task.srv
  Uplimb_MoveJ.srv
  Uplimb_MoveL.srv
  Uplimb_TeachMode.srv
  RunCmd.srv
)

add_service_files(
  FILES
  Robot_SetState.srv
  Robot_BasicInfo.srv
  Robot_WifiInfo.srv
  Robot_ConnectWifi.srv
  Robot_JsonInfo.srv
  Robot_CameraInfo.srv
)

add_service_files(
  FILES
  Audio_TTS.srv
  Audio_Listen.srv
  Audio_MediaPlay.srv
  Audio_GetVolume.srv
  Audio_SetDevice.srv
  Audio_SetVolume.srv
  Audio_GetDeviceList.srv
  Audio_VideoPlay.srv
  LLM_Chat.srv
  LLM_NLU.srv
  LLM_Session.srv
  )

add_service_files(
  FILES
  Manip_GetScenePose.srv
  Manip_GetTrajectory.srv
)

# 生成消息和服务
generate_messages(
  DEPENDENCIES
  std_msgs  # 如果你的消息依赖于 std_msgs 等其他消息
  geometry_msgs
  trajectory_msgs
)

# 声明catkin包
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime sensor_msgs # 必须声明生成消息所需的依赖
)

# 包含 ROS 头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS}  # 关键！确保包含此路径
)

# 安装设置
install(DIRECTORY msg srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

