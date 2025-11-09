# ZJ Humanoid ROS API 接口文档

**Description**: ZJ Humanoid ROS APIs - Services and Topics
**Version**: v1.0.0
**Generated At**: 2025-11-09 09:46:50

## Services

### /zj_humanoid/audio/LLM_chat

**Description**: LLM对话服务

**Type**: `audio/LLM_chat`

**Note**: 语音模块的版本号是多少

---

### /zj_humanoid/audio/listen

**Description**: 倾听服务

**Type**: `audio/Listen`

**Note**: 开始倾听

---

### /zj_humanoid/audio/media_play

**Description**: 音频文件播放

**Type**: `audio/MediaPlay`

**Note**: 播放’公司介绍.mp3‘

---

### /zj_humanoid/audio/microphone/get_devices_list

**Description**: 麦克风列表

**Type**: `audio/GetDeviceList`

**Note**: 检查当前有多少个麦克风设备 回复数量应大于1

---

### /zj_humanoid/audio/microphone/select_device

**Description**: 选中麦克风

**Type**: `audio/SetDevice`

**Note**: 选择第一个麦克风

---

### /zj_humanoid/audio/speaker/get_devices_list

**Description**: 获取播放设备

**Type**: `audio/GetDeviceList`

**Note**: 检查当前有多少个喇叭设备 回复数量应大于1

---

### /zj_humanoid/audio/speaker/get_volume

**Description**: 获取当前音量

**Type**: `audio/GetVolume`

**Note**: 获取当前的系统音量大小 应回复音量0~100

---

### /zj_humanoid/audio/speaker/select_device

**Description**: 选中生效喇叭

**Type**: `audio/SetDevice`

**Note**: 选择第一个喇叭

---

### /zj_humanoid/audio/speaker/set_volume

**Description**: 设置音量大小

**Type**: `audio/SetVolume`

**Note**: 设置音量为50

---

### /zj_humanoid/audio/tts_service

**Description**: 文字转语音

**Type**: `audio/TTS`

**Note**: 请让机器人说‘hello world‘

---

### /zj_humanoid/audio/version

**Description**: 语音模块的版本号

**Type**: `std_srvs/Trigger`

---

### /zj_humanoid/hand/finger_pressures/left/zero

**Description**: 置零手指传感器

**Type**: `std_srvs/Trigger`

**Note**: 置零压力传感器数值

**Demos**: data.yaml

---

### /zj_humanoid/hand/gesture_switch/dual

**Description**: 双手手势切换

**Type**: `hand/Gesture`

**Demos**: dual_hand_gesture_switch.yaml

---

### /zj_humanoid/hand/gesture_switch/left

**Description**: 左手手势切换

**Type**: `hand/Gesture`

**Demos**: left_hand_gesture_switch.yaml

---

### /zj_humanoid/hand/gesture_switch/right

**Description**: 右手手势切换

**Type**: `hand/Gesture`

**Demos**: right_hand_gesture_switch.yaml

---

### /zj_humanoid/hand/joint_switch/dual

**Description**: 双手手掌关节运动

**Type**: `hand/HandJoint`

**Demos**: dual_hand_joint_reset.yaml, dual_hand_set_gesture_two.yaml, dual_hand_set_gesture_yeah.yaml

---

### /zj_humanoid/hand/joint_switch/left

**Description**: 左手手掌关节运动

**Type**: `hand/HandJoint`

**Note**: 左手食指弯曲40度 订阅/hand_joint_states左手食指数值应接近40度

**Demos**: left_hand_joint_reset.yaml, left_hand_set_gesture_two.yaml, left_hand_set_gesture_yeah.yaml

---

### /zj_humanoid/hand/joint_switch/right

**Description**: 右手手掌关节运动

**Type**: `hand/HandJoint`

**Note**: 右手食指弯曲40度 订阅/hand_joint_states左手食指数值应接近40度

**Demos**: right_hand_joint_reset.yaml, right_hand_set_gesture_two.yaml, right_hand_set_gesture_yeah.yaml

---

### /zj_humanoid/hand/task_switch/left

**Description**: 左手掌任务控制

**Type**: `std_srvs/Bool`

---

### /zj_humanoid/hand/task_switch/right

**Description**: 右手掌任务控制

**Type**: `std_srvs/Bool`

---

### /zj_humanoid/hand/versions

**Description**: 灵巧手版本号

**Type**: `std_srvs/Trigger`

**Note**: 查询当前灵巧手子系统的版本号

---

### /zj_humanoid/hand/wrist_force_sensor/left/zero

**Description**: 置零腕部传感器

**Type**: `std_srvs/Trigger`

**Note**: 置零腕部传感器数值

**Demos**: data.yaml

---

### /zj_humanoid/lowerlimb/versions

**Description**: 下肢模块版本

**Type**: `std_srvs/Trigger`

---

### /zj_humanoid/manipulation/camera_calibration

**Description**: 相机内外参标定

**Type**: `manipulation/CameraCalibration`

**Note**: 自动相机内外参标定，外参标定时机器人会执行一段轨迹，拍摄不同角度的照片，从而计算外参

**Demos**: data.yaml

---

### /zj_humanoid/manipulation/execute_pick_task

**Description**: 执行抓取服务

**Type**: `manipulation/ExecutePickTask`

**Note**: 输出物品名称执行抓取服务

**Demos**: data.yaml

---

### /zj_humanoid/manipulation/grasp_teach_service

**Description**: 视觉抓取示教

**Type**: `manipulation/GraspTeach`

**Note**: 视觉示教抓取，让机器人知道该从什么方位抓取物品

**Demos**: data.yaml

---

### /zj_humanoid/manipulation/joint_space_trajectory_planner

**Description**: 关节空间轨迹规划

**Type**: `manipulation/GetTrajectory`

**Note**: 节空间轨迹规划，输出关节轨迹，示教模式下记录各个关节数据，据此生成完整的执行轨迹

**Demos**: data.yaml

---

### /zj_humanoid/manipulation/pose_estimation_service

**Description**: 获取目标位姿

**Type**: `manipulation/PoseEst`

**Note**: 输入图像获取指定物品的6D位姿

**Demos**: data.yaml

---

### /zj_humanoid/manipulation/pose_space_trajectory_planner

**Description**: 末端轨迹规划

**Type**: `manipulation/MotionPlan`

**Note**: 末端空间轨迹规划，示教模式下记录各个末端执行器数据，据此生成完整的执行轨迹

**Demos**: data.yaml

---

### /zj_humanoid/manipulation/scene_update

**Description**: 场景更新

**Type**: `manipulation/SceneUpdate`

**Note**: 机器人场景更新,基于二维码，需要场景中有二维码，机器人抓取物品前的环境感知

**Demos**: data.yaml

---

### /zj_humanoid/manipulation/version

**Description**: 操作模块版本号

**Type**: ``

**Demos**: data.yaml

---

### /zj_humanoid/navigation/version

**Description**: 定位导航版本号

**Type**: `std_srvs/Trigger`

**Demos**: data.yaml

---

### /zj_humanoid/robot/basic_info

**Description**: 机器人基础信息

**Type**: `zj_robot/BasicInfo`

**Note**: 描述下机器人的基础信息 回复应包含机器人的型号，硬件版本号，软件版本号，IP地址

**Demos**: I2_info.yaml, WA2_info.yaml

---

### /zj_humanoid/robot/face_show/media_play

**Description**: 脸部显示视频

**Type**: `zj_robot/FaceShow`

**Note**: 机器人脸部屏幕显示,播放视频或图像文件，如播放“Hello_World.mp4”

**Demos**: Robot_media_play.yaml

---

### /zj_humanoid/robot/face_show/text_show

**Description**: 脸部显示文字

**Type**: `zj_robot/FaceText`

**Note**: 机器人脸部屏幕显示文字，支持指令显示“Hello World”

**Demos**: Robot_text_show.yaml

---

### /zj_humanoid/robot/joint_motor/set_zero

**Description**: 电机自动标零

**Type**: `zj_robot/SetZero`

**Note**: 机器人关节自动标零

---

### /zj_humanoid/robot/orin_states/connect_wifi

**Description**: orin连接wifi

**Type**: `zj_robot/ConnectWifi`

**Note**: 尝试让机器人大脑orin去连接wifi热点

---

### /zj_humanoid/robot/orin_states/wifi_list

**Description**: orin_wifi列表

**Type**: `zj_robot/WifiList`

**Note**: 获取机器人大脑检测到的wifi热点名称，当前机器人大脑检测到多少个wifi信号 回复应大于1

**Demos**: wifi_detected.yaml

---

### /zj_humanoid/robot/pico_states/connect_wifi

**Description**: pico连接wifi

**Type**: `zj_robot/ConnectWifi`

**Note**: 尝试让机器人小脑pico去连接wifi热点

**Demos**: connect_wifi_pico.yaml

---

### /zj_humanoid/robot/pico_states/wifi_list

**Description**: pico_wifi列表

**Type**: `zj_robot/WifiList`

**Note**: 获取机器人小脑检测到的wifi热点名称，当前机器人小脑检测到多少个wifi信号 回复应大于1

**Demos**: connect_wifi_pico.yaml

---

### /zj_humanoid/robot/set_robot_state/OFF

**Description**: 机器人关机

**Type**: `std_srvs/Trigger`

**Note**: 将机器人关机 3秒后，大小脑关机，之后后没法检测到机器人建立ros链接

---

### /zj_humanoid/robot/set_robot_state/restart

**Description**: 状态机重启

**Type**: `std_srvs/Trigger`

**Note**: 机器人先进stop软急停状态，再自动变为RUN启动运行，在某些故障状态可以执行，但如果有异常的存在，也可能会失败，将机器人状态机重启 持续检测robot_state话题，经过最长60秒钟的等待，状态应切换为：RUN

---

### /zj_humanoid/robot/set_robot_state/run

**Description**: 状态机运行

**Type**: `std_srvs/Trigger`

**Note**: 如果机器人处于非RUN状态，尝试将机器人状态值设置为RUN，但如果有异常的存在，也可能会失败，将机器人状态设置为RUN 持续检测robot_state话题，经过最长60秒钟的等待，状态应切换为：RUN

---

### /zj_humanoid/robot/set_robot_state/stop

**Description**: 机器人软急停

**Type**: `std_srvs/Trigger`

**Note**: 机器人软急停状态，状态机值将切换为ERR，在机器发生异常时使用，将机器人状态设置为stop 1秒后，检测robot_state话题，状态应切换为：ERR

---

### /zj_humanoid/sensor/CAM_A/camera_info

**Description**: 左眼参数信息

**Type**: `sensor/CameraInfo`

**Note**: 相机A的参数信息,相机A安装在机器人左眼的位置上，相机A的分辨率是多少 回复应包含1280和720

---

### /zj_humanoid/sensor/CAM_B/camera_info

**Description**: 右眼参数信息

**Type**: `sensor/CameraInfo`

**Note**: 相机B的参数信息，相机B安装在机器人右眼的位置上，相机B的分辨率是多少 回复应包含1280和720

---

### /zj_humanoid/sensor/CAM_C/camera_info

**Description**: 相机C参数信息

**Type**: `sensor/CameraInfo`

**Note**: 相机C的参数信息，相机C大致安装在机器人右侧太阳穴的位置上，相机C的分辨率是多少 回复应包含1280和720

---

### /zj_humanoid/sensor/CAM_D/camera_info

**Description**: 相机D参数信息

**Type**: `sensor/CameraInfo`

**Note**: 相机D的参数信息，相机D大致安装在机器人左侧太阳穴的位置上，相机D的分辨率是多少 回复应包含1280和720

---

### /zj_humanoid/upperlimb/FK/left_arm

**Description**: 左臂正解

**Type**: `upperlimb/FK`

**Demos**: case1.yaml

---

### /zj_humanoid/upperlimb/FK/right_arm

**Description**: 右臂正解

**Type**: `upperlimb/FK`

**Demos**: case1.yaml

---

### /zj_humanoid/upperlimb/IK/left_arm

**Description**: 左臂逆解

**Type**: `upperlimb/IK`

**Note**: 左臂逆解

**Demos**: case1.yaml

---

### /zj_humanoid/upperlimb/IK/right_arm

**Description**: 右臂逆解

**Type**: `upperlimb/IK`

**Demos**: case1.yaml

---

### /zj_humanoid/upperlimb/go_down/dual_arm

**Description**: 双臂放下

**Type**: `std_srvs/Trigger`

**Demos**: data.yaml

---

### /zj_humanoid/upperlimb/go_down/left_arm

**Description**: 左臂放下

**Type**: `std_srvs/Trigger`

**Demos**: data.yaml

---

### /zj_humanoid/upperlimb/go_down/right_arm

**Description**: 右臂放下

**Type**: `std_srvs/Trigger`

**Demos**: data.yaml

---

### /zj_humanoid/upperlimb/go_home/dual_arm

**Description**: 双臂回到home点

**Type**: `std_srvs/Trigger`

**Note**: 双臂回到内置设置的home点

**Demos**: data.yaml

---

### /zj_humanoid/upperlimb/go_home/left_arm

**Description**: 左臂回到home点

**Type**: `std_srvs/Trigger`

**Note**: 左臂回到内置设置的home点

**Demos**: data.yaml

---

### /zj_humanoid/upperlimb/go_home/lifting

**Description**: 升降回到内home点

**Type**: `std_srvs/Trigger`

**Note**: 升降回到内置设置的home点

**Demos**: data.yaml

---

### /zj_humanoid/upperlimb/go_home/neck

**Description**: 脖子回到home点

**Type**: `std_srvs/Trigger`

**Note**: 脖子回到内置设置的home点

**Demos**: data.yaml

---

### /zj_humanoid/upperlimb/go_home/right_arm

**Description**: 右臂回到home点

**Type**: `std_srvs/Trigger`

**Note**: 右臂回到内置设置的home点

**Demos**: data.yaml

---

### /zj_humanoid/upperlimb/go_home/waist

**Description**: 腰部回到home点

**Type**: `std_srvs/Trigger`

**Note**: 腰部回到内置设置的home点

**Demos**: data.yaml

---

### /zj_humanoid/upperlimb/go_home/whole_body

**Description**: 全身回到home点

**Type**: `upperlimb/ArmType`

**Note**: 全身指定部位回到内置设置的home点

**Demos**: left_arm_disenable_teachmode.yaml

---

### /zj_humanoid/upperlimb/movej/dual_arm

**Description**: 双臂movej

**Type**: `upperlimb/MoveJ`

**Note**: 关节空间下,双臂点到点运动

**Demos**: dual_arm_v_acc_case1.yaml, dual_arm_t_case1.yaml

---

### /zj_humanoid/upperlimb/movej/left_arm

**Description**: 左臂movej

**Type**: `upperlimb/MoveJ`

**Note**: 关节空间下,左臂点到点运动

**Demos**: left_arm_v_acc_case1.yaml, left_arm_t_case1.yaml

---

### /zj_humanoid/upperlimb/movej/lift

**Description**: 升降movej

**Type**: `upperlimb/MoveJ`

**Note**: 关节空间下,升降点到点运动

**Demos**: lift_v_acc_case1.yaml, lift_t_case1.yaml

---

### /zj_humanoid/upperlimb/movej/neck

**Description**: 脖子movej

**Type**: `upperlimb/MoveJ`

**Note**: 关节空间下,脖子点到点运动

**Demos**: neck_v_acc_case1.yaml, neck_t_case1.yaml

---

### /zj_humanoid/upperlimb/movej/right_arm

**Description**: 右臂movej

**Type**: `upperlimb/MoveJ`

**Note**: 关节空间下,右臂点到点运动

**Demos**: right_arm_v_acc_case1.yaml, right_arm_t_case1.yaml

---

### /zj_humanoid/upperlimb/movej/waist

**Description**: 腰部movej

**Type**: `upperlimb/MoveJ`

**Note**: 关节空间下,腰部点到点运动

**Demos**: waist_v_acc_case1.yaml, waist_t_case1.yaml

---

### /zj_humanoid/upperlimb/movej/whole_body

**Description**: 全身movej

**Type**: `upperlimb/MoveJ`

**Note**: 关节空间下,全身各部位点到点运动

**Demos**: whole_body_v_acc_case1.yaml, whole_body_t_case1.yaml

---

### /zj_humanoid/upperlimb/movej_by_path/dual_arm

**Description**: 双臂轨迹movej

**Type**: `upperlimb/MoveJByPath`

**Note**: 关节空间下,双臂轨迹点路径运动

**Demos**: case1.yaml

---

### /zj_humanoid/upperlimb/movej_by_path/left_arm

**Description**: 左臂轨迹movej

**Type**: `upperlimb/MoveJByPath`

**Note**: 关节空间下,左臂轨迹点路径运动

**Demos**: case1.yaml

---

### /zj_humanoid/upperlimb/movej_by_path/right_arm

**Description**: 右臂轨迹movej

**Type**: `upperlimb/MoveJByPath`

**Note**: 关节空间下,右臂轨迹点路径运动

**Demos**: case1.yaml

---

### /zj_humanoid/upperlimb/movej_by_pose/dual_arm

**Description**: 双臂末端movej

**Type**: `upperlimb/MoveJByPose`

**Note**: tcp末端空间下,双臂末端位姿movej

**Demos**: case1.yaml

---

### /zj_humanoid/upperlimb/movej_by_pose/left_arm

**Description**: 左臂末端movej

**Type**: `upperlimb/MoveJByPose`

**Note**: tcp末端空间下,左臂末端位姿movej

**Demos**: case1.yaml

---

### /zj_humanoid/upperlimb/movej_by_pose/right_arm

**Description**: 右臂末端movej

**Type**: `upperlimb/MoveJByPose`

**Note**: tcp末端空间下,右臂末端位姿movej

**Demos**: case1.yaml

---

### /zj_humanoid/upperlimb/movel/dual_arm

**Description**: 双臂movel

**Type**: `upperlimb/MoveL`

**Note**: 关节空间下,双臂直线轨迹点运动

**Demos**: case1.yaml

---

### /zj_humanoid/upperlimb/movel/left_arm

**Description**: 左臂movel

**Type**: `upperlimb/MoveL`

**Note**: 关节空间下,左臂直线轨迹点运动

**Demos**: case1.yaml

---

### /zj_humanoid/upperlimb/movel/right_arm

**Description**: 右臂movel

**Type**: `upperlimb/MoveL`

**Note**: 关节空间下,右臂直线轨迹点运动

**Demos**: case1.yaml

---

### /zj_humanoid/upperlimb/servoj/clear_params

**Description**: 退出servoj

**Type**: `upperlimb/Servo`

**Note**: 退出笛卡尔空间 高频位置跟随控制

---

### /zj_humanoid/upperlimb/servoj/set_params

**Description**: 设置servoj参数

**Type**: `upperlimb/Servo`

**Note**: 设置关节空间 高频位置跟随控制参数

---

### /zj_humanoid/upperlimb/servol/clear_params

**Description**: 退出servol

**Type**: `upperlimb/Servo`

**Note**: 退出笛卡尔空间 高频位置跟随控制

---

### /zj_humanoid/upperlimb/servol/set_params

**Description**: 设置servol参数

**Type**: `upperlimb/Servo`

**Note**: 设置笛卡尔空间 高频位置跟随控制参数

---

### /zj_humanoid/upperlimb/speedl/enable_speedl

**Description**: 启用speedl

**Type**: `std_srvs/SetBool`

**Note**: 启用笛卡尔空间 速度控制

---

### /zj_humanoid/upperlimb/stop_robot

**Description**: 停止上肢运动

**Type**: `std_srvs/Trigger`

**Note**: 停止机器人运动

**Demos**: data.yaml

---

### /zj_humanoid/upperlimb/teach_mode/enter

**Description**: 进入示教模式

**Type**: `upperlimb/ArmType`

**Demos**: left_arm_enable_teachmode.yaml

---

### /zj_humanoid/upperlimb/teach_mode/exit

**Description**: 退出示教模式

**Type**: `upperlimb/ArmType`

**Demos**: left_arm_disenable_teachmode.yaml

---

### /zj_humanoid/upperlimb/versions

**Description**: 上肢模块版本

**Type**: `std_srvs/Trigger`

**Note**: 查询当前上肢子系统的软件版本号 应回复软件版本号

---

## Topics

### /zj_humanoid/audio/asr_text

**Description**: 语音转文字

**Type**: `std_msgs/String`

**Direction**: publish

**Note**: 当前机器人听到了什么

---

### /zj_humanoid/audio/audio_data

**Description**: 音频流数据

**Type**: `audio/AudioData`

**Direction**: publish

**Throttle Rate**: 10 Hz

**Note**: 麦克风收音后的音频数据流

---

### /zj_humanoid/audio/listen_state

**Description**: 唤醒倾听状态

**Type**: `audio/ListenInfo`

**Direction**: publish

**Throttle Rate**: 10 Hz

**Note**: 当前是否为倾听状态

---

### /zj_humanoid/hand/finger_pressures/left

**Description**: 左手压力传感器

**Type**: `hand/PressureSensor`

**Direction**: publish

**Note**: 当前左手压力传感器数值

---

### /zj_humanoid/hand/finger_pressures/right

**Description**: 右手压力传感器数据

**Type**: `hand/PressureSensor`

**Direction**: publish

**Note**: 当前右手压力传感器数值

---

### /zj_humanoid/hand/joint_states

**Description**: 手部关节状态

**Type**: `sensor_msgs/JointState`

**Direction**: publish

**Throttle Rate**: 10 Hz

**Note**: 当前左手食指的角度是多少 应回复0-80度之间

---

### /zj_humanoid/hand/wrist_force_sensor/left

**Description**: 右手腕部传感器值

**Type**: `geometry_msgs/WrenchStamped`

**Direction**: publish

**Throttle Rate**: 10 Hz

**Note**: 当前左手腕部的检测到多少力 应回复0牛顿

---

### /zj_humanoid/hand/wrist_force_sensor/right

**Description**: 左手腕部传感器值

**Type**: `geometry_msgs/WrenchStamped`

**Direction**: publish

**Throttle Rate**: 10 Hz

**Note**: 当前右手腕部的检测到多少力 应回复0牛顿

---

### /zj_humanoid/lowerlimb/body_imu

**Description**: 腰部imu值

**Type**: `sensor_msgs/JointState`

**Direction**: publish

**Throttle Rate**: 10 Hz

---

### /zj_humanoid/lowerlimb/cmd_vel/calib

**Description**: 导航控制行走

**Type**: `geometry_msgs/Twist`

**Direction**: subscribe

**Throttle Rate**: 100 Hz

---

### /zj_humanoid/lowerlimb/cmd_vel/joy

**Description**: 游戏手柄控制行走

**Type**: `geometry_msgs/Twist`

**Direction**: subscribe

**Throttle Rate**: 100 Hz

---

### /zj_humanoid/lowerlimb/cmd_vel/web

**Description**: 网页控制行走

**Type**: `geometry_msgs/Twist`

**Direction**: subscribe

**Throttle Rate**: 100 Hz

---

### /zj_humanoid/lowerlimb/debug_info

**Description**: 运控debug信息

**Type**: `std_msgs/String`

**Direction**: publish

**Throttle Rate**: 1000 Hz

**Note**: 运控debug信息

---

### /zj_humanoid/lowerlimb/occupancy_state

**Description**: 上肢模式控制

**Type**: `std_msgs/Float32`

**Direction**: publish

**Note**: 上肢模式控制，可设置为下肢控制模式

---

### /zj_humanoid/lowerlimb/set_lie

**Description**: 下肢泄力

**Type**: `std_msgs/Float32`

**Direction**: subscribe

**Note**: 下肢泄力，软急停

---

### /zj_humanoid/lowerlimb/set_stand

**Description**: 站立姿态

**Type**: `std_msgs/Float32`

**Direction**: subscribe

**Note**: 站立姿态初始化

---

### /zj_humanoid/lowerlimb/start_move

**Description**: 开启运动模式

**Type**: `std_msgs/Float32`

**Direction**: subscribe

**Note**: 开启运动模式，算法开始响应速度控制请求

---

### /zj_humanoid/navigation/local_map

**Description**: 局部障碍物信息

**Type**: `navigation/LocalMap`

**Direction**: subscribe

**Throttle Rate**: 100 Hz

---

### /zj_humanoid/navigation/map

**Description**: 全局地图信息

**Type**: `nav_msgs/OccupancyGrid`

**Direction**: subscribe

**Note**: 全局地图信息

---

### /zj_humanoid/navigation/navigation_status

**Description**: 当前导航状态

**Type**: `navigation/NavigationStatus`

**Direction**: subscribe

**Throttle Rate**: 50 Hz

**Note**: 当前导航状态信息

---

### /zj_humanoid/navigation/odom_info

**Description**: 当前位姿信息

**Type**: `nav_msgs/Odometry`

**Direction**: subscribe

**Throttle Rate**: 100 Hz

**Note**: 当前位姿信息，有定位时才会输出结果

---

### /zj_humanoid/navigation/task_info

**Description**: 发布导航任务

**Type**: `navigation/TakInfo`

**Direction**: publish

**Throttle Rate**: 1000 Hz

**Note**: 任务信息，该话题仅发布导航任务，不返回导航的结果

**Demos**: rostopic_pub_zj_humanoid_navigation_task_info.yaml

---

### /zj_humanoid/robot/battery_info

**Description**: 电池相关信息

**Type**: `zj_robot/BatteryInfo`

**Direction**: publish

**Throttle Rate**: 1000 Hz

**Note**: 电池BMS相关信息，机器人当前电量还剩多少 回复值应为1~100%

**Demos**: battery_info.yaml

---

### /zj_humanoid/robot/joint_motor/errors

**Description**: 关节电机错误信息

**Type**: `sensor_msgs/JointState`

**Direction**: publish

**Throttle Rate**: 1000 Hz

**Note**: 机器人关节电机错误信息，机器人关节是否有错误发生 回复应包含：没有

---

### /zj_humanoid/robot/joint_motor/temperatures

**Description**: 关节电机温度信息

**Type**: `sensor_msgs/JointState`

**Direction**: publish

**Throttle Rate**: 1000 Hz

**Note**: 关节电机温度信息，当前机器人膝关节温度是多少 回复应介于10-80度之间

---

### /zj_humanoid/robot/monitor

**Description**: 运行状态检测

**Type**: `zj_robot/ModulesMonitor`

**Direction**: publish

**Throttle Rate**: 1000 Hz

**Note**: 机器人内部软件和算法模块运行状态检测, 包含上肢，灵巧手，遥控器，下肢，四目相机，深度相机，定位模块，导航模块，语音模块等

**Demos**: robot_modules_monitor.yaml

---

### /zj_humanoid/robot/orin_states/errors

**Description**: orin错误汇总

**Type**: `zj_robot/Errors`

**Direction**: publish

**Throttle Rate**: 1000 Hz

**Note**: 机器人大脑orin错误汇总，包括over_temp,over_cpu,over_mem,over_disk等，机器人大脑模块是否有错误发生 回复应包含：没有

**Demos**: over_temp_err.yaml

---

### /zj_humanoid/robot/orin_states/resource

**Description**: orin资源统计

**Type**: `zj_robot/Resource`

**Direction**: publish

**Throttle Rate**: 1000 Hz

**Note**: 机器人大脑的资源状态 回复应包含：大脑的cpu,温度，内存，硬盘的用量

**Demos**: get_orin_resouce.yaml

---

### /zj_humanoid/robot/pico_states/errors

**Description**: pico错误汇总

**Type**: `zj_robot/Errors`

**Direction**: publish

**Throttle Rate**: 1000 Hz

**Note**: 机器人小脑pico错误汇总，包括over_temp,over_cpu,over_mem,over_disk等，机器人小脑模块是否有错误发生 回复应包含：没有

**Demos**: over_temp_err.yaml

---

### /zj_humanoid/robot/pico_states/resource

**Description**: pico资源统计

**Type**: `zj_robot/Resource`

**Direction**: publish

**Note**: 机器人小脑pico资源状态 回复应包含：小脑的cpu,温度，内存，硬盘的用量

**Demos**: get_pico_resouce.yaml

---

### /zj_humanoid/robot/robot_state

**Description**: 机器人状态机值

**Type**: `zj_robot/RobotState`

**Direction**: publish

**Throttle Rate**: 1000 Hz

**Note**: 机器人状态机值实时发布，只有当机器人进入RUN状态，机器人才能进行动作的执行，机器人当前处于什么状态 回复应包含：RUN状态

---

### /zj_humanoid/robot/work_status_form_start

**Description**: 工作状态

**Type**: `zj_robot/WorkStatus`

**Direction**: publish

**Throttle Rate**: 1000 Hz

**Note**: 机器人开机后单次工作状态发布，包含已运行时间，剩余工作时间，行进里程数等，描述下机器人本次开机后工作状态 回复因包含：已运行时间，剩余工作时间，行进里程数

**Demos**: robot_work_status.yaml

---

### /zj_humanoid/sensor/CAM_A/compressed

**Description**: 左眼相机JPG

**Type**: `sensor_msgs/CompressedImage`

**Direction**: publish

**Throttle Rate**: 62.5 Hz

**Note**: 左眼相机的JPG图像数据

---

### /zj_humanoid/sensor/CAM_A/image_raw

**Description**: 左眼相机RGB

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 62.5 Hz

**Note**: 左眼相机的RGB图像源数据

---

### /zj_humanoid/sensor/CAM_B/compressed

**Description**: 右眼相机JPG

**Type**: `sensor_msgs/CompressedImage`

**Direction**: publish

**Throttle Rate**: 62.5 Hz

**Note**: 右眼相机的JPG图像数据

---

### /zj_humanoid/sensor/CAM_B/image_raw

**Description**: 右眼相机RGB

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 62.5 Hz

**Note**: 右眼相机的RGB图像源数据

---

### /zj_humanoid/sensor/CAM_C/compressed

**Description**: 相机C的JPG

**Type**: `sensor_msgs/CompressedImage`

**Direction**: publish

**Throttle Rate**: 62.5 Hz

**Note**: 右侧太阳穴相机C的JPG图像数据

---

### /zj_humanoid/sensor/CAM_C/image_raw

**Description**: 相机C的RGB

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 62.5 Hz

**Note**: 右侧太阳穴相机C的RGB图像源数据

---

### /zj_humanoid/sensor/CAM_D/compressed

**Description**: 相机D的JPG

**Type**: `sensor_msgs/CompressedImage`

**Direction**: publish

**Throttle Rate**: 62.5 Hz

**Note**: 左侧太阳穴相机D的JPG图像数据

---

### /zj_humanoid/sensor/CAM_D/image_raw

**Description**: 相机D的RGB

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 62.5 Hz

**Note**: 左侧太阳穴相机D的RGB图像源数据

---

### /zj_humanoid/sensor/head_imu

**Description**: 头部IMU数据

**Type**: `sensor_msgs/Imu`

**Direction**: publish

**Throttle Rate**: 10 Hz

**Note**: 头部IMU的目前帧率是多少 回复应接近100

---

### /zj_humanoid/sensor/realsense_down/aligned_depth_to_color/camera_info

**Description**: 腹部深度aligned参数

**Type**: `sensor_msgs/CameraInfo`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 腹部深度相机的aligned_depth_to_color参数信息

---

### /zj_humanoid/sensor/realsense_down/aligned_depth_to_color/image_raw

**Description**: 腹部深度aligned图像

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 腹部深度相机的aligned_depth_to_color RGB图像源数据

---

### /zj_humanoid/sensor/realsense_down/aligned_depth_to_color/image_raw/compressed

**Description**: 腹部深度aligned压缩图

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 腹部深度相机的aligned_depth_to_color压缩格式

---

### /zj_humanoid/sensor/realsense_down/color/camera_info

**Description**: 腹部深度RGB参数

**Type**: `sensor_msgs/CameraInfo`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 腹部深度相机的参数信息

---

### /zj_humanoid/sensor/realsense_down/color/image_raw

**Description**: 腹部深度RGB图像

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 腹部深度相机的RGB图像源数据

---

### /zj_humanoid/sensor/realsense_down/color/image_raw/compressed

**Description**: 腹部深度压缩图

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 腹部深度相机的RGB图像JPG格式

---

### /zj_humanoid/sensor/realsense_down/depth/camera_info

**Description**: 腹部相机深度参数

**Type**: `sensor_msgs/CameraInfo`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 腹部深度相机的参数信息

---

### /zj_humanoid/sensor/realsense_down/depth/image_rect_raw

**Description**: 腹部深度RGB图像

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 腹部深度相机的RGB图像源数据

---

### /zj_humanoid/sensor/realsense_down/depth/image_rect_raw/compressed

**Description**: 腹部深度压缩图

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 腹部深度相机的RGB图像JPG格式

---

### /zj_humanoid/sensor/realsense_up/aligned_depth_to_color/camera_info

**Description**: 胸部深度aligned参数

**Type**: `sensor_msgs/CameraInfo`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 胸部深度相机的aligned_depth_to_color参数信息

---

### /zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw

**Description**: 胸部深度aligned图像

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 胸部深度相机的aligned_depth_to_color RGB图像源数据

---

### /zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw/compressed

**Description**: 胸部深度aligned压缩图

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 胸部深度相机的aligned_depth_to_color压缩格式

---

### /zj_humanoid/sensor/realsense_up/color/camera_info

**Description**: 胸部深度RGB参数

**Type**: `sensor_msgs/CameraInfo`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 胸部深度相机的参数信息

---

### /zj_humanoid/sensor/realsense_up/color/image_raw

**Description**: 胸部深度RGB图像

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 胸部深度相机的RGB图像源数据

---

### /zj_humanoid/sensor/realsense_up/color/image_raw/compressed

**Description**: 胸部深度压缩图

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 胸部深度相机的RGB图像JPG格式

---

### /zj_humanoid/sensor/realsense_up/depth/camera_info

**Description**: 胸部相机深度参数

**Type**: `sensor_msgs/CameraInfo`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 胸部深度相机的参数信息

---

### /zj_humanoid/sensor/realsense_up/depth/image_rect_raw

**Description**: 胸部深度RGB图像

**Type**: `sensor_msgs/Image`

**Direction**: publishs

**Throttle Rate**: 33.33 Hz

**Note**: 胸部深度相机的RGB图像源数据

---

### /zj_humanoid/sensor/realsense_up/depth/image_rect_raw/compressed

**Description**: 胸部深度压缩图

**Type**: `sensor_msgs/Image`

**Direction**: publish

**Throttle Rate**: 33.33 Hz

**Note**: 胸部深度相机的RGB图像JPG格式

---

### /zj_humanoid/upperlimb/cmd_states

**Description**: 上肢运行模式

**Type**: `upperlimb/CmdState`

**Direction**: publish

**Throttle Rate**: 10 Hz

**Note**: 当前上肢运行模式是什么 回复应处于停止状态

---

### /zj_humanoid/upperlimb/joint_states

**Description**: 上肢关节位置状态

**Type**: `sensor_msgs/JointState`

**Direction**: publish

**Throttle Rate**: 10 Hz

**Note**: 机器人上肢关节position状态值发布，查询当前机器人颈部pitch的角度 回复应处于+-42度间

---

### /zj_humanoid/upperlimb/servoj/left_arm

**Description**: 左臂servoj

**Type**: `upperlimb/Joints`

**Direction**: subscribe

**Note**: 关节空间 高频位置控制

---

### /zj_humanoid/upperlimb/servoj/neck

**Description**: 颈部servoj

**Type**: `upperlimb/Joints`

**Direction**: subscribe

**Note**: 关节空间 高频位置控制

---

### /zj_humanoid/upperlimb/servoj/right_arm

**Description**: 右臂servoj

**Type**: `upperlimb/Joints`

**Direction**: subscribe

**Note**: 关节空间 高频位置控制

---

### /zj_humanoid/upperlimb/servoj/waist

**Description**: 腰部servoj

**Type**: `upperlimb/Joints`

**Direction**: subscribe

**Note**: 关节空间 高频位置控制

---

### /zj_humanoid/upperlimb/servoj/whole_body

**Description**: 全身servoj

**Type**: `upperlimb/Joints`

**Direction**: subscribe

**Note**: 关节空间 高频位置控制

---

### /zj_humanoid/upperlimb/servol/dual_arm

**Description**: 双臂servol

**Type**: `geometry_msgs/Pose`

**Direction**: subscribe

**Note**: 笛卡尔空间 高频位置跟随控制

---

### /zj_humanoid/upperlimb/servol/left_arm

**Description**: 左臂servol

**Type**: `geometry_msgs/Pose`

**Direction**: subscribe

**Note**: 笛卡尔空间 高频位置跟随控制

---

### /zj_humanoid/upperlimb/servol/right_arm

**Description**: 右臂servol

**Type**: `geometry_msgs/Pose`

**Direction**: subscribe

**Note**: 笛卡尔空间 高频位置跟随控制

---

### /zj_humanoid/upperlimb/speedj/enable_speedj

**Description**: 启用speedj

**Type**: `upperlimb/TcpSpeed`

**Direction**: subscribe

**Note**: 启用关节空间速度控制

---

### /zj_humanoid/upperlimb/speedj/left_arm

**Description**: 左臂speedj

**Type**: `upperlimb/TcpSpeed`

**Direction**: subscribe

**Throttle Rate**: 10 Hz

**Note**: 关节空间速度控制

---

### /zj_humanoid/upperlimb/speedj/lift

**Description**: 升降speedj

**Type**: `upperlimb/TcpSpeed`

**Direction**: subscribe

**Throttle Rate**: 10 Hz

**Note**: 关节空间速度控制

---

### /zj_humanoid/upperlimb/speedj/neck

**Description**: 脖子speedj

**Type**: `upperlimb/TcpSpeed`

**Direction**: subscribe

**Throttle Rate**: 10 Hz

**Note**: 关节空间速度控制

---

### /zj_humanoid/upperlimb/speedj/right_arm

**Description**: 右臂speedj

**Type**: `upperlimb/TcpSpeed`

**Direction**: subscribe

**Throttle Rate**: 100 Hz

**Note**: 关节空间速度控制

---

### /zj_humanoid/upperlimb/speedj/waist

**Description**: 腰speedj

**Type**: `upperlimb/TcpSpeed`

**Direction**: subscribe

**Throttle Rate**: 10 Hz

**Note**: 关节空间速度控制

---

### /zj_humanoid/upperlimb/speedj/whole_body

**Description**: 全身speedj

**Type**: `upperlimb/TcpSpeed`

**Direction**: subscribe

**Throttle Rate**: 10 Hz

**Note**: 关节空间速度控制

---

### /zj_humanoid/upperlimb/speedl/dual_arm

**Description**: 双臂speedl

**Type**: `upperlimb/SpeedL`

**Direction**: subscribe

**Note**: 笛卡尔空间 速度控制

---

### /zj_humanoid/upperlimb/speedl/left_arm

**Description**: 左臂speedl

**Type**: `upperlimb/SpeedL`

**Direction**: subscribe

**Note**: 笛卡尔空间 速度控制

---

### /zj_humanoid/upperlimb/speedl/right_arm

**Description**: 右臂speedl

**Type**: `upperlimb/SpeedL`

**Direction**: subscribe

**Note**: 笛卡尔空间 速度控制

---

### /zj_humanoid/upperlimb/tcp_pose/left_arm

**Description**: 左臂tcp位姿控制

**Type**: `upperlimb/Pose`

**Direction**: publish

**Throttle Rate**: 10 Hz

**Note**: 左手臂末端位姿

---

### /zj_humanoid/upperlimb/tcp_pose/right_arm

**Description**: 右臂tcp位姿控制

**Type**: `upperlimb/Pose`

**Direction**: publish

**Throttle Rate**: 10 Hz

**Note**: 右手臂末端位姿

---

### /zj_humanoid/upperlimb/tcp_speed

**Description**: 双臂tcp速度控制

**Type**: `upperlimb/TcpSpeed`

**Direction**: publish

**Throttle Rate**: 10 Hz

**Note**: 左右手臂末端速度

---

