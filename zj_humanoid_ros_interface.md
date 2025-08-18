# /zj_humanoid

## /robot

### /robot_state
- decription
  - 机器人状态机值实时发布，只有当机器人进入RUN状态，机器人才能进行动作的执行
- type
  - Topic/Publish
- msg
  - RobotState.msg
- hz
  - 1
- demos
  - robot_state_get_set.py
- agent
  - 机器人当前处于什么状态
    - 回复应包含：RUN状态

### /set_robot_state

#### /run
- decription
  - 如果机器人处于非RUN状态，尝试将机器人状态值设置为RUN，但如果有异常的存在，也可能会失败
- type
  - Service
- srv
  - std_srvs/Trigger
- demos
  - robot_state_get_set.py
- agent
  - 将机器人状态设置为RUN
    - 持续检测robot_state话题，经过最长60秒钟的等待，状态应切换为：RUN

#### /stop
- decription
  - 机器人软急停状态，状态机值将切换为ERR，在机器发生异常时使用
- type
  - Service
- srv
  - std_srvs/Trigger
- demos
  - robot_state_get_set.py
- agent
  - 将机器人状态设置为stop
    - 1秒后，检测robot_state话题，状态应切换为：ERR

#### /OFF
- decription
  - 机器人关机，大小脑将同步关机
- type
  - Service
- srv
  - std_srvs/Trigger
- demos
  - robot_state_get_set.py
- agent
  - 将机器人关机
    - 3秒后，大小脑关机，之后后没法检测到机器人建立ros链接

#### /restart
- decription
  - 机器人先进stop软急停状态，再自动变为RUN启动运行，在某些故障状态可以执行，但如果有异常的存在，也可能会失败
- type
  - Service
- srv
  - std_srvs/Trigger
- demos
  - robot_state_get_set.py
- agent
  - 将机器人状态机重启
    - 持续检测robot_state话题，经过最长60秒钟的等待，状态应切换为：RUN

### /basic_info
- description
  - 机器人基础信息，机器人的型号，硬件版本号，软件版本号，IP地址等
- type
  - Service
- srv
  - Robot_BasicInfo.srv
- demos
  - get_robot_basic_info.py
- agent
  - 描述下机器人的基础信息
    - 回复应包含机器人的型号，硬件版本号，软件版本号，IP地址

### /battery_info
- description
  - 机器人主电池和BMS相关信息
- type
  - Topic/Publish
- msg
  - BatteryInfo.msg
- hz
  - 1
- agent
  - 机器人当前电量还剩多少
    - 回复值应为1~100%

### /orin_states

#### /errors
- description
  - 机器人大脑orin错误汇总，包括over_temp,over_cpu,over_mem,over_disk等
- type
  - Topic/Publish
- msg
- hz
  - 1
- agent
  - 机器人大脑模块是否有错误发生
    - 回复应包含：没有

#### /resource
- description
  - 机器人大脑orin资源统计，包括cpu,temperature,memory,disk等信息
- type
  - Topic/Publish
- msg
- hz
  - 1
- agent
  - 机器人大脑的资源状态
    - 回复应包含：大脑的cpu,温度，内存，硬盘的用量

#### /wifi_list
- description
  - 获取机器人大脑检测到的wifi热点名称
- type
  - Service
- srv
  - WifiList.srv
- demos
  - connect_wifi_orin.py
- agent
  - 当前机器人大脑检测到多少个wifi信号
    - 回复应大于1

#### /connect_wifi
- description
  - 尝试让机器人大脑orin去连接wifi热点
- type
  - Service
- srv
  - ConnectWifi.srv
- demos
  - connect_wifi_orin.py

### /pico_states

#### /errors
- description
  - 小脑pico错误汇总，包含over_temp,over_cpu,over_mem,over_disk等
- type
  - Topic/Publish
- msg
- hz
  - 1
- agent
  - 机器人大脑模块是否有错误发生
    - 回复应包含：没有

#### /resource
- description
  - 小脑pico资源统计，包含cpu,temperature,memory,disk等信息
- type
  - Topic/Publish
- msg
- agent
  - 机器人大脑的资源状态
    - 回复应包含：大脑的cpu,温度，内存，硬盘的用量

#### /wifi_list
- description
  - 获取机器人小脑检测到的wifi热点名称
- type
  - Service
- srv
  - WifiList.srv
- demos
  - connect_wifi_pico.py
- agent
  - 当前机器人小脑检测到多少个wifi信号
    - 回复应大于1

#### /connect_wifi
- description
  - 尝试让机器人大脑orin连接wifi热点
- type
  - Service
- srv
  - ConnectWifi.srv
- demos
  - connect_wifi_pico.py

### /joint_motor

#### /errors
- description
  - 机器人关节电机错误信息
- type
  - Topic/Publish
- msg
  - sensor_msgs/JointState
- hz
  - 1
- agent
  - 机器人关节是否有错误发生
    - 回复应包含：没有

#### /temperatures
- description
  - 机器人关节电机温度信息
- type
  - Topic/Publish
- msg
  - sensor_msgs/JointState
- hz
  - 1
- agent
  - 当前机器人膝关节温度是多少
    - 回复应介于10-80度之间

#### /set_zero
- description
  - 电机自动标零服务
- agent
  - 机器人关节自动标零
- type
- demos

### /work_status_form_start
- description
  - 机器人开机后单次工作状态发布，包含已运行时间，剩余工作时间，行进里程数等
- type
  - Topic/Publish
- msg
  - Robot_WorkStatus.msg
- hz
  - 1
- agent
  - 描述下机器人本次开机后工作状态
    - 回复因包含：已运行时间，剩余工作时间，行进里程数

### /face_show

#### /media_play
- description
  - 机器人脸部屏幕显示,播放视频或图像文件
- type
  - Service
- srv
  - Robot_FaceShow.srv
- demos
  - Robot_VideoPlay.py
- agent
  - 播放“Hello_World.mp4”

#### /text_show
- description
  - 机器人脸部屏幕显示文字
- type
  - Service
- srv
  - Robot_FaceSrceen.srv
- demos
  - Robot_VideoPlay.py
- agent
  - 显示“Hello World”

### /monitor
- description
  - 机器人内部软件和算法模块运行状态检测, 包含上肢，灵巧手，遥控器，下肢，四目相机，深度相机，定位模块，导航模块，语音模块等
- type
  - Topic/Publish
- msg
  - ModulesMonitor.msg
- hz

## /upperlimb

### /versions
- decription
  - 上肢模块版本号信息，包含software_version, hardware_verion等
- type
  - Service
- srv
- demos
- agent
  - 查询当前上肢子系统的软件版本号
    - 应回复软件版本号

### /joint_states
- description
  - 机器人上肢关节position状态值发布
- type
  - Topic/Publish
- msg
  - sensor_msgs/JointState
- hz
  - 100
- agent
  - 查询当前机器人颈部pitch的角度
    - 回复应处于+-42度间

### /cmd_states
- description
  - 上肢当前运行模式
- type
  - Topic/Publish
- msg
  - uplimb_msgs/CmdState.msg
- hz
  - 100
- agent
  - 当前上肢运行模式是什么
    - 回复应处于停止状态

### /go_home

#### /left_arm
- description
  - 回原点（该原点数据为内置设置,不带碰撞检测）
- type
  - Service
- srv
  - std_srvs/Trigger

#### /right_arm
- description
  - 回原点（该原点数据为内置设置）
- type
  - Service
- srv
  - std_srvs/Trigger

#### /dual_arm
- description
  - 回原点（该原点数据为内置设置）
- type
  - Service
- srv
  - std_srvs/Trigger

#### /whole_body
- description
  - 回原点（该原点数据为内置设置）
- type
  - Service
- srv
  - std_srvs/Trigger

#### /set_home

### /teach_mode

#### /enter
- description
  - 进入示教模式
- type
  - Service
- srv
  - uplimb/ArmType

#### /exit
- description
  - 退出示教模式
- type
  - Service
- srv
  - uplimb/ArmType

### /stop_moving
- description
  - 停止双臂运动
- type
  - Service
- srv
  - std_srvs/Trigger
- agent
  - 停止运动

### /tcp_pose
- left_arm
  - description
    - 左手臂末端位姿
  - type
    - Topic/Publish
  - msg
    - geometry_msgs/Pose
  - hz
    - 100
- right_arm
  - description
    - 右手臂末端位姿
  - type
    - Topic/Publish
  - msg
    - uplimb_msgs/Pose.msg
  - hz
    - 100

### /tcp_speed
- description
  - 左右手臂末端速度
- type
  - Topic/Publish
- msg
  - uplimb/TcpSpeed
- hz
  - 100

### /speedj

#### /left_arm
- description
  - 关节空间速度控制
- type
  - Topic/Subscribe
- msg
  - uplimb/TcpSpeed
- hz
  - 100

#### /right_arm
- description
  - 关节空间速度控制
- type
  - Topic/Subscribe
- msg
  - uplimb/TcpSpeed
- hz
  - 10

#### /neck
- description
  - 关节空间速度控制
- type
  - Topic/Subscribe
- msg
  - uplimb/TcpSpeed
- hz
  - 100

#### /waist
- description
  - 关节空间速度控制
- type
  - Topic/Subscribe
- msg
  - uplimb/TcpSpeed
- hz
  - 100

#### /lift
- description
  - 关节空间速度控制
- type
  - Topic/Subscribe
- msg
  - uplimb/TcpSpeed
- hz
  - 100

#### /whole_body
- description
  - 关节空间速度控制
- type
  - Topic/Subscribe
- msg
  - uplimb/TcpSpeed
- hz
  - 100

#### /enable_speedj
- description
  - 启用关节空间速度控制
- type
  - Topic/Subscribe
- msg
  - uplimb/TcpSpeed

### /speedl

#### /left_arm
- description
  - 笛卡尔空间 速度控制
- type
  - Topic/Subscribe
- srv
  - uplimb/SpeedL

#### /right_arm
- description
  - 笛卡尔空间 速度控制
- type
  - Topic/Subscribe
- srv
  - uplimb/SpeedL

#### /dual_arm
- description
  - 笛卡尔空间 速度控制
- type
  - Topic/Subscribe
- srv
  - uplimb/SpeedL

#### /enable_speedl
- description
  - 启用笛卡尔空间 速度控制
- type
  - Service
- srv
  - std_srvs/SetBool

### /servoj

#### /left_arm
- description
  - 关节空间 高频位置控制
- type
  - Topic/Subscribe
- srv
  - uplimb/Joints

#### /right_arm
- description
  - 关节空间 高频位置控制
- type
  - Topic/Subscribe
- srv
  - uplimb/Joints

#### /whole_body
- description
  - 关节空间 高频位置控制
- type
  - Topic/Subscribe
- srv
  - uplimb/Joints

#### /set_params
- description
  - 设置关节空间 高频位置跟随控制参数
- type
  - Service
- srv
  - uplimb/Servo

#### /clear_params
- description
  - 退出笛卡尔空间 高频位置跟随控制
- type
  - Service
- srv
  - uplimb/Servo

### /servol

#### /left_arm
- description
  - 笛卡尔空间 高频位置跟随控制
- type
  - Topic/Subscribe
- msg
  - geometry_msgs/Pose

#### /right_arm
- description
  - 笛卡尔空间 高频位置跟随控制
- type
  - Topic/Subscribe
- msg
  - geometry_msgs/Pose

#### /dual_arm
- description
  - 笛卡尔空间 高频位置跟随控制
- type
  - Topic/Subscribe
- msg
  - geometry_msgs/Pose

#### /set_params
- description
  - 设置笛卡尔空间 高频位置跟随控制参数
- type
  - Service
- srv
  - uplimb/Servo

#### /clear_params
- description
  - 退出笛卡尔空间 高频位置跟随控制
- type
  - Service
- srv
  - uplimb/Servo

### /movej

#### /left_arm
- description
  - 关节轨迹点运动
- type
  - Service
- srv
  - uplimb/MoveJ

#### /right_arm
- description
  - 关节轨迹点运动
- type
  - Service
- srv
  - uplimb/MoveJ

#### /neck
- description
  - 关节轨迹点运动
- type
  - Service
- srv
  - uplimb/MoveJ

#### /waist
- description
  - 关节轨迹点运动
- type
  - Service
- srv
  - uplimb/MoveJ

#### /lift
- description
  - 关节轨迹点运动
- type
  - Service
- srv
  - uplimb/MoveJ

#### /whole_body
- description
  - 关节轨迹点运动
- type
  - Service
- srv
  - uplimb/MoveJ

### /movej_by_pose

#### /left_arm
- description
  - 关节轨迹点运动(通过位姿)
- type
  - Service
- srv
  - uplimb/MoveJByPose

#### /right_arm
- description
  - 关节轨迹点运动(通过位姿)
- type
  - Service
- srv
  - uplimb/MoveJByPose

#### /dual_arm
- description
  - 关节轨迹点运动(通过位姿)
- type
  - Service
- srv
  - uplimb/MoveJByPose

### /movej_by_path

#### /left_arm
- description
  - 关节路径轨迹运动
- type
  - Service
- srv
  - uplimb/MoveJByPath

#### /right_arm
- description
  - 关节路径轨迹运动
- type
  - Service
- srv
  - uplimb/MoveJByPath

#### /dual_arm
- description
  - 关节路径轨迹运动
- type
  - Service
- srv
  - uplimb/MoveJByPath

#### /whole_body
- description
  - 关节路径轨迹运动
- type
  - Service
- srv
  - uplimb/MoveJByPath

### /movel

#### /left_arm
- description
  - 直线轨迹点运动
- type
  - Service
- srv
  - uplimb/MoveL

#### /right_arm
- description
  - 直线轨迹点运动
- type
  - Service
- srv
  - uplimb/MoveL

#### /dual_arm
- description
  - 直线轨迹点运动
- type
  - Service
- srv
  - uplimb/MoveL

### /movel_by_path

#### /left_arm
- description
  - 直线路径轨迹运动
- type
  - Service
- srv
  - uplimb/MoveLByPath

#### /right_arm
- description
  - 直线路径轨迹运动
- type
  - Service
- srv
  - uplimb/MoveLByPath

#### /dual_arm
- description
  - 直线路径轨迹运动
- type
  - Service
- srv
  - uplimb/MoveLByPath

### /IK

#### /left_arm
- description
  - 正运动学求解
- type
  - Service
- srv
  - uplimb/FK

#### /right_arm
- description
  - 正运动学求解
- type
  - Service
- srv
  - uplimb/FK

### /FK

#### /left_arm
- description
  - 正运动学接口
- type
  - Service
- srv
  - uplimb/IK

#### /right_arm
- description
  - 正运动学接口
- type
  - Service
- srv
  - uplimb/IK

## /hand

### /hand_joint_states
- description
  - 手部关节状态
- type
  - Topic/Publish
- msg
  - sensor_msgs/JointState
- hz
  - 100
- agent
  - 当前左手食指的角度是多少
    - 应回复0-80度之间

### /wrist_force_6d

#### /left_hand
- description
  - 右手腕部6维力传感器值
- type
  - Topic/Publish
- msg
  - geometry_msgs/WrenchStamped
- hz
  - 100
- agent
  - 当前左手腕部的检测到多少力
    - 应回复0牛顿

#### /right_hand
- description
  - 左手腕部6维力传感器值
- type
  - Topic/Publish
- msg
  - geometry_msgs/WrenchStamped
- hz
  - 100
- agent
  - 当前右手腕部的检测到多少力
    - 应回复0牛顿

### /finger_pressures
- description
  - 手指压力传感器状态值
- type
  - Topic/Publish
- msg
  - sensor_msgs/JointState
- hz
  - 100
- agent
  - 当前左手食指的检测到多少力
    - 应回复0牛顿

### /gesture_switch

#### /left_hand
- description
  - 左手掌手势控制
- type
  - Service
- srv
- demos
- agent
  - 左手摆出1的手势
    - 订阅/hand_joint_states左手中指数值应大于60

#### /right_hand
- description
  - 右手掌手势控制
- type
  - Service
- srv
- demos
- agent
  - 右手摆出1的手势
    - 订阅/hand_joint_states右手中指数值应大于60

### /task_switch

#### /left_hand
- description
  - 左手掌任务控制
- type
  - Service
- srv
- demos

#### /right_hand
- description
  - 右手掌任务控制
- type
  - Service
- srv
- demos

### /joint_switch

#### /left_hand
- description
  - 左手掌关节控制
- type
  - Service
- srv
- demos
- agent
  - 左手食指弯曲40度
    - 订阅/hand_joint_states左手食指数值应接近40度

#### /right_hand
- description
  - 左手掌关节控制
- type
  - Service
- srv
- demos
- agent
  - 右手食指弯曲40度
    - 订阅/hand_joint_states左手食指数值应接近40度

### /versions
- decription
  - 灵巧手版本号信息
- type
  - Service
- srv
  - software_version, hardware_verion
- demos
- agent
  - 查询当前灵巧手子系统的版本号

## /lowerlimb

### /cmd_vel

#### /joy
- description
  - 游戏手柄控制行走
- type
  - Topic/Subscribe
- msg
  - geometry_msgs/Twist
- hz
  - 10

#### /web
- description
  - 网络控制行走
- type
  - Topic/Subscribe
- msg
  - geometry_msgs/Twist
- hz
  - 10

#### /calib
- description
  - 导航算法控制行走
- type
  - Topic/Subscribe
- msg
  - geometry_msgs/Twist
- hz
  - 10

### /set_stand
- decription
  - 站立姿态
- type
  - Topic/Subscribe

### /set_lie
- decription
  - 下肢泄力
- type
  - Topic/Subscribe

### /start_move
- decription
  - 开启运动模式
- type
  - Topic/Subscribe

### /uplimb_occupation
- decription
  - 上肢控制请求
- type
  - Service

### /versions
- decription
  - 上肢模块版本号信息
- type
  - Service
- srv
  - software_version, hardware_verion
- demos

### /body_imu
- description
  - 腰部imu值
- type
  - Topic/Publish
- msg
  - sensor_msgs/JointState
- hz
  - 100

## /sensor

### /CAM_A

#### /camera_info
- decription
  - 相机A的参数信息
- type
  - Service
- srv
  - Robot_CameraInfo.srv
- demos
- agent
  - 相机A的分辨率是多少
    - 回复应包含1280和720
  - 相机A安装在机器人左眼的位置上

#### /image_raw
- description
  - 相机A的RGB图像源数据
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 16
- agent
  - 相机A的目前帧率是多少
    - 回复应接近16

#### /compressed
- description
  - 相机A的RGB图像源数据
- type
  - Topic/Publish
- msg
  - sensor_msgs/CompressedImage
- hz
  - 16

### /CAM_B

#### /camera_info
- decription
  - 相机B的参数信息
- type
  - Service
- srv
  - Robot_CameraInfo.srv
- demos
- agent
  - 相机B的分辨率是多少
    - 回复应包含1280和720
  - 相机B安装在机器人右眼的位置上

#### /image_raw
- description
  - 相机B的RGB图像源数据
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 16
- agent
  - 相机B的目前帧率是多少
    - 回复应接近16

#### /compressed
- description
  - 相机B的RGB图像源数据
- type
  - Topic/Publish
- msg
  - sensor_msgs/CompressedImage
- hz
  - 16

### /CAM_C

#### /camera_info
- decription
  - 相机C的参数信息
- type
  - Service
- srv
  - Robot_CameraInfo.srv
- demos
- agent
  - 相机C的目前帧率是多少
    - 回复应接近16
  - 相机C大致安装在机器人右侧太阳穴的位置上

#### /image_raw
- description
  - 相机C的RGB图像源数据
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 16
- agent
  - 相机C的目前帧率是多少
    - 回复应接近16

#### /compressed
- description
  - 相机C的RGB图像源数据
- type
  - Topic/Publish
- msg
  - sensor_msgs/CompressedImage
- hz
  - 16

### /CAM_D

#### /camera_info
- decription
  - 相机D的参数信息
- type
  - Service
- srv
  - Robot_CameraInfo.srv
- demos
- agent
  - 相机D的目前帧率是多少
    - 回复应接近16
  - 相机D大致安装在机器人左侧太阳穴的位置上

#### /image_raw
- description
  - 相机D的RGB图像源数据
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 16
- agent
  - 相机D的目前帧率是多少
    - 回复应接近16

#### /compressed
- description
  - 相机D的RGB图像JPG格式
- type
  - Topic/Publish
- msg
  - sensor_msgs/CompressedImage
- hz
  - 16

### /head_imu
- description
  - 头部相机的IMU数据
- type
  - Topic/Publish
- msg
  - sensor_msgs/Imu
- hz
  - 100
- agent
  - 头部IMU的目前帧率是多少
    - 回复应接近100

### /realsense_up

#### /aligned_depth_to_color
- description
  - 和RGB对齐后的深度图

##### /camera_info
- decription
  - 胸部深度相机的参数信息
- type
  - Topic/Publish
- msg
  - sensor_msgs/CameraInfo
- hz
  - 30

##### /image_raw
- description
  - 胸部深度相机的RGB图像源数据
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 30

##### /image_raw

###### /compressed
- description
  - 胸部深度相机的RGB图像JPG格式
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 30

#### /color
- description
  - 深度相机RGB图

##### /camera_info
- decription
  - 胸部深度相机的参数信息
- type
  - Topic/Publish
- msg
  - sensor_msgs/CameraInfo
- hz
  - 30

##### /image_raw
- description
  - 胸部深度相机的RGB图像源数据
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 30

##### /image_raw

###### /compressed
- description
  - 胸部深度相机的RGB图像JPG格式
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 30

#### /depth
- 名称：深度图

##### /camera_info
- decription
  - 胸部深度相机的参数信息
- type
  - Topic/Publish
- msg
  - sensor_msgs/CameraInfo
- hz
  - 30

##### /image_raw
- description
  - 胸部深度相机的RGB图像源数据
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 30

##### /image_raw

###### /compressed
- description
  - 胸部深度相机的RGB图像JPG格式
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 30

### /realsense_down

#### /aligned_depth_to_color
- description
  - 和RGB对齐后的深度图

##### /camera_info
- decription
  - 胸部深度相机的参数信息
- type
  - Topic/Publish
- msg
  - sensor_msgs/CameraInfo
- hz
  - 30

##### /image_raw
- description
  - 胸部深度相机的RGB图像源数据
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 30

##### /image_raw

###### /compressed
- description
  - 胸部深度相机的RGB图像JPG格式
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 30

#### /color
- description
  - 深度相机RGB图

##### /camera_info
- decription
  - 胸部深度相机的参数信息
- type
  - Topic/Publish
- msg
  - sensor_msgs/CameraInfo
- hz
  - 30

##### /image_raw
- description
  - 胸部深度相机的RGB图像源数据
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 30

##### /image_raw

###### /compressed
- description
  - 胸部深度相机的RGB图像JPG格式
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 30

#### /depth
- 名称：深度图

##### /camera_info
- decription
  - 胸部深度相机的参数信息
- type
  - Topic/Publish
- msg
  - sensor_msgs/CameraInfo
- hz
  - 30

##### /image_raw
- description
  - 胸部深度相机的RGB图像源数据
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 30

##### /image_raw

###### /compressed
- description
  - 胸部深度相机的RGB图像JPG格式
- type
  - Topic/Publish
- msg
  - sensor_msgs/Image
- hz
  - 30

### /point_cloud_360
- description

## /manipulate

### /naviai_manip_update_scene_service
- description
  - 机器人场景更新,基于二维码，需要场景中有二维码
- type
  - Service
- srv
  - SceneUpdate.srv
- demos
  - naviai_manip_scene_update_client.py
- agent
  - 机器人抓取物品前的环境感知

### /joint_space_trajectory_planner
- description
  - 关节空间轨迹规划，输出关节轨迹
- type
  - Service
- srv
  - Manip_GetTrajectory.srv
- demos
  - trajectory_plan_client.py
- agent
  - 示教模式下记录各个关节数据，据此生成完整的执行轨迹

### /naviai_manip_motion_plan_service
- description
  - 末端空间轨迹规划
- type
  - Service
- srv
  - MotionPlan.srv
- demos
  - naviai_manip_motion_plan_client.py
- agent
  - 示教模式下记录各个末端执行器数据，据此生成完整的执行轨迹

### /camera_calibration
- description
  - 相机内外参标定
- type
  - Service
- srv
  - Manip_CameraCalibration.srv
- agent
  - 自动相机内外参标定，外参标定时机器人会执行一段轨迹，拍摄不同角度的照片，从而计算外参

### /grasp_teach_service
- description
  - 视觉抓取示教服务
- type
  - Service
- srv
  - GraspTeach.srv
- agent
  - 视觉示教抓取，让机器人知道该从什么方位抓取物品

### /pose_estimation_service
- description
  - 获取目标物体位姿
- type
  - Service
- srv
  - PoseEst.srv
- demos
  - pose_estimator_client.py
- agent
  - 输入图像获取指定物品的6D位姿

### /instance_segmentation_service
- description
  - 实例分割服务
- type
  - Service
- srv
  - InstSeg.srv
- demos
  - seg_pre_service_client.py
- agent
  - 输入图像获取指定物品的实例分割信息

### /execute_pick_task
- description
  - 输出物品名称执行抓取服务
- type
  - Service
- srv
  - Manip_ExecutePickTask.srv
- demos
  - rosservice call /execute_pick_task "target_label: chips_can_orin"
- agent
  - 帮我拿xxx物品
    - 调用该服务执行抓取

## /navigation

### /odom_info
- description
  - 当前位姿信息，有定位时才会输出结果
- type
  - Topic
- msg
  - Odometry
- hz
  - 10
- demos
  - rostopic echo /odom_info
- agent
  - 机器人在当前地图上的具体位姿，若机器人发布导航任务后不动可先订阅这个话题查看定位是否有问题

### /local_map
- description
  - 局部障碍物信息
- type
  - Topic
- msg
  - LocalMap.msg
- hz
  - 10
- demos
  - rostopic echo /odom_info
- agent
  - 机器人周围的障碍物信息，若机器人导航到一半不动可订阅这个话题查看是否遇障，或者通过rviz查看激光点云

### /task_info
- description
  - 发布导航任务信息，该话题仅发布导航任务，不返回导航的结果
- type
  - Topic
- msg
  - TakInfo.msg
- hz
  - 1
- demos
  - point2point.py
- agent
  - 需要机器人实现点对点的导航，可发布该话题，只需要填写x,y,yaw这三个数值即可

### /navigation_status
- description
  - 当前导航状态信息
- type
  - Topic
- msg
  - NavigationStatus.msg
- hz
  - 20
- demos
  - rostopic echo /navigation_status
- agent
  - 0:没有定位，1：导航完成，2：执行中，3：规划中
- globle_map

## /audio

### /microphone

#### /get_devices_list
- description
  - 获取麦克风设备列表
- type
  - Service
- srv
  - GetDeviceList.srv
- demo
- agent
  - 检查当前有多少个麦克风设备
    - 回复数量应大于1

#### /select_device
- description
  - 选中生效麦克风
- type
  - Service
- srv
  - SetDevice
- demo
- agent
  - 选择第一个麦克风

#### /audio_data
- description
  - 麦克风收音后的音频数据流
- msg:
  - AudioDate
- hz

### /speaker

#### /get_devices_list
- description
  - 获取喇叭设备列表
- type
  - Service
- GetDeviceList.srv
- demo
- agent
  - 检查当前有多少个喇叭设备
    - 回复数量应大于1

#### /select_device
- description
  - 选中生效喇叭
- type
  - Service
- srv
  - SetDevice
- demo
- agent
  - 选择第一个喇叭

#### /get_volume
- description
  - 获取当前音量
- type
  - Service
- msg
- agent
  - 获取当前的系统音量大小
    - 应回复音量0~100

#### /set_volume
- description
  - 设置当前音量大小
- type
  - Service
- agent
  - 设置音量为50

### /listen
- description
  - 倾听服务
- type
  - Service
- srv
- agent
  - 开始倾听

### /listen_state
- description
  - 唤醒倾听状态发布
- type
  - Topic/Publish
- hz
  - 100hz
- agent
  - 当前是否为倾听状态

### /asr_text
- description
  - 语音转文字服务
- agent
  - 当前机器人听到了什么
- type
  - Topic/Publish
- msg

### /tts_service
- description
  - 文字转语音服务
- agent
  - 请让机器人说“hello world”
- demo

### /media_play
- description
  - 音频文件播放
- 请播放“公司介绍.mp3”
- type
  - Service
- srv
- demo

### /LLM_chat
- description
  - LLM智能对话服务
- 和机器人发起对话，说“hello world”
- type
  - Service
- srv
- demo
