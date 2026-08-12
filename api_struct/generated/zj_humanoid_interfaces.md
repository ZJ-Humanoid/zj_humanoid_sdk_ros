# ZJ Humanoid ROS API 接口文档

**Description**: 浙江人形机器人 Navi 系列 ROS1 API
**Version**: v1.5.0
**Generated At**: 2026-08-12 17:34:53

## Services

Total: 137 services in 10 subsystems

---

## 📦 AUDIO (9 services)

### audio.1. /zj_humanoid/audio/microphone/get_devices_list

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/microphone/get_devices_list` |
| **Type** | `audio/GetDeviceList` |
| **Description** | 麦克风列表 |
| **Note** | 检查当前有多少个麦克风设备 回复数量应大于1 |

### audio.2. /zj_humanoid/audio/microphone/select_device

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/microphone/select_device` |
| **Type** | `audio/SetDevice` |
| **Description** | 选中麦克风 |
| **Note** | 选择第一个麦克风 |

### audio.3. /zj_humanoid/audio/speaker/get_devices_list

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/speaker/get_devices_list` |
| **Type** | `audio/GetDeviceList` |
| **Description** | 获取播放设备 |
| **Note** | 检查当前有多少个喇叭设备 回复数量应大于1 |

### audio.4. /zj_humanoid/audio/speaker/get_volume

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/speaker/get_volume` |
| **Type** | `audio/GetVolume` |
| **Description** | 获取当前音量 |
| **Note** | 获取当前的系统音量大小 应回复音量0~100 |

### audio.5. /zj_humanoid/audio/speaker/select_device

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/speaker/select_device` |
| **Type** | `audio/SetDevice` |
| **Description** | 选中生效喇叭 |
| **Note** | 选择第一个喇叭 |

### audio.6. /zj_humanoid/audio/speaker/set_volume

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/speaker/set_volume` |
| **Type** | `audio/SetVolume` |
| **Description** | 设置音量大小 |
| **Note** | 设置音量为50 |

### audio.7. /zj_humanoid/audio/speaker/stop

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/speaker/stop` |
| **Type** | `audio/SpeakerStop` |
| **Description** | 停止当前播放并清空播放队列 |
| **Note** | 新一轮 TTS 播放前可调用此服务打断旧队列 |

### audio.8. /zj_humanoid/audio/tts_service

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/tts_service` |
| **Type** | `audio/TTS` |
| **Description** | 文字转语音 |
| **Note** | 请让机器人说'hello world' |

### audio.9. /zj_humanoid/audio/version

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/version` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 语音模块的版本号 |

## 📦 CHASSIS (7 services)

### chassis.1. /zj_humanoid/chassis/agv_charge

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/chassis/agv_charge` |
| **Type** | `chassis_msgs/ChargeControl` |
| **Description** | 底盘AGV充电控制 |
| **Note** | WA1、WA2 通用 |

### chassis.2. /zj_humanoid/chassis/agv_reset

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/chassis/agv_reset` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 底盘AGV复位 |
| **Note** | 底盘AGV复位，恢复到初始状态 |

### chassis.3. /zj_humanoid/chassis/agv_version

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/chassis/agv_version` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 底盘AGV版本 |
| **Note** | 获取底盘AGV模块的版本信息 |

### chassis.4. /zj_humanoid/chassis/get_config

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/chassis/get_config` |
| **Type** | `chassis_msgs/GetChassisConfig` |
| **Description** | 查询底盘配置 |
| **Note** | 按配置键查询，空键的行为以底盘实现为准 |

### chassis.5. /zj_humanoid/chassis/set_config

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/chassis/set_config` |
| **Type** | `chassis_msgs/SetChassisConfig` |
| **Description** | 设置底盘配置 |
| **Note** | 以键值对列表批量设置配置 |

### chassis.6. /zj_humanoid/chassis/soc_keep

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/chassis/soc_keep` |
| **Type** | `chassis_msgs/SOCkeepControl` |
| **Description** | 设置底盘保电参数 |
| **Note** | 可设置上下限、电压和电流 |

### chassis.7. /zj_humanoid/chassis/soft_estop

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/chassis/soft_estop` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 底盘软急停 |
| **Note** | 触发底盘软件急停 |

## 📦 HAND (13 services)

### hand.1. /zj_humanoid/hand/finger_pressures/left/zero

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/finger_pressures/left/zero` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 调用左手指尖压力传感器零位校准服务 |
| **Note** | 对左手指尖压力传感器进行零位校准,清除当前传感器偏置,将当前读数设为零点 |

### hand.2. /zj_humanoid/hand/finger_pressures/right/zero

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/finger_pressures/right/zero` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 调用右手指尖压力传感器零位校准服务 |
| **Note** | 对右手指尖压力传感器进行零位校准,清除当前传感器偏置,将当前读数设为零点 |

### hand.3. /zj_humanoid/hand/gesture_switch/dual

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/gesture_switch/dual` |
| **Type** | `hand/Gesture` |
| **Description** | 双手手势切换 |
| **Note** | 同时控制左右手执行指定手势。gesture_name数组中索引0为左手,索引1为右手。支持的手势包括:RESET、ROCK、ONE、TWO、THREE、FOUR等 |

### hand.4. /zj_humanoid/hand/gesture_switch/left

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/gesture_switch/left` |
| **Type** | `hand/Gesture` |
| **Description** | 左手手势切换 |
| **Note** | 控制左手执行指定手势。支持的手势包括:RESET、ROCK、ONE、TWO、THREE、FOUR等,手势名称大小写不敏感 |

### hand.5. /zj_humanoid/hand/gesture_switch/right

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/gesture_switch/right` |
| **Type** | `hand/Gesture` |
| **Description** | 右手手势切换 |
| **Note** | 控制右手执行指定手势。支持的手势包括:RESET、ROCK、ONE、TWO、THREE、FOUR等,手势名称大小写不敏感 |

### hand.6. /zj_humanoid/hand/joint_switch/dual

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/joint_switch/dual` |
| **Type** | `hand/HandJoint` |
| **Description** | 双手关节控制 |
| **Note** | 同时控制双手各关节运动到指定角度。双手关节数组会被合并为12个元素的数组发送给服务。前6个为左手,后6个为右手 |

### hand.7. /zj_humanoid/hand/joint_switch/left

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/joint_switch/left` |
| **Type** | `hand/HandJoint` |
| **Description** | 左手关节控制 |
| **Note** | 控制左手各关节运动到指定角度。关节角度数组顺序为[拇指弯曲,拇指摆动,食指弯曲,中指弯曲,无名指弯曲,小指弯曲],单位:弧度。关节角度会被限制在安全范围内 |

### hand.8. /zj_humanoid/hand/joint_switch/right

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/joint_switch/right` |
| **Type** | `hand/HandJoint` |
| **Description** | 右手关节控制 |
| **Note** | 控制右手各关节运动到指定角度。关节角度数组顺序为[拇指弯曲,拇指摆动,食指弯曲,中指弯曲,无名指弯曲,小指弯曲],单位:弧度。关节角度会被限制在安全范围内 |

### hand.9. /zj_humanoid/hand/task_switch/left

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/task_switch/left` |
| **Type** | `std_srvs/Bool` |
| **Description** | 左手掌任务控制 |

### hand.10. /zj_humanoid/hand/task_switch/right

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/task_switch/right` |
| **Type** | `std_srvs/Bool` |
| **Description** | 右手掌任务控制 |

### hand.11. /zj_humanoid/hand/version

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/version` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 手部模块版本号 |
| **Note** | 查询手部控制模块的版本信息 |

### hand.12. /zj_humanoid/hand/wrist_force_sensor/left/zero

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/wrist_force_sensor/left/zero` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 调用左手腕力传感器零位校准服务 |
| **Note** | 对左手腕力传感器进行零位校准,清除当前传感器偏置,将当前力和力矩读数设为零点 |

### hand.13. /zj_humanoid/hand/wrist_force_sensor/right/zero

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/wrist_force_sensor/right/zero` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 调用右手腕力传感器零位校准服务 |
| **Note** | 对右手腕力传感器进行零位校准,清除当前传感器偏置,将当前力和力矩读数设为零点 |

## 📦 LOWERLIMB (1 services)

### lowerlimb.1. /zj_humanoid/lowerlimb/version

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/lowerlimb/version` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 下肢模块版本 |

## 📦 MANIPULATION (8 services)

### manipulation.1. /zj_humanoid/manipulation/camera_calibration

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/manipulation/camera_calibration` |
| **Type** | `manipulation/CameraCalibration` |
| **Description** | 相机内外参标定 |
| **Note** | 自动相机内外参标定，外参标定时机器人会执行一段轨迹，拍摄不同角度的照片，从而计算外参 |

### manipulation.2. /zj_humanoid/manipulation/execute_pick_task

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/manipulation/execute_pick_task` |
| **Type** | `manipulation/ExecutePickTask` |
| **Description** | 执行抓取服务 |
| **Note** | 输出物品名称执行抓取服务 |

### manipulation.3. /zj_humanoid/manipulation/grasp_teach_service

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/manipulation/grasp_teach_service` |
| **Type** | `manipulation/GraspTeach` |
| **Description** | 视觉抓取示教 |
| **Note** | 视觉示教抓取，让机器人知道该从什么方位抓取物品 |

### manipulation.4. /zj_humanoid/manipulation/joint_space_trajectory_planner

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/manipulation/joint_space_trajectory_planner` |
| **Type** | `manipulation/JointSpaceTrajPlan` |
| **Description** | 关节空间轨迹规划 |
| **Note** | 节空间轨迹规划，输出关节轨迹，示教模式下记录各个关节数据，据此生成完整的执行轨迹 |

### manipulation.5. /zj_humanoid/manipulation/pose_estimation_service

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/manipulation/pose_estimation_service` |
| **Type** | `manipulation/PoseEst` |
| **Description** | 获取目标位姿 |
| **Note** | 输入图像获取指定物品的6D位姿 |

### manipulation.6. /zj_humanoid/manipulation/pose_space_trajectory_planner

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/manipulation/pose_space_trajectory_planner` |
| **Type** | `manipulation/PoseSpaceTrajPlan` |
| **Description** | 末端轨迹规划 |
| **Note** | 末端空间轨迹规划，示教模式下记录各个末端执行器数据，据此生成完整的执行轨迹 |

### manipulation.7. /zj_humanoid/manipulation/scene_update

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/manipulation/scene_update` |
| **Type** | `manipulation/SceneUpdate` |
| **Description** | 场景更新 |
| **Note** | 机器人场景更新,基于二维码，需要场景中有二维码，机器人抓取物品前的环境感知 |

### manipulation.8. /zj_humanoid/manipulation/version

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/manipulation/version` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 操作模块版本号 |

## 📦 NAVIGATION (5 services)

### navigation.1. /zj_humanoid/navigation/get_cur_map_info

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/navigation/get_cur_map_info` |
| **Type** | `map_server_msgs/GetCurMapInfo` |
| **Description** | 获取当前地图信息 |

### navigation.2. /zj_humanoid/navigation/get_map_list

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/navigation/get_map_list` |
| **Type** | `map_server_msgs/GetMapList` |
| **Description** | 获取可用地图列表 |

### navigation.3. /zj_humanoid/navigation/map_server_version

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/navigation/map_server_version` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 获取地图管理模块版本 |
| **Note** | 1.5 新增 |

### navigation.4. /zj_humanoid/navigation/set_map

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/navigation/set_map` |
| **Type** | `map_server_msgs/SetMap` |
| **Description** | 切换当前地图 |

### navigation.5. /zj_humanoid/navigation/version

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/navigation/version` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 定位导航版本号 |

## 📦 PERCEPTION (7 services)

### perception.1. /perception/mapping_service

| Field | Value |
|-------|-------|
| **Service Name** | `/perception/mapping_service` |
| **Type** | `naviai_localization_msgs/Mapping` |
| **Description** | 开始建图兼容接口 |
| **Note** | 兼容旧版调用；1.5 推荐使用 /zj_humanoid/perception/start_mapping |

### perception.2. /perception/post_processing

| Field | Value |
|-------|-------|
| **Service Name** | `/perception/post_processing` |
| **Type** | `naviai_localization_msgs/Post_processing` |
| **Description** | 结束建图兼容接口 |
| **Note** | 兼容旧版调用；1.5 推荐使用 finish_mapping Action |

### perception.3. /zj_humanoid/perception/location_version

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/perception/location_version` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 获取定位模块版本 |

### perception.4. /zj_humanoid/perception/mapping_version

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/perception/mapping_version` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 获取建图模块版本 |

### perception.5. /zj_humanoid/perception/perception_version

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/perception/perception_version` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 获取感知模块版本 |

### perception.6. /zj_humanoid/perception/reloc

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/perception/reloc` |
| **Type** | `naviai_localization_msgs/Lio` |
| **Description** | 触发地图重定位 |
| **Note** | 支持按地图名称、ID或路径发起重定位 |

### perception.7. /zj_humanoid/perception/start_mapping

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/perception/start_mapping` |
| **Type** | `naviai_localization_msgs/Mapping` |
| **Description** | 开始建图 |
| **Note** | 可设置地图名、高度范围、分辨率和场景类型 |

## 📦 ROBOT (13 services)

### robot.1. /zj_humanoid/robot/basic_info

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/basic_info` |
| **Type** | `zj_robot/BasicInfo` |
| **Description** | 机器人基础信息 |
| **Note** | 描述下机器人的基础信息 回复应包含机器人的型号，硬件版本号，软件版本号，IP地址 |

### robot.2. /zj_humanoid/robot/face_show/media_play

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/face_show/media_play` |
| **Type** | `zj_robot/FaceShow` |
| **Description** | 脸部显示视频 |
| **Note** | 机器人脸部屏幕显示,播放视频或图像文件，如播放“Hello_World.mp4” |

### robot.3. /zj_humanoid/robot/face_show/text_show

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/face_show/text_show` |
| **Type** | `zj_robot/FaceScreen` |
| **Description** | 脸部显示文字 |
| **Note** | 机器人脸部屏幕显示文字，支持指令显示“Hello World” |

### robot.4. /zj_humanoid/robot/hardware/set_led

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/hardware/set_led` |
| **Type** | `zj_robot/SetLED` |
| **Description** | 设置 WA2 左右灯带和前灯状态 |
| **Note** | 1.5 新增；由 robot_manger 统一持有底层 XDDP 连接 |

### robot.5. /zj_humanoid/robot/joint_motor/set_zero

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/joint_motor/set_zero` |
| **Type** | `zj_robot/SetZero` |
| **Description** | 电机自动标零 |
| **Note** | 机器人关节自动标零 |

### robot.6. /zj_humanoid/robot/orin_states/connect_wifi

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/orin_states/connect_wifi` |
| **Type** | `zj_robot/ConnectWifi` |
| **Description** | orin连接wifi |
| **Note** | 尝试让机器人大脑orin去连接wifi热点 |

### robot.7. /zj_humanoid/robot/orin_states/wifi_list

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/orin_states/wifi_list` |
| **Type** | `zj_robot/WifiList` |
| **Description** | orin_wifi列表 |
| **Note** | 获取机器人大脑检测到的wifi热点名称，当前机器人大脑检测到多少个wifi信号 回复应大于1 |

### robot.8. /zj_humanoid/robot/pico_states/connect_wifi

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/pico_states/connect_wifi` |
| **Type** | `zj_robot/ConnectWifi` |
| **Description** | pico连接wifi |
| **Note** | 尝试让机器人小脑pico去连接wifi热点 |

### robot.9. /zj_humanoid/robot/pico_states/wifi_list

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/pico_states/wifi_list` |
| **Type** | `zj_robot/WifiList` |
| **Description** | pico_wifi列表 |
| **Note** | 获取机器人小脑检测到的wifi热点名称，当前机器人小脑检测到多少个wifi信号 回复应大于1 |

### robot.10. /zj_humanoid/robot/set_robot_state/OFF

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/set_robot_state/OFF` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 机器人关机 |
| **Note** | 将机器人关机 3秒后，大小脑关机，之后后没法检测到机器人建立ros链接 |

### robot.11. /zj_humanoid/robot/set_robot_state/restart

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/set_robot_state/restart` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 状态机重启 |
| **Note** | 机器人先进stop软急停状态，再自动变为RUN启动运行，在某些故障状态可以执行，但如果有异常的存在，也可能会失败，将机器人状态机重启 持续检测robot_state话题，经过最长60秒钟的等待，状态应切换为：RUN |

### robot.12. /zj_humanoid/robot/set_robot_state/run

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/set_robot_state/run` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 状态机运行 |
| **Note** | 如果机器人处于非RUN状态，尝试将机器人状态值设置为RUN，但如果有异常的存在，也可能会失败，将机器人状态设置为RUN 持续检测robot_state话题，经过最长60秒钟的等待，状态应切换为：RUN |

### robot.13. /zj_humanoid/robot/set_robot_state/stop

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/set_robot_state/stop` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 机器人软急停 |
| **Note** | 机器人软急停状态，状态机值将切换为ERR，在机器发生异常时使用，将机器人状态设置为stop 1秒后，检测robot_state话题，状态应切换为：ERR |

## 📦 SENSOR (4 services)

### sensor.1. /zj_humanoid/sensor/CAM_A/camera_info

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/sensor/CAM_A/camera_info` |
| **Type** | `sensor/CameraInfo` |
| **Description** | 左眼参数信息 |
| **Note** | 相机A的参数信息,相机A安装在机器人左眼的位置上，相机A的分辨率是多少 回复应包含1280和720 |

### sensor.2. /zj_humanoid/sensor/CAM_B/camera_info

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/sensor/CAM_B/camera_info` |
| **Type** | `sensor/CameraInfo` |
| **Description** | 右眼参数信息 |
| **Note** | 相机B的参数信息，相机B安装在机器人右眼的位置上，相机B的分辨率是多少 回复应包含1280和720 |

### sensor.3. /zj_humanoid/sensor/CAM_C/camera_info

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/sensor/CAM_C/camera_info` |
| **Type** | `sensor/CameraInfo` |
| **Description** | 相机C参数信息 |
| **Note** | 相机C的参数信息，相机C大致安装在机器人右侧太阳穴的位置上，相机C的分辨率是多少 回复应包含1280和720 |

### sensor.4. /zj_humanoid/sensor/CAM_D/camera_info

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/sensor/CAM_D/camera_info` |
| **Type** | `sensor/CameraInfo` |
| **Description** | 相机D参数信息 |
| **Note** | 相机D的参数信息，相机D大致安装在机器人左侧太阳穴的位置上，相机D的分辨率是多少 回复应包含1280和720 |

## 📦 UPPERLIMB (70 services)

### upperlimb.1. /zj_humanoid/upperlimb/CartesianPlanningParamsGet

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/CartesianPlanningParamsGet` |
| **Type** | `upperlimb/CartesianPlanningParamsGet` |
| **Description** | 获取笛卡尔规划参数 |
| **Note** | 1.4 新增 |

### upperlimb.2. /zj_humanoid/upperlimb/CartesianPlanningParamsSet

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/CartesianPlanningParamsSet` |
| **Type** | `upperlimb/CartesianPlanningParamsSet` |
| **Description** | 设置笛卡尔规划参数 |
| **Note** | 1.4 新增 |

### upperlimb.3. /zj_humanoid/upperlimb/FK/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/FK/left_arm` |
| **Type** | `upperlimb/FK` |
| **Description** | 左臂正解 |

### upperlimb.4. /zj_humanoid/upperlimb/FK/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/FK/right_arm` |
| **Type** | `upperlimb/FK` |
| **Description** | 右臂正解 |

### upperlimb.5. /zj_humanoid/upperlimb/FK/whole_body

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/FK/whole_body` |
| **Type** | `upperlimb/FK` |
| **Description** | 全身运动学正解 |
| **Note** | 1.4 新增；2.0.0 响应增加 arm_type 和 arm_type_array |

### upperlimb.6. /zj_humanoid/upperlimb/IK/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/IK/left_arm` |
| **Type** | `upperlimb/IK` |
| **Description** | 左臂逆解 |
| **Note** | 左臂逆解 |

### upperlimb.7. /zj_humanoid/upperlimb/IK/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/IK/right_arm` |
| **Type** | `upperlimb/IK` |
| **Description** | 右臂逆解 |

### upperlimb.8. /zj_humanoid/upperlimb/IK/whole_body

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/IK/whole_body` |
| **Type** | `upperlimb/IK` |
| **Description** | 全身运动学逆解 |
| **Note** | 双臂求解时 pose 数组需包含两个目标位姿 |

### upperlimb.9. /zj_humanoid/upperlimb/basic_info

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/basic_info` |
| **Type** | `upperlimb/BasicInfoGet` |
| **Description** | 获取当前机型上肢部位与关节信息 |
| **Note** | 上肢 2.0.0 新增，返回可用部位掩码、关节总数和部位列表 |

### upperlimb.10. /zj_humanoid/upperlimb/clear_servo_params

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/clear_servo_params` |
| **Type** | `upperlimb/Servo` |
| **Description** | 清除伺服参数 |
| **Note** | 清除上肢伺服参数配置 |

### upperlimb.11. /zj_humanoid/upperlimb/collision_detection/enable

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/collision_detection/enable` |
| **Type** | `std_srvs/SetBool` |
| **Description** | 启用或停用碰撞保护 |
| **Note** | 1.3 新增 |

### upperlimb.12. /zj_humanoid/upperlimb/collision_detection/get_params

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/collision_detection/get_params` |
| **Type** | `upperlimb/ParamsGet` |
| **Description** | 获取碰撞保护参数 |

### upperlimb.13. /zj_humanoid/upperlimb/collision_detection/set_params

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/collision_detection/set_params` |
| **Type** | `upperlimb/ParamsSet` |
| **Description** | 设置碰撞保护参数 |

### upperlimb.14. /zj_humanoid/upperlimb/enable_speedj

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/enable_speedj` |
| **Type** | `std_srvs/SetBool` |
| **Description** | 关节速度控制开关 |
| **Note** | 启用或禁用上肢speedj速度控制模式 |

### upperlimb.15. /zj_humanoid/upperlimb/enable_speedl

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/enable_speedl` |
| **Type** | `std_srvs/SetBool` |
| **Description** | 启用笛卡尔空间速度控制 |
| **Note** | 启用或禁用上肢speedl笛卡尔空间速度控制模式 |

### upperlimb.16. /zj_humanoid/upperlimb/go_down/dual_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_down/dual_arm` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 双臂放下 |

### upperlimb.17. /zj_humanoid/upperlimb/go_down/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_down/left_arm` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 左臂放下 |

### upperlimb.18. /zj_humanoid/upperlimb/go_down/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_down/right_arm` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 右臂放下 |

### upperlimb.19. /zj_humanoid/upperlimb/go_home/dual_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_home/dual_arm` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 双臂回到home点 |
| **Note** | 双臂回到内置设置的home点 |

### upperlimb.20. /zj_humanoid/upperlimb/go_home/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_home/left_arm` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 左臂回到home点 |
| **Note** | 左臂回到内置设置的home点 |

### upperlimb.21. /zj_humanoid/upperlimb/go_home/lifting

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_home/lifting` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 升降回到home点 |
| **Note** | 升降回到内置设置的home点 |

### upperlimb.22. /zj_humanoid/upperlimb/go_home/neck

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_home/neck` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 脖子回到home点 |
| **Note** | 脖子回到内置设置的home点 |

### upperlimb.23. /zj_humanoid/upperlimb/go_home/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_home/right_arm` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 右臂回到home点 |
| **Note** | 右臂回到内置设置的home点 |

### upperlimb.24. /zj_humanoid/upperlimb/go_home/waist

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_home/waist` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 腰部回到home点 |
| **Note** | 腰部回到内置设置的home点 |

### upperlimb.25. /zj_humanoid/upperlimb/go_home/whole_body

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_home/whole_body` |
| **Type** | `upperlimb/ArmType` |
| **Description** | 全身回到home点 |
| **Note** | 全身指定部位回到内置设置的home点 |

### upperlimb.26. /zj_humanoid/upperlimb/is_singular/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/is_singular/left_arm` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 检查左臂奇异点 |
| **Note** | 检查左臂当前位置是否处于奇异点配置 |

### upperlimb.27. /zj_humanoid/upperlimb/is_singular/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/is_singular/right_arm` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 检查右臂奇异点 |
| **Note** | 检查右臂当前位置是否处于奇异点配置 |

### upperlimb.28. /zj_humanoid/upperlimb/is_singular/whole_body

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/is_singular/whole_body` |
| **Type** | `upperlimb/IsSingular` |
| **Description** | 检查全身目标位姿是否奇异 |
| **Note** | 1.4 新增 |

### upperlimb.29. /zj_humanoid/upperlimb/jacobian

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/jacobian` |
| **Type** | `upperlimb/JacobianMatrixGet` |
| **Description** | 获取雅可比矩阵 |
| **Note** | 通过请求中的 arm_type 选择左臂、右臂或全身 |

### upperlimb.30. /zj_humanoid/upperlimb/motion/lists

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/motion/lists` |
| **Type** | `upperlimb/FilesLists` |
| **Description** | 获取动作列表 |
| **Note** | 获取可用的预定义动作列表 |

### upperlimb.31. /zj_humanoid/upperlimb/motion/load

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/motion/load` |
| **Type** | `upperlimb/MotionsManage` |
| **Description** | 加载动作文件 |
| **Note** | 从文件加载预定义动作到内存 |

### upperlimb.32. /zj_humanoid/upperlimb/motion/loaded_lists

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/motion/loaded_lists` |
| **Type** | `upperlimb/MotionsLoadedList` |
| **Description** | 获取已加载动作列表 |
| **Note** | 获取当前已加载到内存的动作列表 |

### upperlimb.33. /zj_humanoid/upperlimb/motion/unload

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/motion/unload` |
| **Type** | `upperlimb/MotionsManage` |
| **Description** | 卸载动作文件 |
| **Note** | 从内存中卸载指定的预定义动作 |

### upperlimb.34. /zj_humanoid/upperlimb/movej/dual_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej/dual_arm` |
| **Type** | `upperlimb/MoveJ` |
| **Description** | 双臂movej |
| **Note** | 关节空间下,双臂点到点运动 |

### upperlimb.35. /zj_humanoid/upperlimb/movej/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej/left_arm` |
| **Type** | `upperlimb/MoveJ` |
| **Description** | 左臂movej |
| **Note** | 关节空间下,左臂点到点运动 |

### upperlimb.36. /zj_humanoid/upperlimb/movej/lifting

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej/lifting` |
| **Type** | `upperlimb/MoveJ` |
| **Description** | 升降movej |
| **Note** | 关节空间下,升降点到点运动 |

### upperlimb.37. /zj_humanoid/upperlimb/movej/neck

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej/neck` |
| **Type** | `upperlimb/MoveJ` |
| **Description** | 脖子movej |
| **Note** | 关节空间下,脖子点到点运动 |

### upperlimb.38. /zj_humanoid/upperlimb/movej/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej/right_arm` |
| **Type** | `upperlimb/MoveJ` |
| **Description** | 右臂movej |
| **Note** | 关节空间下,右臂点到点运动 |

### upperlimb.39. /zj_humanoid/upperlimb/movej/waist

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej/waist` |
| **Type** | `upperlimb/MoveJ` |
| **Description** | 腰部movej |
| **Note** | 关节空间下,腰部点到点运动 |

### upperlimb.40. /zj_humanoid/upperlimb/movej/whole_body

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej/whole_body` |
| **Type** | `upperlimb/MoveJ` |
| **Description** | 全身movej |
| **Note** | 关节空间下,全身各部位点到点运动 |

### upperlimb.41. /zj_humanoid/upperlimb/movej_by_path/dual_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_path/dual_arm` |
| **Type** | `upperlimb/MoveJByPath` |
| **Description** | 双臂轨迹movej |
| **Note** | 关节空间下,双臂轨迹点路径运动 |

### upperlimb.42. /zj_humanoid/upperlimb/movej_by_path/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_path/left_arm` |
| **Type** | `upperlimb/MoveJByPath` |
| **Description** | 左臂轨迹movej |
| **Note** | 关节空间下,左臂轨迹点路径运动 |

### upperlimb.43. /zj_humanoid/upperlimb/movej_by_path/lifting

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_path/lifting` |
| **Type** | `upperlimb/MoveJByPath` |
| **Description** | 关节空间下,脖子轨迹点路径运动 |
| **Note** | 控制颈部按照关节空间路径运动 |

### upperlimb.44. /zj_humanoid/upperlimb/movej_by_path/neck

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_path/neck` |
| **Type** | `upperlimb/MoveJByPath` |
| **Description** | 关节空间下,脖子轨迹点路径运动 |
| **Note** | 控制颈部按照关节空间路径运动 |

### upperlimb.45. /zj_humanoid/upperlimb/movej_by_path/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_path/right_arm` |
| **Type** | `upperlimb/MoveJByPath` |
| **Description** | 右臂轨迹movej |
| **Note** | 关节空间下,右臂轨迹点路径运动 |

### upperlimb.46. /zj_humanoid/upperlimb/movej_by_path/waist

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_path/waist` |
| **Type** | `upperlimb/MoveJByPath` |
| **Description** | 关节空间下,腰部轨迹点路径运动 |
| **Note** | 控制腰部按照关节空间路径运动 |

### upperlimb.47. /zj_humanoid/upperlimb/movej_by_path/whole_body

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_path/whole_body` |
| **Type** | `upperlimb/MoveJByPath` |
| **Description** | 关节空间下,全身轨迹点路径运动 |
| **Note** | 控制全身按照关节空间路径运动 |

### upperlimb.48. /zj_humanoid/upperlimb/movej_by_pose/dual_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_pose/dual_arm` |
| **Type** | `upperlimb/MoveJByPose` |
| **Description** | 双臂末端movej |
| **Note** | tcp末端空间下,双臂末端位姿movej |

### upperlimb.49. /zj_humanoid/upperlimb/movej_by_pose/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_pose/left_arm` |
| **Type** | `upperlimb/MoveJByPose` |
| **Description** | 左臂末端movej |
| **Note** | tcp末端空间下,左臂末端位姿movej |

### upperlimb.50. /zj_humanoid/upperlimb/movej_by_pose/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_pose/right_arm` |
| **Type** | `upperlimb/MoveJByPose` |
| **Description** | 右臂末端movej |
| **Note** | tcp末端空间下,右臂末端位姿movej |

### upperlimb.51. /zj_humanoid/upperlimb/movej_by_pose/whole_body

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_pose/whole_body` |
| **Type** | `upperlimb/MoveJByPose` |
| **Description** | 通过目标位姿进行全身关节空间规划 |
| **Note** | 1.4 新增 |

### upperlimb.52. /zj_humanoid/upperlimb/movel/dual_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movel/dual_arm` |
| **Type** | `upperlimb/MoveL` |
| **Description** | 双臂movel |
| **Note** | 关节空间下,双臂直线轨迹点运动 |

### upperlimb.53. /zj_humanoid/upperlimb/movel/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movel/left_arm` |
| **Type** | `upperlimb/MoveL` |
| **Description** | 左臂movel |
| **Note** | 关节空间下,左臂直线轨迹点运动 |

### upperlimb.54. /zj_humanoid/upperlimb/movel/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movel/right_arm` |
| **Type** | `upperlimb/MoveL` |
| **Description** | 右臂movel |
| **Note** | 关节空间下,右臂直线轨迹点运动 |

### upperlimb.55. /zj_humanoid/upperlimb/movel/whole_body

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movel/whole_body` |
| **Type** | `upperlimb/MoveL` |
| **Description** | 全身笛卡尔直线点运动 |
| **Note** | 1.4 新增 |

### upperlimb.56. /zj_humanoid/upperlimb/movel_by_path/dual_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movel_by_path/dual_arm` |
| **Type** | `upperlimb/MoveJByPath` |
| **Description** | 双臂轨迹movej |
| **Note** | 关节空间下,双臂轨迹点路径运动 |

### upperlimb.57. /zj_humanoid/upperlimb/movel_by_path/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movel_by_path/left_arm` |
| **Type** | `upperlimb/MoveJByPath` |
| **Description** | 左臂轨迹movej |
| **Note** | 关节空间下,左臂轨迹点路径运动 |

### upperlimb.58. /zj_humanoid/upperlimb/movel_by_path/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movel_by_path/right_arm` |
| **Type** | `upperlimb/MoveJByPath` |
| **Description** | 右臂轨迹movej |
| **Note** | 关节空间下,右臂轨迹点路径运动 |

### upperlimb.59. /zj_humanoid/upperlimb/movel_by_path/whole_body

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movel_by_path/whole_body` |
| **Type** | `upperlimb/MoveLByPath` |
| **Description** | 全身笛卡尔直线路径运动 |
| **Note** | 1.4 新增 |

### upperlimb.60. /zj_humanoid/upperlimb/payload/get

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/payload/get` |
| **Type** | `upperlimb/PayLoadGet` |
| **Description** | 获取末端负载 |

### upperlimb.61. /zj_humanoid/upperlimb/payload/set

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/payload/set` |
| **Type** | `upperlimb/PayLoadSet` |
| **Description** | 设置末端负载 |

### upperlimb.62. /zj_humanoid/upperlimb/safety_lock

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/safety_lock` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 安全锁定 |
| **Note** | 启用上肢安全锁定,防止意外运动 |

### upperlimb.63. /zj_humanoid/upperlimb/set_servo_params

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/set_servo_params` |
| **Type** | `upperlimb/Servo` |
| **Description** | 设置伺服参数 |
| **Note** | 设置上肢伺服参数配置,包括时间和增益参数 |

### upperlimb.64. /zj_humanoid/upperlimb/stop

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/stop` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 停止上肢运动 |
| **Note** | 立即停止上肢所有运动 |

### upperlimb.65. /zj_humanoid/upperlimb/teach_mode/enter

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/teach_mode/enter` |
| **Type** | `upperlimb/ArmType` |
| **Description** | 进入示教模式 |

### upperlimb.66. /zj_humanoid/upperlimb/teach_mode/exit

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/teach_mode/exit` |
| **Type** | `upperlimb/ArmType` |
| **Description** | 退出示教模式 |

### upperlimb.67. /zj_humanoid/upperlimb/teach_mode/lists

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/teach_mode/lists` |
| **Type** | `upperlimb/FilesLists` |
| **Description** | 获取拖动示教轨迹列表 |

### upperlimb.68. /zj_humanoid/upperlimb/teach_mode/stop_record

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/teach_mode/stop_record` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 停止记录拖动示教轨迹 |

### upperlimb.69. /zj_humanoid/upperlimb/unlock

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/unlock` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 解除锁定 |
| **Note** | 解除上肢安全锁定,允许运动控制 |

### upperlimb.70. /zj_humanoid/upperlimb/version

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/version` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 上肢版本信息 |
| **Note** | 查询上肢控制模块的版本信息 |

## Topics

Total: 126 topics in 9 subsystems

---

## 📡 AUDIO (11 topics)

### audio.1. /zj_humanoid/audio/asr_text

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/audio/asr_text` |
| **Type** | `std_msgs/String` |
| **Direction** | 📤 Publish |
| **Description** | 语音转文字 |
| **Note** | 当前机器人听到了什么 |

### audio.2. /zj_humanoid/audio/listen

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/audio/listen` |
| **Type** | `std_msgs/Bool` |
| **Direction** | 📥 Subscribe |
| **Description** | 唤醒控制 |
| **Note** | 手动唤醒/关闭唤醒模式，true=唤醒，false=休眠 |

### audio.3. /zj_humanoid/audio/listen_state

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/audio/listen_state` |
| **Type** | `std_msgs/Bool` |
| **Direction** | 📤 Publish |
| **Description** | 唤醒倾听状态 |
| **Note** | 当前是否为倾听状态，true=正在倾听，false=未倾听 |

### audio.4. /zj_humanoid/audio/microphone/audio_data_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/audio/microphone/audio_data_raw` |
| **Type** | `audio/AudioData` |
| **Direction** | 📤 Publish |
| **Description** | 麦克风原始 PCM 音频流 |
| **Note** | 采样率和声道数以消息字段为准 |

### audio.5. /zj_humanoid/audio/microphone/status

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/audio/microphone/status` |
| **Type** | `audio/MicStatus` |
| **Direction** | 📤 Publish |
| **Description** | 麦克风设备与采集状态 |
| **Note** | 默认约 1 Hz 发布，设备切换和错误发生时会及时更新 |

### audio.6. /zj_humanoid/audio/microphone/wake_data

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/audio/microphone/wake_data` |
| **Type** | `audio/AudioData` |
| **Direction** | 📤 Publish |
| **Description** | 音频流数据 |
| **Note** | 麦克风收音后的音频数据流 |

### audio.7. /zj_humanoid/audio/microphone/wake_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/audio/microphone/wake_info` |
| **Type** | `audio/AudioData` |
| **Direction** | 📤 Publish |
| **Description** | 音频流数据 |
| **Note** | 麦克风收音后的音频数据流 |

### audio.8. /zj_humanoid/audio/speaker/pcm_play

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/audio/speaker/pcm_play` |
| **Type** | `audio/AudioData` |
| **Direction** | 📥 Subscribe |
| **Description** | PCM 流式播放输入 |
| **Note** | 输入为 S16_LE PCM 数据 |

### audio.9. /zj_humanoid/audio/speaker/playback_status

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/audio/speaker/playback_status` |
| **Type** | `audio/PlaybackStatus` |
| **Direction** | 📤 Publish |
| **Description** | 播放状态变化通知 |
| **Note** | 状态包含 IDLE、PLAYING 和 STOPPED |

### audio.10. /zj_humanoid/audio/speaker/ref_data

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/audio/speaker/ref_data` |
| **Type** | `audio/AudioData` |
| **Direction** | 📤 Publish |
| **Description** | 扬声器软件播放参考数据 |
| **Note** | 用于 AEC 辅助处理 |

### audio.11. /zj_humanoid/audio/speaker/status

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/audio/speaker/status` |
| **Type** | `audio/SpeakerStatus` |
| **Direction** | 📤 Publish |
| **Description** | 扬声器设备与播放状态 |
| **Note** | 默认约 1 Hz 发布 |

## 📡 CHASSIS (10 topics)

### chassis.1. /zj_humanoid/chassis/agv_imu

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/chassis/agv_imu` |
| **Type** | `sensor_msgs/Imu` |
| **Direction** | 📤 Publish |
| **Description** | 底盘IMU数据 |
| **Note** | 底盘AGV的IMU传感器数据,包含角速度、加速度和姿态信息 |

### chassis.2. /zj_humanoid/chassis/agv_state

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/chassis/agv_state` |
| **Type** | `chassis_msgs/AGVNewState` |
| **Direction** | 📤 Publish |
| **Description** | 底盘AGV整体状态 |
| **Note** | 包含运行状态、急停、碰撞和电池信息 |

### chassis.3. /zj_humanoid/chassis/charge_state

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/chassis/charge_state` |
| **Type** | `chassis_msgs/PowerStatusStamped` |
| **Direction** | 📤 Publish |
| **Description** | 手充连接状态 |
| **Note** | WA1 专属 |

### chassis.4. /zj_humanoid/chassis/dido_state

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/chassis/dido_state` |
| **Type** | `chassis_msgs/DIDOState` |
| **Direction** | 📤 Publish |
| **Description** | 数字输入输出与关机状态 |
| **Note** | WA1、WA2 通用 |

### chassis.5. /zj_humanoid/chassis/laser_scan

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/chassis/laser_scan` |
| **Type** | `sensor_msgs/LaserScan` |
| **Direction** | 📤 Publish |
| **Description** | 底盘二维激光数据 |
| **Note** | WA1 专属 |

### chassis.6. /zj_humanoid/chassis/motor_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/chassis/motor_info` |
| **Type** | `chassis_msgs/MotorInfo` |
| **Direction** | 📤 Publish |
| **Description** | 底盘电机状态信息 |
| **Note** | 包含各电机的转速、位置、电流、连接状态和错误码 |

### chassis.7. /zj_humanoid/chassis/odom_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/chassis/odom_info` |
| **Type** | `nav_msgs/Odometry` |
| **Direction** | 📤 Publish |
| **Description** | 底盘里程计信息 |
| **Note** | 底盘里程计数据,包含位置、速度等信息 |

### chassis.8. /zj_humanoid/chassis/steer_command

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/chassis/steer_command` |
| **Type** | `chassis_msgs/SteerCommand` |
| **Direction** | 📥 Subscribe |
| **Description** | 底盘转向控制指令 |
| **Note** | 发送到底盘的转向控制指令，包含目标角度和速度 |

### chassis.9. /zj_humanoid/chassis/steer_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/chassis/steer_info` |
| **Type** | `chassis_msgs/SteerInfo` |
| **Direction** | 📤 Publish |
| **Description** | 底盘转向状态信息 |
| **Note** | 底盘AGV的转向状态信息，包含当前转向角度、速度和目标位置 |

### chassis.10. /zj_humanoid/cmd_vel/calib

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/cmd_vel/calib` |
| **Type** | `geometry_msgs/Twist` |
| **Direction** | 📥 Subscribe |
| **Description** | 底盘速度控制输入 |
| **Note** | WA1、WA2 通用 |

## 📡 HAND (5 topics)

### hand.1. /zj_humanoid/hand/finger_pressures/left

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/hand/finger_pressures/left` |
| **Type** | `hand/PressureSensor` |
| **Direction** | 📤 Publish |
| **Description** | 左手指尖压力 |
| **Note** | 接收左手指尖压力传感器数据,压力值顺序为[大拇指,食指,中指,无名指,小拇指],单位为0.1N |

### hand.2. /zj_humanoid/hand/finger_pressures/right

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/hand/finger_pressures/right` |
| **Type** | `hand/PressureSensor` |
| **Direction** | 📤 Publish |
| **Description** | 右手指尖压力 |
| **Note** | 接收右手指尖压力传感器数据,压力值顺序为[大拇指,食指,中指,无名指,小拇指],单位为0.1N |

### hand.3. /zj_humanoid/hand/joint_states

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/hand/joint_states` |
| **Type** | `sensor_msgs/JointState` |
| **Direction** | 📤 Publish |
| **Description** | 手部关节状态 |
| **Note** | 订阅手部所有关节的位置状态,包括左右手各6个关节:拇指弯曲、拇指摆动、食指弯曲、中指弯曲、无名指弯曲、小指弯曲 |

### hand.4. /zj_humanoid/hand/wrist_force_sensor/left

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/hand/wrist_force_sensor/left` |
| **Type** | `geometry_msgs/WrenchStamped` |
| **Direction** | 📤 Publish |
| **Description** | 左手腕力传感器 |
| **Note** | 发布左手腕力传感器数据,包括力和力矩的三轴分量 |

### hand.5. /zj_humanoid/hand/wrist_force_sensor/right

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/hand/wrist_force_sensor/right` |
| **Type** | `geometry_msgs/WrenchStamped` |
| **Direction** | 📤 Publish |
| **Description** | 右手腕力传感器 |
| **Note** | 接收右手腕力传感器数据,包括力和力矩的三轴分量 |

## 📡 LOWERLIMB (11 topics)

### lowerlimb.1. /zj_humanoid/lowerlimb/body_imu

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/body_imu` |
| **Type** | `sensor_msgs/JointState` |
| **Direction** | 📤 Publish |
| **Description** | 腰部imu值 |
| **Note** | 在双足I2机器人中，IMU位于URDF中的base_link，轮臂机器人目前暂不适用该topic |

### lowerlimb.2. /zj_humanoid/lowerlimb/cmd_vel/calib

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/cmd_vel/calib` |
| **Type** | `geometry_msgs/Twist` |
| **Direction** | 📥 Subscribe |
| **Description** | 导航控制行走 |
| **Note** | 在多个cmd_vel topic中，优先级joy > calib > web |

### lowerlimb.3. /zj_humanoid/lowerlimb/cmd_vel/joy

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/cmd_vel/joy` |
| **Type** | `geometry_msgs/Twist` |
| **Direction** | 📥 Subscribe |
| **Description** | 游戏手柄控制行走 |
| **Note** | 在多个cmd_vel topic中，优先级joy > calib > web |

### lowerlimb.4. /zj_humanoid/lowerlimb/cmd_vel/web

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/cmd_vel/web` |
| **Type** | `geometry_msgs/Twist` |
| **Direction** | 📥 Subscribe |
| **Description** | 网页控制行走 |
| **Note** | 在多个cmd_vel topic中，优先级joy > calib > web |

### lowerlimb.5. /zj_humanoid/lowerlimb/debug_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/debug_info` |
| **Type** | `std_msgs/String` |
| **Direction** | 📤 Publish |
| **Description** | 运控debug信息 |
| **Note** | 双足型号运控debug信息，轮臂机器人暂不适用 |

### lowerlimb.6. /zj_humanoid/lowerlimb/motor_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/motor_info` |
| **Type** | `sensor_msgs/JointState` |
| **Direction** | 📤 Publish |
| **Description** | 下肢电机信息 |
| **Note** | 发布下肢各电机的状态信息，包括位置、速度、力矩等 |

### lowerlimb.7. /zj_humanoid/lowerlimb/set_lie

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/set_lie` |
| **Type** | `std_msgs/Float32` |
| **Direction** | 📥 Subscribe |
| **Description** | 下肢泄力 |
| **Note** | 双足型号下肢泄力，软急停，轮臂机器人暂不适用 |

### lowerlimb.8. /zj_humanoid/lowerlimb/set_stand

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/set_stand` |
| **Type** | `std_msgs/Float32` |
| **Direction** | 📥 Subscribe |
| **Description** | 站立姿态 |
| **Note** | 双足机器人站立姿态初始化，轮臂机器人暂不适用 |

### lowerlimb.9. /zj_humanoid/lowerlimb/start_move

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/start_move` |
| **Type** | `std_msgs/Float32` |
| **Direction** | 📥 Subscribe |
| **Description** | 开启运动模式 |
| **Note** | 双足机器人开启运动模式，算法开始响应速度控制请求，轮臂机器人暂不适用 |

### lowerlimb.10. /zj_humanoid/lowerlimb/state

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/state` |
| **Type** | `sensor_msgs/JointState` |
| **Direction** | 📤 Publish |
| **Description** | 下肢状态信息 |
| **Note** | 发布下肢整体状态信息，包括当前姿态、运动状态等 |

### lowerlimb.11. /zj_humanoid/lowerlimb/version

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/version` |
| **Type** | `std_msgs/String` |
| **Direction** | 📤 Publish |
| **Description** | 下肢模块版本信息 |
| **Note** | 发布下肢模块的版本信息，与service版本获取功能一致 |

## 📡 NAVIGATION (7 topics)

### navigation.1. /zj_humanoid/navigation/local_map

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/navigation/local_map` |
| **Type** | `navigation/LocalMap` |
| **Direction** | 📤 Publish |
| **Description** | 局部障碍物信息 |

### navigation.2. /zj_humanoid/navigation/local_map_render

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/navigation/local_map_render` |
| **Type** | `navigation/LocalMap` |
| **Direction** | 📤 Publish |
| **Description** | 前端展示用轻量局部地图 |
| **Note** | v1.5.0 新增，不替代导航使用的 local_map |

### navigation.3. /zj_humanoid/navigation/map

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/navigation/map` |
| **Type** | `nav_msgs/OccupancyGrid` |
| **Direction** | 📤 Publish |
| **Description** | 全局地图信息 |
| **Note** | 地图管理模块发布的当前全局地图 |

### navigation.4. /zj_humanoid/navigation/map_metadata

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/navigation/map_metadata` |
| **Type** | `nav_msgs/MapMetaData` |
| **Direction** | 📤 Publish |
| **Description** | 当前地图元数据 |

### navigation.5. /zj_humanoid/navigation/navigation_code

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/navigation/navigation_code` |
| **Type** | `navigation/ModuleStatus` |
| **Direction** | 📤 Publish |
| **Description** | 导航模块状态与错误码 |

### navigation.6. /zj_humanoid/navigation/navigation_status

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/navigation/navigation_status` |
| **Type** | `navigation/NavigationState` |
| **Direction** | 📤 Publish |
| **Description** | 当前导航状态 |
| **Note** | 当前导航状态信息 |

### navigation.7. /zj_humanoid/navigation/odom_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/navigation/odom_info` |
| **Type** | `nav_msgs/Odometry` |
| **Direction** | 📤 Publish |
| **Description** | 当前位姿信息 |
| **Note** | 当前位姿信息，有定位时才会输出结果 |

## 📡 PERCEPTION (4 topics)

### perception.1. /perception/mapping_result

| Field | Value |
|-------|-------|
| **Topic Name** | `/perception/mapping_result` |
| **Type** | `naviai_localization_msgs/MappingResult` |
| **Direction** | 📤 Publish |
| **Description** | 建图结果 |
| **Note** | 兼容旧版建图结果话题 |

### perception.2. /zj_humanoid/perception/location_code

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/perception/location_code` |
| **Type** | `module_common_msgs/ModuleStatus` |
| **Direction** | 📤 Publish |
| **Description** | 定位模块状态与错误码 |

### perception.3. /zj_humanoid/perception/mapping_code

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/perception/mapping_code` |
| **Type** | `module_common_msgs/ModuleStatus` |
| **Direction** | 📤 Publish |
| **Description** | 建图模块状态与错误码 |

### perception.4. /zj_humanoid/perception/perception_code

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/perception/perception_code` |
| **Type** | `module_common_msgs/ModuleStatus` |
| **Direction** | 📤 Publish |
| **Description** | 感知模块状态与错误码 |

## 📡 ROBOT (10 topics)

### robot.1. /zj_humanoid/robot/battery_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/robot/battery_info` |
| **Type** | `sensor_msgs/BatteryState` |
| **Direction** | 📤 Publish |
| **Description** | 电池相关信息 |
| **Note** | percentage 取值为 0.0-1.0，WA1、WA2 通用 |

### robot.2. /zj_humanoid/robot/joint_motor/errors

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/robot/joint_motor/errors` |
| **Type** | `sensor_msgs/JointState` |
| **Direction** | 📤 Publish |
| **Description** | 关节电机错误信息 |
| **Note** | 机器人关节电机错误信息，机器人关节是否有错误发生 回复应包含：没有 |

### robot.3. /zj_humanoid/robot/joint_motor/temperatures

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/robot/joint_motor/temperatures` |
| **Type** | `sensor_msgs/JointState` |
| **Direction** | 📤 Publish |
| **Description** | 关节电机温度信息 |
| **Note** | 关节电机温度信息，当前机器人膝关节温度是多少 回复应介于10-80度之间 |

### robot.4. /zj_humanoid/robot/monitor

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/robot/monitor` |
| **Type** | `zj_robot/ModulesMonitor` |
| **Direction** | 📤 Publish |
| **Description** | 运行状态检测 |
| **Note** | 机器人内部软件和算法模块运行状态检测, 包含上肢，灵巧手，遥控器，下肢，四目相机，深度相机，定位模块，导航模块，语音模块等 |

### robot.5. /zj_humanoid/robot/orin_states/errors

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/robot/orin_states/errors` |
| **Type** | `zj_robot/Errors` |
| **Direction** | 📤 Publish |
| **Description** | orin错误汇总 |
| **Note** | 机器人大脑orin错误汇总，包括over_temp,over_cpu,over_mem,over_disk等，机器人大脑模块是否有错误发生 回复应包含：没有 |

### robot.6. /zj_humanoid/robot/orin_states/resource

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/robot/orin_states/resource` |
| **Type** | `zj_robot/Resource` |
| **Direction** | 📤 Publish |
| **Description** | orin资源统计 |
| **Note** | 机器人大脑的资源状态 回复应包含：大脑的cpu,温度，内存，硬盘的用量 |

### robot.7. /zj_humanoid/robot/pico_states/errors

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/robot/pico_states/errors` |
| **Type** | `zj_robot/Errors` |
| **Direction** | 📤 Publish |
| **Description** | pico错误汇总 |
| **Note** | 机器人小脑pico错误汇总，包括over_temp,over_cpu,over_mem,over_disk等，机器人小脑模块是否有错误发生 回复应包含：没有 |

### robot.8. /zj_humanoid/robot/pico_states/resource

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/robot/pico_states/resource` |
| **Type** | `zj_robot/Resource` |
| **Direction** | 📤 Publish |
| **Description** | pico资源统计 |
| **Note** | 机器人小脑pico资源状态 回复应包含：小脑的cpu,温度，内存，硬盘的用量 |

### robot.9. /zj_humanoid/robot/robot_state

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/robot/robot_state` |
| **Type** | `zj_robot/RobotState` |
| **Direction** | 📤 Publish |
| **Description** | 机器人状态机值 |
| **Note** | 机器人状态机值实时发布，只有当机器人进入RUN状态，机器人才能进行动作的执行，机器人当前处于什么状态 回复应包含：RUN状态 |

### robot.10. /zj_humanoid/robot/work_status_from_start

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/robot/work_status_from_start` |
| **Type** | `zj_robot/WorkStatus` |
| **Direction** | 📤 Publish |
| **Description** | 工作状态 |
| **Note** | 机器人开机后单次工作状态发布，包含已运行时间，剩余工作时间，行进里程数等，描述下机器人本次开机后工作状态 回复因包含：已运行时间，剩余工作时间，行进里程数 |

## 📡 SENSOR (36 topics)

### sensor.1. /zj_humanoid/sensor/CAM_A/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/CAM_A/compressed` |
| **Type** | `sensor_msgs/CompressedImage` |
| **Direction** | 📤 Publish |
| **Description** | 左眼相机JPG |
| **Note** | 左眼相机的JPG图像数据 |

### sensor.2. /zj_humanoid/sensor/CAM_A/image_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/CAM_A/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 左眼相机RGB |
| **Note** | 左眼相机的RGB图像源数据 |

### sensor.3. /zj_humanoid/sensor/CAM_B/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/CAM_B/compressed` |
| **Type** | `sensor_msgs/CompressedImage` |
| **Direction** | 📤 Publish |
| **Description** | 右眼相机JPG |
| **Note** | 右眼相机的JPG图像数据 |

### sensor.4. /zj_humanoid/sensor/CAM_B/image_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/CAM_B/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 右眼相机RGB |
| **Note** | 右眼相机的RGB图像源数据 |

### sensor.5. /zj_humanoid/sensor/CAM_C/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/CAM_C/compressed` |
| **Type** | `sensor_msgs/CompressedImage` |
| **Direction** | 📤 Publish |
| **Description** | 相机C的JPG |
| **Note** | 右侧太阳穴相机C的JPG图像数据 |

### sensor.6. /zj_humanoid/sensor/CAM_C/image_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/CAM_C/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 相机C的RGB |
| **Note** | 右侧太阳穴相机C的RGB图像源数据 |

### sensor.7. /zj_humanoid/sensor/CAM_D/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/CAM_D/compressed` |
| **Type** | `sensor_msgs/CompressedImage` |
| **Direction** | 📤 Publish |
| **Description** | 相机D的JPG |
| **Note** | 左侧太阳穴相机D的JPG图像数据 |

### sensor.8. /zj_humanoid/sensor/CAM_D/image_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/CAM_D/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 相机D的RGB |
| **Note** | 左侧太阳穴相机D的RGB图像源数据 |

### sensor.9. /zj_humanoid/sensor/head_imu

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/head_imu` |
| **Type** | `sensor_msgs/Imu` |
| **Direction** | 📤 Publish |
| **Description** | 头部IMU数据 |
| **Note** | 头部IMU的目前帧率是多少 回复应接近100 |

### sensor.10. /zj_humanoid/sensor/realsense_down/aligned_depth_to_color/camera_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/aligned_depth_to_color/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度aligned参数 |
| **Note** | 腹部深度相机的aligned_depth_to_color参数信息 |

### sensor.11. /zj_humanoid/sensor/realsense_down/aligned_depth_to_color/image_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/aligned_depth_to_color/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度aligned图像 |
| **Note** | 腹部深度相机的aligned_depth_to_color RGB图像源数据 |

### sensor.12. /zj_humanoid/sensor/realsense_down/aligned_depth_to_color/image_raw/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/aligned_depth_to_color/image_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度aligned压缩图 |
| **Note** | 腹部深度相机的aligned_depth_to_color压缩格式 |

### sensor.13. /zj_humanoid/sensor/realsense_down/color/camera_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/color/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度RGB参数 |
| **Note** | 腹部深度相机的参数信息 |

### sensor.14. /zj_humanoid/sensor/realsense_down/color/image_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/color/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度RGB图像 |
| **Note** | 腹部深度相机的RGB图像源数据 |

### sensor.15. /zj_humanoid/sensor/realsense_down/color/image_raw/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/color/image_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度压缩图 |
| **Note** | 腹部深度相机的RGB图像JPG格式 |

### sensor.16. /zj_humanoid/sensor/realsense_down/depth/camera_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/depth/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 腹部相机深度参数 |
| **Note** | 腹部深度相机的参数信息 |

### sensor.17. /zj_humanoid/sensor/realsense_down/depth/image_rect_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/depth/image_rect_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度RGB图像 |
| **Note** | 腹部深度相机的RGB图像源数据 |

### sensor.18. /zj_humanoid/sensor/realsense_down/depth/image_rect_raw/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_down/depth/image_rect_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 腹部深度压缩图 |
| **Note** | 腹部深度相机的RGB图像JPG格式 |

### sensor.19. /zj_humanoid/sensor/realsense_head/aligned_depth_to_color/camera_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 头部深度aligned参数 |
| **Note** | 头部深度相机的aligned_depth_to_color参数信息 |

### sensor.20. /zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 头部深度aligned图像 |
| **Note** | 头部深度相机的aligned_depth_to_color RGB图像源数据 |

### sensor.21. /zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 头部深度aligned压缩图 |
| **Note** | 头部深度相机的aligned_depth_to_color压缩格式 |

### sensor.22. /zj_humanoid/sensor/realsense_head/color/camera_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_head/color/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 头部深度RGB参数 |
| **Note** | 头部深度相机的参数信息 |

### sensor.23. /zj_humanoid/sensor/realsense_head/color/image_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_head/color/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 头部深度RGB图像 |
| **Note** | 头部深度相机的RGB图像源数据 |

### sensor.24. /zj_humanoid/sensor/realsense_head/color/image_raw/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_head/color/image_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 头部深度压缩图 |
| **Note** | 头部深度相机的RGB图像JPG格式 |

### sensor.25. /zj_humanoid/sensor/realsense_head/depth/camera_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_head/depth/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 头部相机深度参数 |
| **Note** | 头部深度相机的参数信息 |

### sensor.26. /zj_humanoid/sensor/realsense_head/depth/image_rect_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_head/depth/image_rect_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 头部深度RGB图像 |
| **Note** | 头部深度相机的深度图像源数据 |

### sensor.27. /zj_humanoid/sensor/realsense_head/depth/image_rect_raw/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_head/depth/image_rect_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 头部深度压缩图 |
| **Note** | 头部深度相机的深度图像JPG格式 |

### sensor.28. /zj_humanoid/sensor/realsense_up/aligned_depth_to_color/camera_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/aligned_depth_to_color/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度aligned参数 |
| **Note** | 胸部深度相机的aligned_depth_to_color参数信息 |

### sensor.29. /zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度aligned图像 |
| **Note** | 胸部深度相机的aligned_depth_to_color RGB图像源数据 |

### sensor.30. /zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度aligned压缩图 |
| **Note** | 胸部深度相机的aligned_depth_to_color压缩格式 |

### sensor.31. /zj_humanoid/sensor/realsense_up/color/camera_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/color/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度RGB参数 |
| **Note** | 胸部深度相机的参数信息 |

### sensor.32. /zj_humanoid/sensor/realsense_up/color/image_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/color/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度RGB图像 |
| **Note** | 胸部深度相机的RGB图像源数据 |

### sensor.33. /zj_humanoid/sensor/realsense_up/color/image_raw/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/color/image_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度压缩图 |
| **Note** | 胸部深度相机的RGB图像JPG格式 |

### sensor.34. /zj_humanoid/sensor/realsense_up/depth/camera_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/depth/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 胸部相机深度参数 |
| **Note** | 胸部深度相机的参数信息 |

### sensor.35. /zj_humanoid/sensor/realsense_up/depth/image_rect_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/depth/image_rect_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度RGB图像 |
| **Note** | 胸部深度相机的RGB图像源数据 |

### sensor.36. /zj_humanoid/sensor/realsense_up/depth/image_rect_raw/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/depth/image_rect_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度压缩图 |
| **Note** | 胸部深度相机的RGB图像JPG格式 |

## 📡 UPPERLIMB (32 topics)

### upperlimb.1. /zj_humanoid/sim/upperlimb/from_algo

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sim/upperlimb/from_algo` |
| **Type** | `upperlimb/Joints` |
| **Direction** | 📥 Subscribe |
| **Description** | 上肢算法输出仿真 |
| **Note** | 1.3 新增 |

### upperlimb.2. /zj_humanoid/sim/upperlimb/to_algo

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sim/upperlimb/to_algo` |
| **Type** | `upperlimb/Joints` |
| **Direction** | 📤 Publish |
| **Description** | 仿真上肢数据输入算法 |
| **Note** | 1.3 新增 |

### upperlimb.3. /zj_humanoid/upperlimb/jacobian/left_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/jacobian/left_arm` |
| **Type** | `std_msgs/Float64MultiArray` |
| **Direction** | 📤 Publish |
| **Description** | 左臂雅可比矩阵 |
| **Note** | 发布左臂当前位置的雅可比矩阵,用于速度和力的映射 |

### upperlimb.4. /zj_humanoid/upperlimb/jacobian/right_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/jacobian/right_arm` |
| **Type** | `std_msgs/Float64MultiArray` |
| **Direction** | 📤 Publish |
| **Description** | 右臂雅可比矩阵 |
| **Note** | 发布右臂当前位置的雅可比矩阵,用于速度和力的映射 |

### upperlimb.5. /zj_humanoid/upperlimb/joint_states

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/joint_states` |
| **Type** | `sensor_msgs/JointState` |
| **Direction** | 📤 Publish |
| **Description** | 上肢关节位置 |
| **Note** | 机器人上肢关节position状态值发布，查询当前机器人颈部pitch的角度 回复应处于+-42度间 |

### upperlimb.6. /zj_humanoid/upperlimb/occupancy_state

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/occupancy_state` |
| **Type** | `std_msgs/Int8` |
| **Direction** | 📥 Subscribe |
| **Description** | 上肢占用控制 |
| **Note** | 该话题发布上肢的当前占用状态,用于防止多个控制源同时控制机器人 |

### upperlimb.7. /zj_humanoid/upperlimb/servoj/dual_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servoj/dual_arm` |
| **Type** | `upperlimb/Joints` |
| **Direction** | 📤 Publish |
| **Description** | 双臂servoj |
| **Note** | 双臂关节空间伺服控制,不要使用定时sleep,该接口执行需要准确的时间戳会达到更好的效果 |

### upperlimb.8. /zj_humanoid/upperlimb/servoj/left_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servoj/left_arm` |
| **Type** | `upperlimb/Joints` |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂servoj |
| **Note** | 关节空间 高频位置控制 |

### upperlimb.9. /zj_humanoid/upperlimb/servoj/lifting

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servoj/lifting` |
| **Type** | `upperlimb/Joints` |
| **Direction** | 📥 Subscribe |
| **Description** | 全身servoj |
| **Note** | 关节空间 高频位置控制 |

### upperlimb.10. /zj_humanoid/upperlimb/servoj/neck

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servoj/neck` |
| **Type** | `upperlimb/Joints` |
| **Direction** | 📥 Subscribe |
| **Description** | 颈部servoj |
| **Note** | 关节空间 高频位置控制 |

### upperlimb.11. /zj_humanoid/upperlimb/servoj/right_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servoj/right_arm` |
| **Type** | `upperlimb/Joints` |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂servoj |
| **Note** | 关节空间 高频位置控制 |

### upperlimb.12. /zj_humanoid/upperlimb/servoj/waist

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servoj/waist` |
| **Type** | `upperlimb/Joints` |
| **Direction** | 📥 Subscribe |
| **Description** | 腰部servoj |
| **Note** | 关节空间 高频位置控制 |

### upperlimb.13. /zj_humanoid/upperlimb/servoj/whole_body

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servoj/whole_body` |
| **Type** | `upperlimb/Joints` |
| **Direction** | 📥 Subscribe |
| **Description** | 全身servoj |
| **Note** | 关节空间 高频位置控制 |

### upperlimb.14. /zj_humanoid/upperlimb/servol/dual_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servol/dual_arm` |
| **Type** | `geometry_msgs/Pose` |
| **Direction** | 📥 Subscribe |
| **Description** | 双臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### upperlimb.15. /zj_humanoid/upperlimb/servol/left_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servol/left_arm` |
| **Type** | `geometry_msgs/Pose` |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### upperlimb.16. /zj_humanoid/upperlimb/servol/right_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servol/right_arm` |
| **Type** | `geometry_msgs/Pose` |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### upperlimb.17. /zj_humanoid/upperlimb/speedj/dual_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedj/dual_arm` |
| **Type** | `upperlimb/SpeedJ` |
| **Direction** | 📤 Publish |
| **Description** | 双臂关节speedj |
| **Note** | 双臂关节空间速度控制 |

### upperlimb.18. /zj_humanoid/upperlimb/speedj/left_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedj/left_arm` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂speedj |
| **Note** | 关节空间速度控制 |

### upperlimb.19. /zj_humanoid/upperlimb/speedj/lifting

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedj/lifting` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📥 Subscribe |
| **Description** | 升降speedj |
| **Note** | 关节空间速度控制 |

### upperlimb.20. /zj_humanoid/upperlimb/speedj/neck

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedj/neck` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📥 Subscribe |
| **Description** | 脖子speedj |
| **Note** | 关节空间速度控制 |

### upperlimb.21. /zj_humanoid/upperlimb/speedj/right_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedj/right_arm` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂speedj |
| **Note** | 关节空间速度控制 |

### upperlimb.22. /zj_humanoid/upperlimb/speedj/waist

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedj/waist` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📥 Subscribe |
| **Description** | 腰speedj |
| **Note** | 关节空间速度控制 |

### upperlimb.23. /zj_humanoid/upperlimb/speedj/whole_body

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedj/whole_body` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📥 Subscribe |
| **Description** | 全身speedj |
| **Note** | 关节空间速度控制 |

### upperlimb.24. /zj_humanoid/upperlimb/speedl/dual_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedl/dual_arm` |
| **Type** | `upperlimb/SpeedL` |
| **Direction** | 📥 Subscribe |
| **Description** | 双臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### upperlimb.25. /zj_humanoid/upperlimb/speedl/left_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedl/left_arm` |
| **Type** | `upperlimb/SpeedL` |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### upperlimb.26. /zj_humanoid/upperlimb/speedl/right_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedl/right_arm` |
| **Type** | `upperlimb/SpeedL` |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### upperlimb.27. /zj_humanoid/upperlimb/speedl/whole_body

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedl/whole_body` |
| **Type** | `upperlimb/SpeedL` |
| **Direction** | 📥 Subscribe |
| **Description** | 全身笛卡尔速度控制 |
| **Note** | 1.4 新增 |

### upperlimb.28. /zj_humanoid/upperlimb/tcp_pose/left_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/tcp_pose/left_arm` |
| **Type** | `upperlimb/Pose` |
| **Direction** | 📤 Publish |
| **Description** | 左臂tcp位姿控制 |
| **Note** | 左手臂末端位姿 |

### upperlimb.29. /zj_humanoid/upperlimb/tcp_pose/right_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/tcp_pose/right_arm` |
| **Type** | `upperlimb/Pose` |
| **Direction** | 📤 Publish |
| **Description** | 右臂tcp位姿控制 |
| **Note** | 右手臂末端位姿 |

### upperlimb.30. /zj_humanoid/upperlimb/tcp_speed/dual_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/tcp_speed/dual_arm` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📥 Subscribe |
| **Description** | 双臂TCP速度 |
| **Note** | 该话题发布双臂末端执行器(TCP)的实时速度信息 |

### upperlimb.31. /zj_humanoid/upperlimb/uplimb_occupation

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/uplimb_occupation` |
| **Type** | `std_msgs/Int8` |
| **Direction** | 📤 Publish |
| **Description** | 上肢占用状态 |
| **Note** | 用于发布上肢占用状态信息 |

### upperlimb.32. /zj_humanoid/upperlimb/uplimb_state

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/uplimb_state` |
| **Type** | `upperlimb/UplimbState` |
| **Direction** | 📥 Subscribe |
| **Description** | 当前上肢命令 |
| **Note** | 该话题发布上肢机器人的当前命令状态信息 |

## Actions

Total: 8 actions in 5 subsystems

---

## ⚙️ AUDIO (2 actions)

### audio.1. /zj_humanoid/audio/LLM_chat

| Field | Value |
|-------|-------|
| **Action Name** | `/zj_humanoid/audio/LLM_chat` |
| **Type** | `audio/LLMChat` |
| **Description** | 大模型对话 |
| **Note** | 支持上下文、语音输出和流式文本反馈 |

### audio.2. /zj_humanoid/audio/media_play

| Field | Value |
|-------|-------|
| **Action Name** | `/zj_humanoid/audio/media_play` |
| **Type** | `audio/MediaPlay` |
| **Description** | 播放媒体文件 |
| **Note** | 文件播放倍率范围为 0.5-2.0，播放期间可调用 speaker/stop 中断 |

## ⚙️ MANIPULATION (1 actions)

### manipulation.1. /zj_humanoid/manipulation/instance_segmentation_action

| Field | Value |
|-------|-------|
| **Action Name** | `/zj_humanoid/manipulation/instance_segmentation_action` |
| **Type** | `manipulation/InstSeg` |
| **Description** | 实例分割 |

## ⚙️ NAVIGATION (1 actions)

### navigation.1. /zj_humanoid/navigation/navigation

| Field | Value |
|-------|-------|
| **Action Name** | `/zj_humanoid/navigation/navigation` |
| **Type** | `navigation/Navigation` |
| **Description** | 导航接口 |

## ⚙️ PERCEPTION (1 actions)

### perception.1. /zj_humanoid/perception/finish_mapping

| Field | Value |
|-------|-------|
| **Action Name** | `/zj_humanoid/perception/finish_mapping` |
| **Type** | `naviai_localization_msgs/FinishMapping` |
| **Description** | 结束或中止建图 |
| **Note** | method=0 时结束并保存地图，method=1 时中止且不保存；后处理开始后无法被取消 |

## ⚙️ UPPERLIMB (3 actions)

### upperlimb.1. /zj_humanoid/upperlimb/motion

| Field | Value |
|-------|-------|
| **Action Name** | `/zj_humanoid/upperlimb/motion` |
| **Type** | `upperlimb/MotionExecute` |
| **Description** | 执行预加载轨迹 |

### upperlimb.2. /zj_humanoid/upperlimb/teach_mode/replay

| Field | Value |
|-------|-------|
| **Action Name** | `/zj_humanoid/upperlimb/teach_mode/replay` |
| **Type** | `upperlimb/DragTeachReplay` |
| **Description** | 回放拖动示教轨迹 |
| **Note** | 1.3 新增 |

### upperlimb.3. /zj_humanoid/upperlimb/teach_mode/start_record

| Field | Value |
|-------|-------|
| **Action Name** | `/zj_humanoid/upperlimb/teach_mode/start_record` |
| **Type** | `upperlimb/DragTeachRecord` |
| **Description** | 开始记录拖动示教轨迹 |
| **Note** | 1.3 新增 |

---

## Summary

- **Total Services**: 137
- **Total Topics**: 126
- **Total Actions**: 8
- **Total Interfaces**: 271
- **Subsystems**: 10 (services), 9 (topics), 5 (actions)
