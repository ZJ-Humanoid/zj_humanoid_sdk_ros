# ZJ Humanoid ROS API 接口文档

**Description**: ZJ Humanoid ROS1 APIs
**Version**: v1.0.0
**Generated At**: 2025-11-12 15:33:05

## Services

Total: 87 services in 8 subsystems

---

## 📦 AUDIO (11 services)

### audio.1. /zj_humanoid/audio/LLM_chat

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/LLM_chat` |
| **Type** | `audio/LLM_chat` |
| **Description** | LLM对话服务 |
| **Note** | 语音模块的版本号是多少 |

### audio.2. /zj_humanoid/audio/listen

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/listen` |
| **Type** | `audio/Listen` |
| **Description** | 倾听服务 |
| **Note** | 开始倾听 |

### audio.3. /zj_humanoid/audio/media_play

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/media_play` |
| **Type** | `audio/MediaPlay` |
| **Description** | 音频文件播放 |
| **Note** | 播放’公司介绍.mp3‘ |

### audio.4. /zj_humanoid/audio/microphone/get_devices_list

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/microphone/get_devices_list` |
| **Type** | `audio/GetDeviceList` |
| **Description** | 麦克风列表 |
| **Note** | 检查当前有多少个麦克风设备 回复数量应大于1 |

### audio.5. /zj_humanoid/audio/microphone/select_device

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/microphone/select_device` |
| **Type** | `audio/SetDevice` |
| **Description** | 选中麦克风 |
| **Note** | 选择第一个麦克风 |

### audio.6. /zj_humanoid/audio/speaker/get_devices_list

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/speaker/get_devices_list` |
| **Type** | `audio/GetDeviceList` |
| **Description** | 获取播放设备 |
| **Note** | 检查当前有多少个喇叭设备 回复数量应大于1 |

### audio.7. /zj_humanoid/audio/speaker/get_volume

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/speaker/get_volume` |
| **Type** | `audio/GetVolume` |
| **Description** | 获取当前音量 |
| **Note** | 获取当前的系统音量大小 应回复音量0~100 |

### audio.8. /zj_humanoid/audio/speaker/select_device

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/speaker/select_device` |
| **Type** | `audio/SetDevice` |
| **Description** | 选中生效喇叭 |
| **Note** | 选择第一个喇叭 |

### audio.9. /zj_humanoid/audio/speaker/set_volume

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/speaker/set_volume` |
| **Type** | `audio/SetVolume` |
| **Description** | 设置音量大小 |
| **Note** | 设置音量为50 |

### audio.10. /zj_humanoid/audio/tts_service

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/tts_service` |
| **Type** | `audio/TTS` |
| **Description** | 文字转语音 |
| **Note** | 请让机器人说‘hello world‘ |

### audio.11. /zj_humanoid/audio/version

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/audio/version` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 语音模块的版本号 |

## 📦 HAND (11 services)

### hand.1. /zj_humanoid/hand/finger_pressures/left/zero

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/finger_pressures/left/zero` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 置零手指传感器 |
| **Note** | 置零压力传感器数值 |

### hand.2. /zj_humanoid/hand/gesture_switch/dual

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/gesture_switch/dual` |
| **Type** | `hand/Gesture` |
| **Description** | 双手手势切换 |

### hand.3. /zj_humanoid/hand/gesture_switch/left

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/gesture_switch/left` |
| **Type** | `hand/Gesture` |
| **Description** | 左手手势切换 |

### hand.4. /zj_humanoid/hand/gesture_switch/right

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/gesture_switch/right` |
| **Type** | `hand/Gesture` |
| **Description** | 右手手势切换 |

### hand.5. /zj_humanoid/hand/joint_switch/dual

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/joint_switch/dual` |
| **Type** | `hand/HandJoint` |
| **Description** | 双手手掌关节运动 |

### hand.6. /zj_humanoid/hand/joint_switch/left

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/joint_switch/left` |
| **Type** | `hand/HandJoint` |
| **Description** | 左手手掌关节运动 |
| **Note** | 左手食指弯曲40度 订阅/hand_joint_states左手食指数值应接近40度 |

### hand.7. /zj_humanoid/hand/joint_switch/right

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/joint_switch/right` |
| **Type** | `hand/HandJoint` |
| **Description** | 右手手掌关节运动 |
| **Note** | 右手食指弯曲40度 订阅/hand_joint_states左手食指数值应接近40度 |

### hand.8. /zj_humanoid/hand/task_switch/left

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/task_switch/left` |
| **Type** | `std_srvs/Bool` |
| **Description** | 左手掌任务控制 |

### hand.9. /zj_humanoid/hand/task_switch/right

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/task_switch/right` |
| **Type** | `std_srvs/Bool` |
| **Description** | 右手掌任务控制 |

### hand.10. /zj_humanoid/hand/versions

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/versions` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 灵巧手版本号 |
| **Note** | 查询当前灵巧手子系统的版本号 |

### hand.11. /zj_humanoid/hand/wrist_force_sensor/left/zero

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/hand/wrist_force_sensor/left/zero` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 置零腕部传感器 |
| **Note** | 置零腕部传感器数值 |

## 📦 LOWERLIMB (1 services)

### lowerlimb.1. /zj_humanoid/lowerlimb/versions

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/lowerlimb/versions` |
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
| **Type** | `manipulation/GetTrajectory` |
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
| **Type** | `manipulation/MotionPlan` |
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
| **Type** | `` |
| **Description** | 操作模块版本号 |

## 📦 NAVIGATION (1 services)

### navigation.1. /zj_humanoid/navigation/version

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/navigation/version` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 定位导航版本号 |

## 📦 ROBOT (12 services)

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
| **Type** | `zj_robot/FaceText` |
| **Description** | 脸部显示文字 |
| **Note** | 机器人脸部屏幕显示文字，支持指令显示“Hello World” |

### robot.4. /zj_humanoid/robot/joint_motor/set_zero

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/joint_motor/set_zero` |
| **Type** | `zj_robot/SetZero` |
| **Description** | 电机自动标零 |
| **Note** | 机器人关节自动标零 |

### robot.5. /zj_humanoid/robot/orin_states/connect_wifi

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/orin_states/connect_wifi` |
| **Type** | `zj_robot/ConnectWifi` |
| **Description** | orin连接wifi |
| **Note** | 尝试让机器人大脑orin去连接wifi热点 |

### robot.6. /zj_humanoid/robot/orin_states/wifi_list

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/orin_states/wifi_list` |
| **Type** | `zj_robot/WifiList` |
| **Description** | orin_wifi列表 |
| **Note** | 获取机器人大脑检测到的wifi热点名称，当前机器人大脑检测到多少个wifi信号 回复应大于1 |

### robot.7. /zj_humanoid/robot/pico_states/connect_wifi

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/pico_states/connect_wifi` |
| **Type** | `zj_robot/ConnectWifi` |
| **Description** | pico连接wifi |
| **Note** | 尝试让机器人小脑pico去连接wifi热点 |

### robot.8. /zj_humanoid/robot/pico_states/wifi_list

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/pico_states/wifi_list` |
| **Type** | `zj_robot/WifiList` |
| **Description** | pico_wifi列表 |
| **Note** | 获取机器人小脑检测到的wifi热点名称，当前机器人小脑检测到多少个wifi信号 回复应大于1 |

### robot.9. /zj_humanoid/robot/set_robot_state/OFF

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/set_robot_state/OFF` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 机器人关机 |
| **Note** | 将机器人关机 3秒后，大小脑关机，之后后没法检测到机器人建立ros链接 |

### robot.10. /zj_humanoid/robot/set_robot_state/restart

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/set_robot_state/restart` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 状态机重启 |
| **Note** | 机器人先进stop软急停状态，再自动变为RUN启动运行，在某些故障状态可以执行，但如果有异常的存在，也可能会失败，将机器人状态机重启 持续检测robot_state话题，经过最长60秒钟的等待，状态应切换为：RUN |

### robot.11. /zj_humanoid/robot/set_robot_state/run

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/robot/set_robot_state/run` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 状态机运行 |
| **Note** | 如果机器人处于非RUN状态，尝试将机器人状态值设置为RUN，但如果有异常的存在，也可能会失败，将机器人状态设置为RUN 持续检测robot_state话题，经过最长60秒钟的等待，状态应切换为：RUN |

### robot.12. /zj_humanoid/robot/set_robot_state/stop

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

## 📦 UPPERLIMB (39 services)

### upperlimb.1. /zj_humanoid/upperlimb/FK/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/FK/left_arm` |
| **Type** | `upperlimb/FK` |
| **Description** | 左臂正解 |

### upperlimb.2. /zj_humanoid/upperlimb/FK/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/FK/right_arm` |
| **Type** | `upperlimb/FK` |
| **Description** | 右臂正解 |

### upperlimb.3. /zj_humanoid/upperlimb/IK/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/IK/left_arm` |
| **Type** | `upperlimb/IK` |
| **Description** | 左臂逆解 |
| **Note** | 左臂逆解 |

### upperlimb.4. /zj_humanoid/upperlimb/IK/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/IK/right_arm` |
| **Type** | `upperlimb/IK` |
| **Description** | 右臂逆解 |

### upperlimb.5. /zj_humanoid/upperlimb/go_down/dual_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_down/dual_arm` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 双臂放下 |

### upperlimb.6. /zj_humanoid/upperlimb/go_down/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_down/left_arm` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 左臂放下 |

### upperlimb.7. /zj_humanoid/upperlimb/go_down/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_down/right_arm` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 右臂放下 |

### upperlimb.8. /zj_humanoid/upperlimb/go_home/dual_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_home/dual_arm` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 双臂回到home点 |
| **Note** | 双臂回到内置设置的home点 |

### upperlimb.9. /zj_humanoid/upperlimb/go_home/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_home/left_arm` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 左臂回到home点 |
| **Note** | 左臂回到内置设置的home点 |

### upperlimb.10. /zj_humanoid/upperlimb/go_home/lifting

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_home/lifting` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 升降回到内home点 |
| **Note** | 升降回到内置设置的home点 |

### upperlimb.11. /zj_humanoid/upperlimb/go_home/neck

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_home/neck` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 脖子回到home点 |
| **Note** | 脖子回到内置设置的home点 |

### upperlimb.12. /zj_humanoid/upperlimb/go_home/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_home/right_arm` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 右臂回到home点 |
| **Note** | 右臂回到内置设置的home点 |

### upperlimb.13. /zj_humanoid/upperlimb/go_home/waist

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_home/waist` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 腰部回到home点 |
| **Note** | 腰部回到内置设置的home点 |

### upperlimb.14. /zj_humanoid/upperlimb/go_home/whole_body

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/go_home/whole_body` |
| **Type** | `upperlimb/ArmType` |
| **Description** | 全身回到home点 |
| **Note** | 全身指定部位回到内置设置的home点 |

### upperlimb.15. /zj_humanoid/upperlimb/movej/dual_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej/dual_arm` |
| **Type** | `upperlimb/MoveJ` |
| **Description** | 双臂movej |
| **Note** | 关节空间下,双臂点到点运动 |

### upperlimb.16. /zj_humanoid/upperlimb/movej/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej/left_arm` |
| **Type** | `upperlimb/MoveJ` |
| **Description** | 左臂movej |
| **Note** | 关节空间下,左臂点到点运动 |

### upperlimb.17. /zj_humanoid/upperlimb/movej/lift

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej/lift` |
| **Type** | `upperlimb/MoveJ` |
| **Description** | 升降movej |
| **Note** | 关节空间下,升降点到点运动 |

### upperlimb.18. /zj_humanoid/upperlimb/movej/neck

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej/neck` |
| **Type** | `upperlimb/MoveJ` |
| **Description** | 脖子movej |
| **Note** | 关节空间下,脖子点到点运动 |

### upperlimb.19. /zj_humanoid/upperlimb/movej/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej/right_arm` |
| **Type** | `upperlimb/MoveJ` |
| **Description** | 右臂movej |
| **Note** | 关节空间下,右臂点到点运动 |

### upperlimb.20. /zj_humanoid/upperlimb/movej/waist

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej/waist` |
| **Type** | `upperlimb/MoveJ` |
| **Description** | 腰部movej |
| **Note** | 关节空间下,腰部点到点运动 |

### upperlimb.21. /zj_humanoid/upperlimb/movej/whole_body

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej/whole_body` |
| **Type** | `upperlimb/MoveJ` |
| **Description** | 全身movej |
| **Note** | 关节空间下,全身各部位点到点运动 |

### upperlimb.22. /zj_humanoid/upperlimb/movej_by_path/dual_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_path/dual_arm` |
| **Type** | `upperlimb/MoveJByPath` |
| **Description** | 双臂轨迹movej |
| **Note** | 关节空间下,双臂轨迹点路径运动 |

### upperlimb.23. /zj_humanoid/upperlimb/movej_by_path/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_path/left_arm` |
| **Type** | `upperlimb/MoveJByPath` |
| **Description** | 左臂轨迹movej |
| **Note** | 关节空间下,左臂轨迹点路径运动 |

### upperlimb.24. /zj_humanoid/upperlimb/movej_by_path/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_path/right_arm` |
| **Type** | `upperlimb/MoveJByPath` |
| **Description** | 右臂轨迹movej |
| **Note** | 关节空间下,右臂轨迹点路径运动 |

### upperlimb.25. /zj_humanoid/upperlimb/movej_by_pose/dual_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_pose/dual_arm` |
| **Type** | `upperlimb/MoveJByPose` |
| **Description** | 双臂末端movej |
| **Note** | tcp末端空间下,双臂末端位姿movej |

### upperlimb.26. /zj_humanoid/upperlimb/movej_by_pose/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_pose/left_arm` |
| **Type** | `upperlimb/MoveJByPose` |
| **Description** | 左臂末端movej |
| **Note** | tcp末端空间下,左臂末端位姿movej |

### upperlimb.27. /zj_humanoid/upperlimb/movej_by_pose/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movej_by_pose/right_arm` |
| **Type** | `upperlimb/MoveJByPose` |
| **Description** | 右臂末端movej |
| **Note** | tcp末端空间下,右臂末端位姿movej |

### upperlimb.28. /zj_humanoid/upperlimb/movel/dual_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movel/dual_arm` |
| **Type** | `upperlimb/MoveL` |
| **Description** | 双臂movel |
| **Note** | 关节空间下,双臂直线轨迹点运动 |

### upperlimb.29. /zj_humanoid/upperlimb/movel/left_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movel/left_arm` |
| **Type** | `upperlimb/MoveL` |
| **Description** | 左臂movel |
| **Note** | 关节空间下,左臂直线轨迹点运动 |

### upperlimb.30. /zj_humanoid/upperlimb/movel/right_arm

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/movel/right_arm` |
| **Type** | `upperlimb/MoveL` |
| **Description** | 右臂movel |
| **Note** | 关节空间下,右臂直线轨迹点运动 |

### upperlimb.31. /zj_humanoid/upperlimb/servoj/clear_params

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/servoj/clear_params` |
| **Type** | `upperlimb/Servo` |
| **Description** | 退出servoj |
| **Note** | 退出笛卡尔空间 高频位置跟随控制 |

### upperlimb.32. /zj_humanoid/upperlimb/servoj/set_params

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/servoj/set_params` |
| **Type** | `upperlimb/Servo` |
| **Description** | 设置servoj参数 |
| **Note** | 设置关节空间 高频位置跟随控制参数 |

### upperlimb.33. /zj_humanoid/upperlimb/servol/clear_params

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/servol/clear_params` |
| **Type** | `upperlimb/Servo` |
| **Description** | 退出servol |
| **Note** | 退出笛卡尔空间 高频位置跟随控制 |

### upperlimb.34. /zj_humanoid/upperlimb/servol/set_params

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/servol/set_params` |
| **Type** | `upperlimb/Servo` |
| **Description** | 设置servol参数 |
| **Note** | 设置笛卡尔空间 高频位置跟随控制参数 |

### upperlimb.35. /zj_humanoid/upperlimb/speedl/enable_speedl

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/speedl/enable_speedl` |
| **Type** | `std_srvs/SetBool` |
| **Description** | 启用speedl |
| **Note** | 启用笛卡尔空间 速度控制 |

### upperlimb.36. /zj_humanoid/upperlimb/stop_robot

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/stop_robot` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 停止上肢运动 |
| **Note** | 停止机器人运动 |

### upperlimb.37. /zj_humanoid/upperlimb/teach_mode/enter

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/teach_mode/enter` |
| **Type** | `upperlimb/ArmType` |
| **Description** | 进入示教模式 |

### upperlimb.38. /zj_humanoid/upperlimb/teach_mode/exit

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/teach_mode/exit` |
| **Type** | `upperlimb/ArmType` |
| **Description** | 退出示教模式 |

### upperlimb.39. /zj_humanoid/upperlimb/versions

| Field | Value |
|-------|-------|
| **Service Name** | `/zj_humanoid/upperlimb/versions` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 上肢模块版本 |
| **Note** | 查询当前上肢子系统的软件版本号 应回复软件版本号 |

## Topics

Total: 82 topics in 7 subsystems

---

## 📡 AUDIO (3 topics)

### audio.1. /zj_humanoid/audio/asr_text

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/audio/asr_text` |
| **Type** | `std_msgs/String` |
| **Direction** | 📤 Publish |
| **Description** | 语音转文字 |
| **Note** | 当前机器人听到了什么 |

### audio.2. /zj_humanoid/audio/audio_data

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/audio/audio_data` |
| **Type** | `audio/AudioData` |
| **Direction** | 📤 Publish |
| **Description** | 音频流数据 |
| **Note** | 麦克风收音后的音频数据流 |

### audio.3. /zj_humanoid/audio/listen_state

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/audio/listen_state` |
| **Type** | `audio/ListenInfo` |
| **Direction** | 📤 Publish |
| **Description** | 唤醒倾听状态 |
| **Note** | 当前是否为倾听状态 |

## 📡 HAND (5 topics)

### hand.1. /zj_humanoid/hand/finger_pressures/left

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/hand/finger_pressures/left` |
| **Type** | `hand/PressureSensor` |
| **Direction** | 📤 Publish |
| **Description** | 左手压力传感器 |
| **Note** | 当前左手压力传感器数值 |

### hand.2. /zj_humanoid/hand/finger_pressures/right

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/hand/finger_pressures/right` |
| **Type** | `hand/PressureSensor` |
| **Direction** | 📤 Publish |
| **Description** | 右手压力传感器数据 |
| **Note** | 当前右手压力传感器数值 |

### hand.3. /zj_humanoid/hand/joint_states

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/hand/joint_states` |
| **Type** | `sensor_msgs/JointState` |
| **Direction** | 📤 Publish |
| **Description** | 手部关节状态 |
| **Note** | 当前左手食指的角度是多少 应回复0-80度之间 |

### hand.4. /zj_humanoid/hand/wrist_force_sensor/left

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/hand/wrist_force_sensor/left` |
| **Type** | `geometry_msgs/WrenchStamped` |
| **Direction** | 📤 Publish |
| **Description** | 右手腕部传感器值 |
| **Note** | 当前左手腕部的检测到多少力 应回复0牛顿 |

### hand.5. /zj_humanoid/hand/wrist_force_sensor/right

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/hand/wrist_force_sensor/right` |
| **Type** | `geometry_msgs/WrenchStamped` |
| **Direction** | 📤 Publish |
| **Description** | 左手腕部传感器值 |
| **Note** | 当前右手腕部的检测到多少力 应回复0牛顿 |

## 📡 LOWERLIMB (9 topics)

### lowerlimb.1. /zj_humanoid/lowerlimb/body_imu

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/body_imu` |
| **Type** | `sensor_msgs/JointState` |
| **Direction** | 📤 Publish |
| **Description** | 腰部imu值 |

### lowerlimb.2. /zj_humanoid/lowerlimb/cmd_vel/calib

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/cmd_vel/calib` |
| **Type** | `geometry_msgs/Twist` |
| **Direction** | 📥 Subscribe |
| **Description** | 导航控制行走 |

### lowerlimb.3. /zj_humanoid/lowerlimb/cmd_vel/joy

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/cmd_vel/joy` |
| **Type** | `geometry_msgs/Twist` |
| **Direction** | 📥 Subscribe |
| **Description** | 游戏手柄控制行走 |

### lowerlimb.4. /zj_humanoid/lowerlimb/cmd_vel/web

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/cmd_vel/web` |
| **Type** | `geometry_msgs/Twist` |
| **Direction** | 📥 Subscribe |
| **Description** | 网页控制行走 |

### lowerlimb.5. /zj_humanoid/lowerlimb/debug_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/debug_info` |
| **Type** | `std_msgs/String` |
| **Direction** | 📤 Publish |
| **Description** | 运控debug信息 |
| **Note** | 运控debug信息 |

### lowerlimb.6. /zj_humanoid/lowerlimb/occupancy_state

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/occupancy_state` |
| **Type** | `std_msgs/Float32` |
| **Direction** | 📤 Publish |
| **Description** | 上肢模式控制 |
| **Note** | 上肢模式控制，可设置为下肢控制模式 |

### lowerlimb.7. /zj_humanoid/lowerlimb/set_lie

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/set_lie` |
| **Type** | `std_msgs/Float32` |
| **Direction** | 📥 Subscribe |
| **Description** | 下肢泄力 |
| **Note** | 下肢泄力，软急停 |

### lowerlimb.8. /zj_humanoid/lowerlimb/set_stand

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/set_stand` |
| **Type** | `std_msgs/Float32` |
| **Direction** | 📥 Subscribe |
| **Description** | 站立姿态 |
| **Note** | 站立姿态初始化 |

### lowerlimb.9. /zj_humanoid/lowerlimb/start_move

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/lowerlimb/start_move` |
| **Type** | `std_msgs/Float32` |
| **Direction** | 📥 Subscribe |
| **Description** | 开启运动模式 |
| **Note** | 开启运动模式，算法开始响应速度控制请求 |

## 📡 NAVIGATION (5 topics)

### navigation.1. /zj_humanoid/navigation/local_map

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/navigation/local_map` |
| **Type** | `navigation/LocalMap` |
| **Direction** | 📥 Subscribe |
| **Description** | 局部障碍物信息 |

### navigation.2. /zj_humanoid/navigation/map

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/navigation/map` |
| **Type** | `nav_msgs/OccupancyGrid` |
| **Direction** | 📥 Subscribe |
| **Description** | 全局地图信息 |
| **Note** | 全局地图信息 |

### navigation.3. /zj_humanoid/navigation/navigation_status

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/navigation/navigation_status` |
| **Type** | `navigation/NavigationStatus` |
| **Direction** | 📥 Subscribe |
| **Description** | 当前导航状态 |
| **Note** | 当前导航状态信息 |

### navigation.4. /zj_humanoid/navigation/odom_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/navigation/odom_info` |
| **Type** | `nav_msgs/Odometry` |
| **Direction** | 📥 Subscribe |
| **Description** | 当前位姿信息 |
| **Note** | 当前位姿信息，有定位时才会输出结果 |

### navigation.5. /zj_humanoid/navigation/task_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/navigation/task_info` |
| **Type** | `navigation/TakInfo` |
| **Direction** | 📤 Publish |
| **Description** | 发布导航任务 |
| **Note** | 任务信息，该话题仅发布导航任务，不返回导航的结果 |

## 📡 ROBOT (10 topics)

### robot.1. /zj_humanoid/robot/battery_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/robot/battery_info` |
| **Type** | `zj_robot/BatteryInfo` |
| **Direction** | 📤 Publish |
| **Description** | 电池相关信息 |
| **Note** | 电池BMS相关信息，机器人当前电量还剩多少 回复值应为1~100% |

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

### robot.10. /zj_humanoid/robot/work_status_form_start

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/robot/work_status_form_start` |
| **Type** | `zj_robot/WorkStatus` |
| **Direction** | 📤 Publish |
| **Description** | 工作状态 |
| **Note** | 机器人开机后单次工作状态发布，包含已运行时间，剩余工作时间，行进里程数等，描述下机器人本次开机后工作状态 回复因包含：已运行时间，剩余工作时间，行进里程数 |

## 📡 SENSOR (27 topics)

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

### sensor.19. /zj_humanoid/sensor/realsense_up/aligned_depth_to_color/camera_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/aligned_depth_to_color/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度aligned参数 |
| **Note** | 胸部深度相机的aligned_depth_to_color参数信息 |

### sensor.20. /zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度aligned图像 |
| **Note** | 胸部深度相机的aligned_depth_to_color RGB图像源数据 |

### sensor.21. /zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/aligned_depth_to_color/image_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度aligned压缩图 |
| **Note** | 胸部深度相机的aligned_depth_to_color压缩格式 |

### sensor.22. /zj_humanoid/sensor/realsense_up/color/camera_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/color/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度RGB参数 |
| **Note** | 胸部深度相机的参数信息 |

### sensor.23. /zj_humanoid/sensor/realsense_up/color/image_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/color/image_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度RGB图像 |
| **Note** | 胸部深度相机的RGB图像源数据 |

### sensor.24. /zj_humanoid/sensor/realsense_up/color/image_raw/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/color/image_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度压缩图 |
| **Note** | 胸部深度相机的RGB图像JPG格式 |

### sensor.25. /zj_humanoid/sensor/realsense_up/depth/camera_info

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/depth/camera_info` |
| **Type** | `sensor_msgs/CameraInfo` |
| **Direction** | 📤 Publish |
| **Description** | 胸部相机深度参数 |
| **Note** | 胸部深度相机的参数信息 |

### sensor.26. /zj_humanoid/sensor/realsense_up/depth/image_rect_raw

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/depth/image_rect_raw` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | publishs |
| **Description** | 胸部深度RGB图像 |
| **Note** | 胸部深度相机的RGB图像源数据 |

### sensor.27. /zj_humanoid/sensor/realsense_up/depth/image_rect_raw/compressed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/sensor/realsense_up/depth/image_rect_raw/compressed` |
| **Type** | `sensor_msgs/Image` |
| **Direction** | 📤 Publish |
| **Description** | 胸部深度压缩图 |
| **Note** | 胸部深度相机的RGB图像JPG格式 |

## 📡 UPPERLIMB (23 topics)

### upperlimb.1. /zj_humanoid/upperlimb/cmd_states

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/cmd_states` |
| **Type** | `upperlimb/CmdState` |
| **Direction** | 📤 Publish |
| **Description** | 上肢运行模式 |
| **Note** | 当前上肢运行模式是什么 回复应处于停止状态 |

### upperlimb.2. /zj_humanoid/upperlimb/joint_states

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/joint_states` |
| **Type** | `sensor_msgs/JointState` |
| **Direction** | 📤 Publish |
| **Description** | 上肢关节位置状态 |
| **Note** | 机器人上肢关节position状态值发布，查询当前机器人颈部pitch的角度 回复应处于+-42度间 |

### upperlimb.3. /zj_humanoid/upperlimb/servoj/left_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servoj/left_arm` |
| **Type** | `upperlimb/Joints` |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂servoj |
| **Note** | 关节空间 高频位置控制 |

### upperlimb.4. /zj_humanoid/upperlimb/servoj/neck

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servoj/neck` |
| **Type** | `upperlimb/Joints` |
| **Direction** | 📥 Subscribe |
| **Description** | 颈部servoj |
| **Note** | 关节空间 高频位置控制 |

### upperlimb.5. /zj_humanoid/upperlimb/servoj/right_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servoj/right_arm` |
| **Type** | `upperlimb/Joints` |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂servoj |
| **Note** | 关节空间 高频位置控制 |

### upperlimb.6. /zj_humanoid/upperlimb/servoj/waist

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servoj/waist` |
| **Type** | `upperlimb/Joints` |
| **Direction** | 📥 Subscribe |
| **Description** | 腰部servoj |
| **Note** | 关节空间 高频位置控制 |

### upperlimb.7. /zj_humanoid/upperlimb/servoj/whole_body

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servoj/whole_body` |
| **Type** | `upperlimb/Joints` |
| **Direction** | 📥 Subscribe |
| **Description** | 全身servoj |
| **Note** | 关节空间 高频位置控制 |

### upperlimb.8. /zj_humanoid/upperlimb/servol/dual_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servol/dual_arm` |
| **Type** | `geometry_msgs/Pose` |
| **Direction** | 📥 Subscribe |
| **Description** | 双臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### upperlimb.9. /zj_humanoid/upperlimb/servol/left_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servol/left_arm` |
| **Type** | `geometry_msgs/Pose` |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### upperlimb.10. /zj_humanoid/upperlimb/servol/right_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/servol/right_arm` |
| **Type** | `geometry_msgs/Pose` |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### upperlimb.11. /zj_humanoid/upperlimb/speedj/enable_speedj

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedj/enable_speedj` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📥 Subscribe |
| **Description** | 启用speedj |
| **Note** | 启用关节空间速度控制 |

### upperlimb.12. /zj_humanoid/upperlimb/speedj/left_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedj/left_arm` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂speedj |
| **Note** | 关节空间速度控制 |

### upperlimb.13. /zj_humanoid/upperlimb/speedj/lift

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedj/lift` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📥 Subscribe |
| **Description** | 升降speedj |
| **Note** | 关节空间速度控制 |

### upperlimb.14. /zj_humanoid/upperlimb/speedj/neck

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedj/neck` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📥 Subscribe |
| **Description** | 脖子speedj |
| **Note** | 关节空间速度控制 |

### upperlimb.15. /zj_humanoid/upperlimb/speedj/right_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedj/right_arm` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂speedj |
| **Note** | 关节空间速度控制 |

### upperlimb.16. /zj_humanoid/upperlimb/speedj/waist

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedj/waist` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📥 Subscribe |
| **Description** | 腰speedj |
| **Note** | 关节空间速度控制 |

### upperlimb.17. /zj_humanoid/upperlimb/speedj/whole_body

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedj/whole_body` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📥 Subscribe |
| **Description** | 全身speedj |
| **Note** | 关节空间速度控制 |

### upperlimb.18. /zj_humanoid/upperlimb/speedl/dual_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedl/dual_arm` |
| **Type** | `upperlimb/SpeedL` |
| **Direction** | 📥 Subscribe |
| **Description** | 双臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### upperlimb.19. /zj_humanoid/upperlimb/speedl/left_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedl/left_arm` |
| **Type** | `upperlimb/SpeedL` |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### upperlimb.20. /zj_humanoid/upperlimb/speedl/right_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/speedl/right_arm` |
| **Type** | `upperlimb/SpeedL` |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### upperlimb.21. /zj_humanoid/upperlimb/tcp_pose/left_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/tcp_pose/left_arm` |
| **Type** | `upperlimb/Pose` |
| **Direction** | 📤 Publish |
| **Description** | 左臂tcp位姿控制 |
| **Note** | 左手臂末端位姿 |

### upperlimb.22. /zj_humanoid/upperlimb/tcp_pose/right_arm

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/tcp_pose/right_arm` |
| **Type** | `upperlimb/Pose` |
| **Direction** | 📤 Publish |
| **Description** | 右臂tcp位姿控制 |
| **Note** | 右手臂末端位姿 |

### upperlimb.23. /zj_humanoid/upperlimb/tcp_speed

| Field | Value |
|-------|-------|
| **Topic Name** | `/zj_humanoid/upperlimb/tcp_speed` |
| **Type** | `upperlimb/TcpSpeed` |
| **Direction** | 📤 Publish |
| **Description** | 双臂tcp速度控制 |
| **Note** | 左右手臂末端速度 |

---

## Summary

- **Total Services**: 87
- **Total Topics**: 82
- **Total Interfaces**: 169
- **Subsystems**: 8 (services), 7 (topics)
