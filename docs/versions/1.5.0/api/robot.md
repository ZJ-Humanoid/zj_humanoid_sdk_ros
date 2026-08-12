---
title: ROBOT 子系统
description: ROBOT 子系统的所有ROS接口
---

# 🤖 ROBOT 子系统

## 📊 接口概览

<Markmap :content="markmapContent" />

<script setup>
const markmapContent = `---
markmap:
  initialExpandLevel: 3
  colorFreezeLevel: 3
  maxWidth: 200
---

# 🤖 ROBOT 子系统
## 📦 Services (13)
- basic_info
- face_show
  - media_play
  - text_show
- hardware
  - set_led
- joint_motor
  - set_zero
- orin_states
  - connect_wifi
  - wifi_list
- pico_states
  - connect_wifi
  - wifi_list
- set_robot_state
  - OFF
  - restart
  - run
  - stop
## 📡 Topics (10)
- battery_info
- monitor
- robot_state
- work_status_from_start
- joint_motor
  - errors
  - temperatures
- orin_states
  - errors
  - resource
- pico_states
  - errors
  - resource`
</script>

---

## 📦 Services (13)

### 1. `basic_info`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/robot/basic_info |
| **Type** | [robot/BasicInfo](../zj_humanoid_types#basicinfo) |
| **Description** | 机器人基础信息 |
| **Note** | 描述下机器人的基础信息 回复应包含机器人的型号，硬件版本号，软件版本号，IP地址 |

### 2. `face_show/media_play`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/robot/face_show/media_play |
| **Type** | [robot/FaceShow](../zj_humanoid_types#faceshow) |
| **Description** | 脸部显示视频 |
| **Note** | 机器人脸部屏幕显示,播放视频或图像文件，如播放“Hello_World.mp4” |

### 3. `face_show/text_show`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/robot/face_show/text_show |
| **Type** | [robot/FaceScreen](../zj_humanoid_types#facescreen) |
| **Description** | 脸部显示文字 |
| **Note** | 机器人脸部屏幕显示文字，支持指令显示“Hello World” |

### 4. `hardware/set_led`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/robot/hardware/set_led |
| **Type** | [robot/SetLED](../zj_humanoid_types#setled) |
| **Description** | 设置 WA2 左右灯带和前灯状态 |
| **Note** | 1.5 新增；由 robot_manger 统一持有底层 XDDP 连接 |

### 5. `joint_motor/set_zero`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/robot/joint_motor/set_zero |
| **Type** | [robot/SetZero](../zj_humanoid_types#setzero) |
| **Description** | 电机自动标零 |
| **Note** | 机器人关节自动标零 |

### 6. `orin_states/connect_wifi`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/robot/orin_states/connect_wifi |
| **Type** | [robot/ConnectWifi](../zj_humanoid_types#connectwifi) |
| **Description** | orin连接wifi |
| **Note** | 尝试让机器人大脑orin去连接wifi热点 |

### 7. `orin_states/wifi_list`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/robot/orin_states/wifi_list |
| **Type** | [robot/WifiList](../zj_humanoid_types#wifilist) |
| **Description** | orin_wifi列表 |
| **Note** | 获取机器人大脑检测到的wifi热点名称，当前机器人大脑检测到多少个wifi信号 回复应大于1 |

### 8. `pico_states/connect_wifi`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/robot/pico_states/connect_wifi |
| **Type** | [robot/ConnectWifi](../zj_humanoid_types#connectwifi) |
| **Description** | pico连接wifi |
| **Note** | 尝试让机器人小脑pico去连接wifi热点 |

### 9. `pico_states/wifi_list`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/robot/pico_states/wifi_list |
| **Type** | [robot/WifiList](../zj_humanoid_types#wifilist) |
| **Description** | pico_wifi列表 |
| **Note** | 获取机器人小脑检测到的wifi热点名称，当前机器人小脑检测到多少个wifi信号 回复应大于1 |

### 10. `set_robot_state/OFF`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/robot/set_robot_state/OFF |
| **Type** | std_srvs/Trigger |
| **Description** | 机器人关机 |
| **Note** | 将机器人关机 3秒后，大小脑关机，之后后没法检测到机器人建立ros链接 |

### 11. `set_robot_state/restart`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/robot/set_robot_state/restart |
| **Type** | std_srvs/Trigger |
| **Description** | 状态机重启 |
| **Note** | 机器人先进stop软急停状态，再自动变为RUN启动运行，在某些故障状态可以执行，但如果有异常的存在，也可能会失败，将机器人状态机重启 持续检测robot_state话题，经过最长60秒钟的等待，状态应切换为：RUN |

### 12. `set_robot_state/run`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/robot/set_robot_state/run |
| **Type** | std_srvs/Trigger |
| **Description** | 状态机运行 |
| **Note** | 如果机器人处于非RUN状态，尝试将机器人状态值设置为RUN，但如果有异常的存在，也可能会失败，将机器人状态设置为RUN 持续检测robot_state话题，经过最长60秒钟的等待，状态应切换为：RUN |

### 13. `set_robot_state/stop`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/robot/set_robot_state/stop |
| **Type** | std_srvs/Trigger |
| **Description** | 机器人软急停 |
| **Note** | 机器人软急停状态，状态机值将切换为ERR，在机器发生异常时使用，将机器人状态设置为stop 1秒后，检测robot_state话题，状态应切换为：ERR |

## 📡 Topics (10)

### 1. `battery_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/robot/battery_info |
| **Type** | sensor_msgs/BatteryState |
| **Direction** | 📤 Publish |
| **Description** | 电池相关信息 |
| **Note** | percentage 取值为 0.0-1.0，WA1、WA2 通用 |

### 2. `joint_motor/errors`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/robot/joint_motor/errors |
| **Type** | sensor_msgs/JointState |
| **Direction** | 📤 Publish |
| **Description** | 关节电机错误信息 |
| **Note** | 机器人关节电机错误信息，机器人关节是否有错误发生 回复应包含：没有 |

### 3. `joint_motor/temperatures`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/robot/joint_motor/temperatures |
| **Type** | sensor_msgs/JointState |
| **Direction** | 📤 Publish |
| **Description** | 关节电机温度信息 |
| **Note** | 关节电机温度信息，当前机器人膝关节温度是多少 回复应介于10-80度之间 |

### 4. `monitor`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/robot/monitor |
| **Type** | [robot/ModulesMonitor](../zj_humanoid_types#modulesmonitor) |
| **Direction** | 📤 Publish |
| **Description** | 运行状态检测 |
| **Note** | 机器人内部软件和算法模块运行状态检测, 包含上肢，灵巧手，遥控器，下肢，四目相机，深度相机，定位模块，导航模块，语音模块等 |

### 5. `orin_states/errors`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/robot/orin_states/errors |
| **Type** | [robot/Errors](../zj_humanoid_types#errors) |
| **Direction** | 📤 Publish |
| **Description** | orin错误汇总 |
| **Note** | 机器人大脑orin错误汇总，包括over_temp,over_cpu,over_mem,over_disk等，机器人大脑模块是否有错误发生 回复应包含：没有 |

### 6. `orin_states/resource`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/robot/orin_states/resource |
| **Type** | [robot/Resource](../zj_humanoid_types#resource) |
| **Direction** | 📤 Publish |
| **Description** | orin资源统计 |
| **Note** | 机器人大脑的资源状态 回复应包含：大脑的cpu,温度，内存，硬盘的用量 |

### 7. `pico_states/errors`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/robot/pico_states/errors |
| **Type** | [robot/Errors](../zj_humanoid_types#errors) |
| **Direction** | 📤 Publish |
| **Description** | pico错误汇总 |
| **Note** | 机器人小脑pico错误汇总，包括over_temp,over_cpu,over_mem,over_disk等，机器人小脑模块是否有错误发生 回复应包含：没有 |

### 8. `pico_states/resource`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/robot/pico_states/resource |
| **Type** | [robot/Resource](../zj_humanoid_types#resource) |
| **Direction** | 📤 Publish |
| **Description** | pico资源统计 |
| **Note** | 机器人小脑pico资源状态 回复应包含：小脑的cpu,温度，内存，硬盘的用量 |

### 9. `robot_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/robot/robot_state |
| **Type** | [robot/RobotState](../zj_humanoid_types#robotstate) |
| **Direction** | 📤 Publish |
| **Description** | 机器人状态机值 |
| **Note** | 机器人状态机值实时发布，只有当机器人进入RUN状态，机器人才能进行动作的执行，机器人当前处于什么状态 回复应包含：RUN状态 |

### 10. `work_status_from_start`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/robot/work_status_from_start |
| **Type** | [robot/WorkStatus](../zj_humanoid_types#workstatus) |
| **Direction** | 📤 Publish |
| **Description** | 工作状态 |
| **Note** | 机器人开机后单次工作状态发布，包含已运行时间，剩余工作时间，行进里程数等，描述下机器人本次开机后工作状态 回复因包含：已运行时间，剩余工作时间，行进里程数 |

