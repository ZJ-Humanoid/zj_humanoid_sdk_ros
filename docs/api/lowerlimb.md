---
title: LOWERLIMB 子系统
description: LOWERLIMB 子系统的所有ROS接口
---

## 📊 接口概览

<Markmap :content="markmapContent" />

<script setup>
const markmapContent = `---
markmap:
  initialExpandLevel: 3
  colorFreezeLevel: 3
  maxWidth: 200
---

# 🦵 LOWERLIMB 子系统
## 📦 Services (1)
- version
## 📡 Topics (11)
- body_imu
- debug_info
- motor_info
- set_lie
- set_stand
- start_move
- state
- version
- cmd_vel
  - calib
  - joy
  - web`
</script>

---

## 📦 Services (1)

### 1. `version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/lowerlimb/version |
| **Type** | std_srvs/Trigger |
| **Description** | 下肢模块版本 |

## 📡 Topics (11)

### 1. `body_imu`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/body_imu |
| **Type** | sensor_msgs/JointState |
| **Direction** | 📤 Publish |
| **Description** | 腰部imu值 |
| **Note** | 在双足I2机器人中，IMU位于URDF中的base_link，轮臂机器人目前暂不适用该topic |

### 2. `cmd_vel/calib`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/cmd_vel/calib |
| **Type** | geometry_msgs/Twist |
| **Direction** | 📥 Subscribe |
| **Description** | 导航控制行走 |
| **Note** | 在多个cmd_vel topic中，优先级joy > calib > web |

### 3. `cmd_vel/joy`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/cmd_vel/joy |
| **Type** | geometry_msgs/Twist |
| **Direction** | 📥 Subscribe |
| **Description** | 游戏手柄控制行走 |
| **Note** | 在多个cmd_vel topic中，优先级joy > calib > web |

### 4. `cmd_vel/web`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/cmd_vel/web |
| **Type** | geometry_msgs/Twist |
| **Direction** | 📥 Subscribe |
| **Description** | 网页控制行走 |
| **Note** | 在多个cmd_vel topic中，优先级joy > calib > web |

### 5. `debug_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/debug_info |
| **Type** | std_msgs/String |
| **Direction** | 📤 Publish |
| **Description** | 运控debug信息 |
| **Note** | 双足型号运控debug信息，轮臂机器人暂不适用 |

### 6. `motor_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/motor_info |
| **Type** | sensor_msgs/JointState |
| **Direction** | 📤 Publish |
| **Description** | 下肢电机信息 |
| **Note** | 发布下肢各电机的状态信息，包括位置、速度、力矩等 |

### 7. `set_lie`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/set_lie |
| **Type** | std_msgs/Float32 |
| **Direction** | 📥 Subscribe |
| **Description** | 下肢泄力 |
| **Note** | 双足型号下肢泄力，软急停，轮臂机器人暂不适用 |

### 8. `set_stand`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/set_stand |
| **Type** | std_msgs/Float32 |
| **Direction** | 📥 Subscribe |
| **Description** | 站立姿态 |
| **Note** | 双足机器人站立姿态初始化，轮臂机器人暂不适用 |

### 9. `start_move`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/start_move |
| **Type** | std_msgs/Float32 |
| **Direction** | 📥 Subscribe |
| **Description** | 开启运动模式 |
| **Note** | 双足机器人开启运动模式，算法开始响应速度控制请求，轮臂机器人暂不适用 |

### 10. `state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/state |
| **Type** | sensor_msgs/JointState |
| **Direction** | 📤 Publish |
| **Description** | 下肢状态信息 |
| **Note** | 发布下肢整体状态信息，包括当前姿态、运动状态等 |

### 11. `version`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/version |
| **Type** | std_msgs/String |
| **Direction** | 📤 Publish |
| **Description** | 下肢模块版本信息 |
| **Note** | 发布下肢模块的版本信息，与service版本获取功能一致 |

