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
- versions
## 📡 Topics (9)
- body_imu
- calib
- joy
- web
- debug_info
- occupancy_state
- set_lie
- set_stand
- start_move`
</script>

---

## 📦 Services (1)

### 1. `versions`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/lowerlimb/versions |
| **Type** | std_srvs/Trigger |
| **Description** | 下肢模块版本 |

## 📡 Topics (9)

### 1. `body_imu`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/body_imu |
| **Type** | sensor_msgs/JointState |
| **Direction** | 📤 Publish |
| **Description** | 腰部imu值 |

### 2. `calib`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/cmd_vel/calib |
| **Type** | geometry_msgs/Twist |
| **Direction** | 📥 Subscribe |
| **Description** | 导航控制行走 |

### 3. `joy`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/cmd_vel/joy |
| **Type** | geometry_msgs/Twist |
| **Direction** | 📥 Subscribe |
| **Description** | 游戏手柄控制行走 |

### 4. `web`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/cmd_vel/web |
| **Type** | geometry_msgs/Twist |
| **Direction** | 📥 Subscribe |
| **Description** | 网页控制行走 |

### 5. `debug_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/debug_info |
| **Type** | std_msgs/String |
| **Direction** | 📤 Publish |
| **Description** | 运控debug信息 |
| **Note** | 运控debug信息 |

### 6. `occupancy_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/occupancy_state |
| **Type** | std_msgs/Float32 |
| **Direction** | 📤 Publish |
| **Description** | 上肢模式控制 |
| **Note** | 上肢模式控制，可设置为下肢控制模式 |

### 7. `set_lie`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/set_lie |
| **Type** | std_msgs/Float32 |
| **Direction** | 📥 Subscribe |
| **Description** | 下肢泄力 |
| **Note** | 下肢泄力，软急停 |

### 8. `set_stand`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/set_stand |
| **Type** | std_msgs/Float32 |
| **Direction** | 📥 Subscribe |
| **Description** | 站立姿态 |
| **Note** | 站立姿态初始化 |

### 9. `start_move`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/lowerlimb/start_move |
| **Type** | std_msgs/Float32 |
| **Direction** | 📥 Subscribe |
| **Description** | 开启运动模式 |
| **Note** | 开启运动模式，算法开始响应速度控制请求 |

