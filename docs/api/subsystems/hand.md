---
title: HAND 子系统
description: HAND 子系统的所有ROS接口
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

# 🖐️ HAND 子系统
## 📦 Services (11)
- versions
- finger_pressures
  - zero
- gesture_switch
  - dual
  - left
  - right
- joint_switch
  - dual
  - left
  - right
- task_switch
  - left
  - right
- wrist_force_sensor
  - zero
## 📡 Topics (5)
- left
- right
- joint_states
- left
- right`
</script>

---

## 📦 Services (11)

### 1. `zero`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/finger_pressures/left/zero |
| **Type** | std_srvs/Trigger |
| **Description** | 置零手指传感器 |
| **Note** | 置零压力传感器数值 |

### 2. `dual`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/gesture_switch/dual |
| **Type** | [hand/Gesture](../../zj_humanoid_types#Gesture) |
| **Description** | 双手手势切换 |

### 3. `left`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/gesture_switch/left |
| **Type** | [hand/Gesture](../../zj_humanoid_types#Gesture) |
| **Description** | 左手手势切换 |

### 4. `right`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/gesture_switch/right |
| **Type** | [hand/Gesture](../../zj_humanoid_types#Gesture) |
| **Description** | 右手手势切换 |

### 5. `dual`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/joint_switch/dual |
| **Type** | [hand/HandJoint](../../zj_humanoid_types#HandJoint) |
| **Description** | 双手手掌关节运动 |

### 6. `left`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/joint_switch/left |
| **Type** | [hand/HandJoint](../../zj_humanoid_types#HandJoint) |
| **Description** | 左手手掌关节运动 |
| **Note** | 左手食指弯曲40度 订阅/hand_joint_states左手食指数值应接近40度 |

### 7. `right`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/joint_switch/right |
| **Type** | [hand/HandJoint](../../zj_humanoid_types#HandJoint) |
| **Description** | 右手手掌关节运动 |
| **Note** | 右手食指弯曲40度 订阅/hand_joint_states左手食指数值应接近40度 |

### 8. `left`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/task_switch/left |
| **Type** | std_srvs/Bool |
| **Description** | 左手掌任务控制 |

### 9. `right`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/task_switch/right |
| **Type** | std_srvs/Bool |
| **Description** | 右手掌任务控制 |

### 10. `versions`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/versions |
| **Type** | std_srvs/Trigger |
| **Description** | 灵巧手版本号 |
| **Note** | 查询当前灵巧手子系统的版本号 |

### 11. `zero`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/wrist_force_sensor/left/zero |
| **Type** | std_srvs/Trigger |
| **Description** | 置零腕部传感器 |
| **Note** | 置零腕部传感器数值 |

## 📡 Topics (5)

### 1. `left`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/hand/finger_pressures/left |
| **Type** | [hand/PressureSensor](../../zj_humanoid_types#PressureSensor) |
| **Direction** | 📤 Publish |
| **Description** | 左手压力传感器 |
| **Note** | 当前左手压力传感器数值 |

### 2. `right`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/hand/finger_pressures/right |
| **Type** | [hand/PressureSensor](../../zj_humanoid_types#PressureSensor) |
| **Direction** | 📤 Publish |
| **Description** | 右手压力传感器数据 |
| **Note** | 当前右手压力传感器数值 |

### 3. `joint_states`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/hand/joint_states |
| **Type** | sensor_msgs/JointState |
| **Direction** | 📤 Publish |
| **Description** | 手部关节状态 |
| **Note** | 当前左手食指的角度是多少 应回复0-80度之间 |

### 4. `left`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/hand/wrist_force_sensor/left |
| **Type** | geometry_msgs/WrenchStamped |
| **Direction** | 📤 Publish |
| **Description** | 右手腕部传感器值 |
| **Note** | 当前左手腕部的检测到多少力 应回复0牛顿 |

### 5. `right`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/hand/wrist_force_sensor/right |
| **Type** | geometry_msgs/WrenchStamped |
| **Direction** | 📤 Publish |
| **Description** | 左手腕部传感器值 |
| **Note** | 当前右手腕部的检测到多少力 应回复0牛顿 |

