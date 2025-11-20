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
## 📦 Services (15)
- version
- finger_pressures
  - zero
  - zero
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
  - zero
  - zero
## 📡 Topics (5)
- left
- right
- joint_states
- left
- right`
</script>

---

## 📦 Services (15)

### 1. `finger_pressures/left/zero`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/finger_pressures/left/zero |
| **Type** | std_srvs/Trigger |
| **Description** | 置零手指传感器 |
| **Note** | 置零压力传感器数值 |

### 2. `finger_pressures/left/zero`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/finger_pressures/left/zero |
| **Type** | std_srvs/Trigger |
| **Description** | 调用左手指尖压力传感器零位校准服务 |
| **Note** | 对左手指尖压力传感器进行零位校准,清除当前传感器偏置,将当前读数设为零点 |

### 3. `finger_pressures/right/zero`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/finger_pressures/right/zero |
| **Type** | std_srvs/Trigger |
| **Description** | 调用右手指尖压力传感器零位校准服务 |
| **Note** | 对右手指尖压力传感器进行零位校准,清除当前传感器偏置,将当前读数设为零点 |

### 4. `gesture_switch/dual`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/gesture_switch/dual |
| **Type** | [hand/Gesture](../zj_humanoid_types#gesture) |
| **Description** | 调用双手手势切换服务 |
| **Note** | 同时控制左右手执行指定手势。gesture_name数组中索引0为左手,索引1为右手。支持的手势包括:RESET、ROCK、ONE、TWO、THREE、FOUR等 |

### 5. `gesture_switch/left`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/gesture_switch/left |
| **Type** | [hand/Gesture](../zj_humanoid_types#gesture) |
| **Description** | 左手手势切换服务 |
| **Note** | 控制左手执行指定手势。支持的手势包括:RESET、ROCK、ONE、TWO、THREE、FOUR等,手势名称大小写不敏感 |

### 6. `gesture_switch/right`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/gesture_switch/right |
| **Type** | [hand/Gesture](../zj_humanoid_types#gesture) |
| **Description** | 调用右手手势切换服务 |
| **Note** | 控制右手执行指定手势。支持的手势包括:RESET、ROCK、ONE、TWO、THREE、FOUR等,手势名称大小写不敏感 |

### 7. `joint_switch/dual`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/joint_switch/dual |
| **Type** | [hand/HandJoint](../zj_humanoid_types#handjoint) |
| **Description** | 调用双手关节控制服务 |
| **Note** | 同时控制双手各关节运动到指定角度。双手关节数组会被合并为12个元素的数组发送给服务。前6个为左手,后6个为右手 |

### 8. `joint_switch/left`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/joint_switch/left |
| **Type** | [hand/HandJoint](../zj_humanoid_types#handjoint) |
| **Description** | 调用左手关节控制服务 |
| **Note** | 控制左手各关节运动到指定角度。关节角度数组顺序为[拇指弯曲,拇指摆动,食指弯曲,中指弯曲,无名指弯曲,小指弯曲],单位:弧度。关节角度会被限制在安全范围内 |

### 9. `joint_switch/right`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/joint_switch/right |
| **Type** | [hand/HandJoint](../zj_humanoid_types#handjoint) |
| **Description** | 调用右手关节控制服务 |
| **Note** | 控制右手各关节运动到指定角度。关节角度数组顺序为[拇指弯曲,拇指摆动,食指弯曲,中指弯曲,无名指弯曲,小指弯曲],单位:弧度。关节角度会被限制在安全范围内 |

### 10. `task_switch/left`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/task_switch/left |
| **Type** | std_srvs/Bool |
| **Description** | 左手掌任务控制 |

### 11. `task_switch/right`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/task_switch/right |
| **Type** | std_srvs/Bool |
| **Description** | 右手掌任务控制 |

### 12. `version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/version |
| **Type** | std_srvs/Trigger |
| **Description** | 获取手部控制模块版本信息 |
| **Note** | 查询手部控制模块的版本信息 |

### 13. `wrist_force_sensor/left/zero`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/wrist_force_sensor/left/zero |
| **Type** | std_srvs/Trigger |
| **Description** | 置零腕部传感器 |
| **Note** | 置零腕部传感器数值 |

### 14. `wrist_force_sensor/left/zero`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/wrist_force_sensor/left/zero |
| **Type** | std_srvs/Trigger |
| **Description** | 调用左手腕力传感器零位校准服务 |
| **Note** | 对左手腕力传感器进行零位校准,清除当前传感器偏置,将当前力和力矩读数设为零点 |

### 15. `wrist_force_sensor/right/zero`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/hand/wrist_force_sensor/right/zero |
| **Type** | std_srvs/Trigger |
| **Description** | 调用右手腕力传感器零位校准服务 |
| **Note** | 对右手腕力传感器进行零位校准,清除当前传感器偏置,将当前力和力矩读数设为零点 |

## 📡 Topics (5)

### 1. `finger_pressures/left`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/hand/finger_pressures/left |
| **Type** | [hand/PressureSensor](../zj_humanoid_types#pressuresensor) |
| **Direction** | 📥 Subscribe |
| **Description** | 订阅左手指尖压力传感器数据 |
| **Note** | 接收左手指尖压力传感器数据,压力值顺序为[大拇指,食指,中指,无名指,小拇指],单位为0.1N |

### 2. `finger_pressures/right`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/hand/finger_pressures/right |
| **Type** | [hand/PressureSensor](../zj_humanoid_types#pressuresensor) |
| **Direction** | 📥 Subscribe |
| **Description** | 订阅右手指尖压力传感器数据 |
| **Note** | 接收右手指尖压力传感器数据,压力值顺序为[大拇指,食指,中指,无名指,小拇指],单位为0.1N |

### 3. `joint_states`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/hand/joint_states |
| **Type** | sensor_msgs/JointState |
| **Direction** | 📥 Subscribe |
| **Description** | 订阅手部关节状态数据 |
| **Note** | 订阅手部所有关节的位置状态,包括左右手各6个关节:拇指弯曲、拇指摆动、食指弯曲、中指弯曲、无名指弯曲、小指弯曲 |

### 4. `wrist_force_sensor/left`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/hand/wrist_force_sensor/left |
| **Type** | geometry_msgs/WrenchStamped |
| **Direction** | 📥 Subscribe |
| **Description** | 订阅左手腕力传感器数据 |
| **Note** | 接收左手腕力传感器数据,包括力和力矩的三轴分量 |

### 5. `wrist_force_sensor/right`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/hand/wrist_force_sensor/right |
| **Type** | geometry_msgs/WrenchStamped |
| **Direction** | 📥 Subscribe |
| **Description** | 订阅右手腕力传感器数据 |
| **Note** | 接收右手腕力传感器数据,包括力和力矩的三轴分量 |

