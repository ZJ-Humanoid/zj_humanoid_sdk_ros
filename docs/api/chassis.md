---
title: CHASSIS 子系统
description: CHASSIS 子系统的所有ROS接口
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

# 📦 CHASSIS 子系统
## 📦 Services (6)
- agv_charge
- agv_connect
- agv_reset
- agv_version
- speed_control
- steer_control
## 📡 Topics (7)
- agv_imu
- agv_state
- calib_vel
- motor_info
- odom_info
- steer_command
- steer_info`
</script>

---

## 📦 Services (4)

### 1. `agv_charge`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/agv_charge |
| **Type** | chassis_msgs/AgvCharge |
| **Description** | 底盘AGV充电控制（WA2） |
| **Note** | 控制底盘AGV的充电操作和查询充电状态 |

### 2. `agv_connect`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/agv_connect |
| **Type** | chassis_msgs/Connect（WA2） |
| **Description** | 连接底盘AGV |
| **Note** | 连接底盘AGV，建立通信连接 |

### 3. `agv_reset`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/agv_reset |
| **Type** | std_srvs/Trigger |
| **Description** | 底盘AGV复位（WA2） |
| **Note** | 底盘AGV复位，恢复到初始状态 |

### 4. `agv_version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/agv_version |
| **Type** | std_srvs/Trigger |
| **Description** | 底盘AGV版本（WA1|WA2） |
| **Note** | 获取底盘AGV模块的版本信息 |

## 📡 Topics (5)

### 1. `agv_imu`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/agv_imu |
| **Type** | sensor_msgs/Imu |
| **Direction** | 📤 Publish |
| **Description** | 底盘IMU数据（WA1|WA2） |
| **Note** | 底盘AGV的IMU传感器数据,包含角速度、加速度和姿态信息 |

### 2. `agv_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/agv_state |
| **Type** | chassis_msgs/AgvState |
| **Direction** | 📤 Publish |
| **Description** | 底盘AGV整体状态（WA2） |
| **Note** | 底盘AGV的整体状态信息，包含运行状态、错误代码和系统状态 |

### 3. `motor_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/motor_info |
| **Type** | chassis_msgs/MotorInfo |
| **Direction** | 📤 Publish |
| **Description** | 底盘电机状态信息（WA2） |
| **Note** | 底盘AGV的电机状态信息数组，包含各电机的电流、速度、温度等参数 |

### 4. `odom_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/odom_info |
| **Type** | nav_msgs/Odometry |
| **Direction** | 📤 Publish |
| **Description** | 底盘里程计信息（WA1|WA2） |
| **Note** | 底盘里程计数据,包含位置、速度等信息 |

### 5. `steer_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/steer_info |
| **Type** | chassis_msgs/SteerInfo |
| **Direction** | 📤 Publish |
| **Description** | 底盘转向状态信息（WA2） |
| **Note** | 底盘AGV的转向状态信息，包含当前转向角度、速度和目标位置 |

### 6. `charge_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/charge_state |
| **Type** | chassis_msgs/PowerStatusStamped |
| **Direction** | 📤 Publish |
| **Description** | 底盘充电状态信息（WA2） |
| **Note** | 底盘AGV的充电状态信息，包含当前充电状态、电压、电流等参数 |

### 7. `stop_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/stop_state |
| **Type** | chassis_msgs/TriggerStamped |
| **Direction** | 📤 Publish |
| **Description** | 底盘停止状态信息（WA1） |
| **Note** | 底盘AGV的停止状态信息，包含当前停止状态、停止时间等参数 |

### 8. `collision_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/collision_state |
| **Type** | chassis_msgs/TriggerStamped |
| **Direction** | 📤 Publish |
| **Description** | 底盘碰撞状态信息（WA1） |
| **Note** | 底盘AGV的碰撞状态信息，包含当前碰撞状态、碰撞时间等参数 |
