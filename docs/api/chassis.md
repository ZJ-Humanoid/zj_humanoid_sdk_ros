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

## 📦 Services (6)

### 1. `agv_charge`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/agv_charge |
| **Type** | zj_humanoid_msgs/AgvCharge |
| **Description** | 底盘AGV充电控制 |
| **Note** | 控制底盘AGV的充电操作和查询充电状态 |

### 2. `agv_connect`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/agv_connect |
| **Type** | std_srvs/Trigger |
| **Description** | 连接底盘AGV |
| **Note** | 连接底盘AGV，建立通信连接 |

### 3. `agv_reset`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/agv_reset |
| **Type** | std_srvs/Trigger |
| **Description** | 底盘AGV复位 |
| **Note** | 底盘AGV复位，恢复到初始状态 |

### 4. `agv_version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/agv_version |
| **Type** | std_srvs/Trigger |
| **Description** | 底盘AGV版本 |
| **Note** | 获取底盘AGV模块的版本信息 |

### 5. `speed_control`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/speed_control |
| **Type** | zj_humanoid_msgs/SpeedControl |
| **Description** | 底盘速度控制 |
| **Note** | 控制底盘AGV的线速度和角速度 |

### 6. `steer_control`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/steer_control |
| **Type** | zj_humanoid_msgs/SteerControl |
| **Description** | 底盘转向控制 |
| **Note** | 控制底盘AGV的转向角度和速度 |

## 📡 Topics (7)

### 1. `agv_imu`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/agv_imu |
| **Type** | sensor_msgs/Imu |
| **Direction** | 📤 Publish |
| **Description** | 底盘IMU数据 |
| **Note** | 底盘AGV的IMU传感器数据,包含角速度、加速度和姿态信息 |

### 2. `agv_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/agv_state |
| **Type** | zj_humanoid_msgs/AgvState |
| **Direction** | 📤 Publish |
| **Description** | 底盘AGV整体状态 |
| **Note** | 底盘AGV的整体状态信息，包含运行状态、错误代码和系统状态 |

### 3. `calib_vel`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/calib_vel |
| **Type** | zj_humanoid_msgs/VelocityCalibData |
| **Direction** | 📤 Publish |
| **Description** | 底盘速度标定数据 |
| **Note** | 底盘AGV的速度标定相关数据，用于速度控制系统的校准和优化 |

### 4. `motor_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/motor_info |
| **Type** | zj_humanoid_msgs/MotorInfoArray |
| **Direction** | 📤 Publish |
| **Description** | 底盘电机状态信息 |
| **Note** | 底盘AGV的电机状态信息数组，包含各电机的电流、速度、温度等参数 |

### 5. `odom_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/odom_info |
| **Type** | nav_msgs/Odometry |
| **Direction** | 📤 Publish |
| **Description** | 底盘里程计信息 |
| **Note** | 底盘里程计数据,包含位置、速度等信息 |

### 6. `steer_command`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/steer_command |
| **Type** | zj_humanoid_msgs/SteerCommand |
| **Direction** | 📥 Subscribe |
| **Description** | 底盘转向控制指令 |
| **Note** | 发送到底盘的转向控制指令，包含目标角度和速度 |

### 7. `steer_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/steer_info |
| **Type** | zj_humanoid_msgs/SteerInfo |
| **Direction** | 📤 Publish |
| **Description** | 底盘转向状态信息 |
| **Note** | 底盘AGV的转向状态信息，包含当前转向角度、速度和目标位置 |

