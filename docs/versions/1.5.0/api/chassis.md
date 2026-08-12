---
title: CHASSIS 子系统
description: CHASSIS 子系统的所有ROS接口
---

# 🚙 CHASSIS 子系统

## 📊 接口概览

<Markmap :content="markmapContent" />

<script setup>
const markmapContent = `---
markmap:
  initialExpandLevel: 3
  colorFreezeLevel: 3
  maxWidth: 200
---

# 🚙 CHASSIS 子系统
## 📦 Services (7)
- agv_charge
- agv_reset
- agv_version
- get_config
- set_config
- soc_keep
- soft_estop
## 📡 Topics (10)
- agv_imu
- agv_state
- charge_state
- dido_state
- laser_scan
- motor_info
- odom_info
- steer_command
- steer_info
- calib`
</script>

---

## 📦 Services (7)

### 1. `agv_charge`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/agv_charge |
| **Type** | [chassis_msgs/ChargeControl](../zj_humanoid_types#chargecontrol) |
| **Description** | 底盘AGV充电控制 |
| **Note** | WA1、WA2 通用 |

### 2. `agv_reset`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/agv_reset |
| **Type** | std_srvs/Trigger |
| **Description** | 底盘AGV复位 |
| **Note** | 底盘AGV复位，恢复到初始状态 |

### 3. `agv_version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/agv_version |
| **Type** | std_srvs/Trigger |
| **Description** | 底盘AGV版本 |
| **Note** | 获取底盘AGV模块的版本信息 |

### 4. `get_config`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/get_config |
| **Type** | [chassis_msgs/GetChassisConfig](../zj_humanoid_types#getchassisconfig) |
| **Description** | 查询底盘配置 |
| **Note** | 按配置键查询，空键的行为以底盘实现为准 |

### 5. `set_config`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/set_config |
| **Type** | [chassis_msgs/SetChassisConfig](../zj_humanoid_types#setchassisconfig) |
| **Description** | 设置底盘配置 |
| **Note** | 以键值对列表批量设置配置 |

### 6. `soc_keep`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/soc_keep |
| **Type** | [chassis_msgs/SOCkeepControl](../zj_humanoid_types#sockeepcontrol) |
| **Description** | 设置底盘保电参数 |
| **Note** | 可设置上下限、电压和电流 |

### 7. `soft_estop`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/chassis/soft_estop |
| **Type** | std_srvs/Trigger |
| **Description** | 底盘软急停 |
| **Note** | 触发底盘软件急停 |

## 📡 Topics (10)

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
| **Type** | [chassis_msgs/AGVNewState](../zj_humanoid_types#agvnewstate) |
| **Direction** | 📤 Publish |
| **Description** | 底盘AGV整体状态 |
| **Note** | 包含运行状态、急停、碰撞和电池信息 |

### 3. `charge_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/charge_state |
| **Type** | [chassis_msgs/PowerStatusStamped](../zj_humanoid_types#powerstatusstamped) |
| **Direction** | 📤 Publish |
| **Description** | 手充连接状态 |
| **Note** | WA1 专属 |

### 4. `dido_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/dido_state |
| **Type** | [chassis_msgs/DIDOState](../zj_humanoid_types#didostate) |
| **Direction** | 📤 Publish |
| **Description** | 数字输入输出与关机状态 |
| **Note** | WA1、WA2 通用 |

### 5. `laser_scan`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/laser_scan |
| **Type** | sensor_msgs/LaserScan |
| **Direction** | 📤 Publish |
| **Description** | 底盘二维激光数据 |
| **Note** | WA1 专属 |

### 6. `motor_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/motor_info |
| **Type** | [chassis_msgs/MotorInfo](../zj_humanoid_types#motorinfo) |
| **Direction** | 📤 Publish |
| **Description** | 底盘电机状态信息 |
| **Note** | 包含各电机的转速、位置、电流、连接状态和错误码 |

### 7. `odom_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/odom_info |
| **Type** | nav_msgs/Odometry |
| **Direction** | 📤 Publish |
| **Description** | 底盘里程计信息 |
| **Note** | 底盘里程计数据,包含位置、速度等信息 |

### 8. `steer_command`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/steer_command |
| **Type** | [chassis_msgs/SteerCommand](../zj_humanoid_types#steercommand) |
| **Direction** | 📥 Subscribe |
| **Description** | 底盘转向控制指令 |
| **Note** | 发送到底盘的转向控制指令，包含目标角度和速度 |

### 9. `steer_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/chassis/steer_info |
| **Type** | [chassis_msgs/SteerInfo](../zj_humanoid_types#steerinfo) |
| **Direction** | 📤 Publish |
| **Description** | 底盘转向状态信息 |
| **Note** | 底盘AGV的转向状态信息，包含当前转向角度、速度和目标位置 |

### 10. `calib`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/cmd_vel/calib |
| **Type** | geometry_msgs/Twist |
| **Direction** | 📥 Subscribe |
| **Description** | 底盘速度控制输入 |
| **Note** | WA1、WA2 通用 |

