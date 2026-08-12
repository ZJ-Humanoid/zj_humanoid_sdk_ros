---
title: NAVIGATION 子系统
description: NAVIGATION 子系统的所有ROS接口
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

# 🧭 NAVIGATION 子系统
## 📦 Services (1)
- version
## 📡 Topics (4)
- map
- local_map
- navigation_status
- odom_info`
</script>

---

## 📦 Services (1)

### 1. `version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/navigation/version |
| **Type** | std_srvs/Trigger |
| **Description** | 定位导航版本号 |

## 📡 Topics (4)

### 1. `map`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /map |
| **Type** | nav_msgs/OccupancyGrid |
| **Direction** | 📥 Subscribe |
| **Description** | 全局地图信息 |
| **Note** | 全局地图信息 |

### 2. `local_map`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/navigation/local_map |
| **Type** | [navigation/LocalMap](../zj_humanoid_types#localmap) |
| **Direction** | 📥 Subscribe |
| **Description** | 局部障碍物信息 |

### 3. `navigation_status`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/navigation/navigation_status |
| **Type** | [navigation/NavigationStatus](../zj_humanoid_types#navigationstatus) |
| **Direction** | 📥 Subscribe |
| **Description** | 当前导航状态 |
| **Note** | 当前导航状态信息 |

### 4. `odom_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/navigation/odom_info |
| **Type** | nav_msgs/Odometry |
| **Direction** | 📥 Subscribe |
| **Description** | 当前位姿信息 |
| **Note** | 当前位姿信息，有定位时才会输出结果 |

