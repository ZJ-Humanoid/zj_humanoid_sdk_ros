---
title: NAVIGATION 子系统
description: NAVIGATION 子系统的所有ROS接口
---

# 🧭 NAVIGATION 子系统

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
## 📦 Services (5)
- get_cur_map_info
- get_map_list
- map_server_version
- set_map
- version
## 📡 Topics (7)
- local_map
- local_map_render
- map
- map_metadata
- navigation_code
- navigation_status
- odom_info
## ⚙️ Actions (1)
- navigation`
</script>

---

## 📦 Services (5)

### 1. `get_cur_map_info`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/navigation/get_cur_map_info |
| **Type** | [map_server_msgs/GetCurMapInfo](../zj_humanoid_types#getcurmapinfo) |
| **Description** | 获取当前地图信息 |

### 2. `get_map_list`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/navigation/get_map_list |
| **Type** | [map_server_msgs/GetMapList](../zj_humanoid_types#getmaplist) |
| **Description** | 获取可用地图列表 |

### 3. `map_server_version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/navigation/map_server_version |
| **Type** | std_srvs/Trigger |
| **Description** | 获取地图管理模块版本 |
| **Note** | 1.5 新增 |

### 4. `set_map`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/navigation/set_map |
| **Type** | [map_server_msgs/SetMap](../zj_humanoid_types#setmap) |
| **Description** | 切换当前地图 |

### 5. `version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/navigation/version |
| **Type** | std_srvs/Trigger |
| **Description** | 定位导航版本号 |

## 📡 Topics (7)

### 1. `local_map`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/navigation/local_map |
| **Type** | [navigation/LocalMap](../zj_humanoid_types#localmap) |
| **Direction** | 📤 Publish |
| **Description** | 局部障碍物信息 |

### 2. `local_map_render`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/navigation/local_map_render |
| **Type** | [navigation/LocalMap](../zj_humanoid_types#localmap) |
| **Direction** | 📤 Publish |
| **Description** | 前端展示用轻量局部地图 |
| **Note** | v1.5.0 新增，不替代导航使用的 local_map |

### 3. `map`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/navigation/map |
| **Type** | nav_msgs/OccupancyGrid |
| **Direction** | 📤 Publish |
| **Description** | 全局地图信息 |
| **Note** | 地图管理模块发布的当前全局地图 |

### 4. `map_metadata`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/navigation/map_metadata |
| **Type** | nav_msgs/MapMetaData |
| **Direction** | 📤 Publish |
| **Description** | 当前地图元数据 |

### 5. `navigation_code`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/navigation/navigation_code |
| **Type** | [navigation/ModuleStatus](../zj_humanoid_types#modulestatus) |
| **Direction** | 📤 Publish |
| **Description** | 导航模块状态与错误码 |

### 6. `navigation_status`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/navigation/navigation_status |
| **Type** | [navigation/NavigationState](../zj_humanoid_types#navigationstate) |
| **Direction** | 📤 Publish |
| **Description** | 当前导航状态 |
| **Note** | 当前导航状态信息 |

### 7. `odom_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/navigation/odom_info |
| **Type** | nav_msgs/Odometry |
| **Direction** | 📤 Publish |
| **Description** | 当前位姿信息 |
| **Note** | 当前位姿信息，有定位时才会输出结果 |

## ⚙️ Actions (1)

### 1. `navigation`

| 字段 | 值 |
|------|-----|
| **Action Name** | /zj_humanoid/navigation/navigation |
| **Type** | [navigation/Navigation](../zj_humanoid_types#navigation) |
| **Description** | 导航接口 |

