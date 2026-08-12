---
title: PERCEPTION 子系统
description: PERCEPTION 子系统的所有ROS接口
---

# 🗺️ PERCEPTION 子系统

## 📊 接口概览

<Markmap :content="markmapContent" />

<script setup>
const markmapContent = `---
markmap:
  initialExpandLevel: 3
  colorFreezeLevel: 3
  maxWidth: 200
---

# 🗺️ PERCEPTION 子系统
## 📦 Services (7)
- mapping_service
- post_processing
- location_version
- mapping_version
- perception_version
- reloc
- start_mapping
## 📡 Topics (4)
- mapping_result
- location_code
- mapping_code
- perception_code
## ⚙️ Actions (1)
- finish_mapping`
</script>

---

## 📦 Services (7)

### 1. `mapping_service`

| 字段 | 值 |
|------|-----|
| **Service Name** | /perception/mapping_service |
| **Type** | [naviai_localization_msgs/Mapping](../zj_humanoid_types#mapping) |
| **Description** | 开始建图兼容接口 |
| **Note** | 兼容旧版调用；1.5 推荐使用 /zj_humanoid/perception/start_mapping |

### 2. `post_processing`

| 字段 | 值 |
|------|-----|
| **Service Name** | /perception/post_processing |
| **Type** | [naviai_localization_msgs/Post_processing](../zj_humanoid_types#post_processing) |
| **Description** | 结束建图兼容接口 |
| **Note** | 兼容旧版调用；1.5 推荐使用 finish_mapping Action |

### 3. `location_version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/perception/location_version |
| **Type** | std_srvs/Trigger |
| **Description** | 获取定位模块版本 |

### 4. `mapping_version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/perception/mapping_version |
| **Type** | std_srvs/Trigger |
| **Description** | 获取建图模块版本 |

### 5. `perception_version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/perception/perception_version |
| **Type** | std_srvs/Trigger |
| **Description** | 获取感知模块版本 |

### 6. `reloc`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/perception/reloc |
| **Type** | [naviai_localization_msgs/Lio](../zj_humanoid_types#lio) |
| **Description** | 触发地图重定位 |
| **Note** | 支持按地图名称、ID或路径发起重定位 |

### 7. `start_mapping`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/perception/start_mapping |
| **Type** | [naviai_localization_msgs/Mapping](../zj_humanoid_types#mapping) |
| **Description** | 开始建图 |
| **Note** | 可设置地图名、高度范围、分辨率和场景类型 |

## 📡 Topics (4)

### 1. `mapping_result`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /perception/mapping_result |
| **Type** | [naviai_localization_msgs/MappingResult](../zj_humanoid_types#mappingresult) |
| **Direction** | 📤 Publish |
| **Description** | 建图结果 |
| **Note** | 兼容旧版建图结果话题 |

### 2. `location_code`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/perception/location_code |
| **Type** | [module_common_msgs/ModuleStatus](../zj_humanoid_types#modulestatus) |
| **Direction** | 📤 Publish |
| **Description** | 定位模块状态与错误码 |

### 3. `mapping_code`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/perception/mapping_code |
| **Type** | [module_common_msgs/ModuleStatus](../zj_humanoid_types#modulestatus) |
| **Direction** | 📤 Publish |
| **Description** | 建图模块状态与错误码 |

### 4. `perception_code`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/perception/perception_code |
| **Type** | [module_common_msgs/ModuleStatus](../zj_humanoid_types#modulestatus) |
| **Direction** | 📤 Publish |
| **Description** | 感知模块状态与错误码 |

## ⚙️ Actions (1)

### 1. `finish_mapping`

| 字段 | 值 |
|------|-----|
| **Action Name** | /zj_humanoid/perception/finish_mapping |
| **Type** | [naviai_localization_msgs/FinishMapping](../zj_humanoid_types#finishmapping) |
| **Description** | 结束或中止建图 |
| **Note** | method=0 时结束并保存地图，method=1 时中止且不保存；后处理开始后无法被取消 |

