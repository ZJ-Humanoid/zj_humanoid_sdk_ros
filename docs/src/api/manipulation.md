---
title: MANIPULATION 子系统
description: MANIPULATION 子系统的所有ROS接口
---

# 🔧 MANIPULATION 子系统

## 📊 接口概览

<Markmap :content="markmapContent" />

<script setup>
const markmapContent = `---
markmap:
  initialExpandLevel: 3
  colorFreezeLevel: 3
  maxWidth: 200
---

# 🔧 MANIPULATION 子系统
## 📦 Services (8)
- camera_calibration
- execute_pick_task
- grasp_teach_service
- joint_space_trajectory_planner
- pose_estimation_service
- pose_space_trajectory_planner
- scene_update
- version
## ⚙️ Actions (1)
- instance_segmentation_action`
</script>

---

## 📦 Services (8)

### 1. `camera_calibration`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/manipulation/camera_calibration |
| **Type** | [manipulation/CameraCalibration](../zj_humanoid_types#cameracalibration) |
| **Description** | 相机内外参标定 |
| **Note** | 自动相机内外参标定，外参标定时机器人会执行一段轨迹，拍摄不同角度的照片，从而计算外参 |

### 2. `execute_pick_task`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/manipulation/execute_pick_task |
| **Type** | [manipulation/ExecutePickTask](../zj_humanoid_types#executepicktask) |
| **Description** | 执行抓取服务 |
| **Note** | 输出物品名称执行抓取服务 |

### 3. `grasp_teach_service`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/manipulation/grasp_teach_service |
| **Type** | [manipulation/GraspTeach](../zj_humanoid_types#graspteach) |
| **Description** | 视觉抓取示教 |
| **Note** | 视觉示教抓取，让机器人知道该从什么方位抓取物品 |

### 4. `joint_space_trajectory_planner`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/manipulation/joint_space_trajectory_planner |
| **Type** | [manipulation/JointSpaceTrajPlan](../zj_humanoid_types#jointspacetrajplan) |
| **Description** | 关节空间轨迹规划 |
| **Note** | 节空间轨迹规划，输出关节轨迹，示教模式下记录各个关节数据，据此生成完整的执行轨迹 |

### 5. `pose_estimation_service`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/manipulation/pose_estimation_service |
| **Type** | [manipulation/PoseEst](../zj_humanoid_types#poseest) |
| **Description** | 获取目标位姿 |
| **Note** | 输入图像获取指定物品的6D位姿 |

### 6. `pose_space_trajectory_planner`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/manipulation/pose_space_trajectory_planner |
| **Type** | [manipulation/PoseSpaceTrajPlan](../zj_humanoid_types#posespacetrajplan) |
| **Description** | 末端轨迹规划 |
| **Note** | 末端空间轨迹规划，示教模式下记录各个末端执行器数据，据此生成完整的执行轨迹 |

### 7. `scene_update`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/manipulation/scene_update |
| **Type** | [manipulation/SceneUpdate](../zj_humanoid_types#sceneupdate) |
| **Description** | 场景更新 |
| **Note** | 机器人场景更新,基于二维码，需要场景中有二维码，机器人抓取物品前的环境感知 |

### 8. `version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/manipulation/version |
| **Type** | std_srvs/Trigger |
| **Description** | 操作模块版本号 |

## ⚙️ Actions (1)

### 1. `instance_segmentation_action`

| 字段 | 值 |
|------|-----|
| **Action Name** | /zj_humanoid/manipulation/instance_segmentation_action |
| **Type** | [manipulation/InstSeg](../zj_humanoid_types#instseg) |
| **Description** | 实例分割 |

