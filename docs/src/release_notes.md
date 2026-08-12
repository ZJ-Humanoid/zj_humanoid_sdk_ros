---
layout: doc
title: ROS API 版本变更
description: ZJ Humanoid ROS1 API v1.3.0 至 v1.5.0 变更记录
---

# ROS API 版本变更

## v1.5.0

- 建图新增 `/zj_humanoid/perception/start_mapping` Service 和 `/zj_humanoid/perception/finish_mapping` Action；旧版 `/perception/mapping_service`、`/perception/post_processing` 保留为兼容接口。
- 定位与感知新增版本查询、模块状态、地图重定位接口，并新增前端展示用 `/zj_humanoid/navigation/local_map_render`。
- 底盘切换到 `chassis_msgs` 新协议，新增 DI/DO、软急停、配置读写、保电、充电状态和二维激光接口；速度输入统一为 `/zj_humanoid/cmd_vel/calib`。
- 上肢消息包升级到 `2.0.0`，新增 `/zj_humanoid/upperlimb/basic_info`，并完善双臂 IK、FK 返回值和带时间戳的控制消息。
- 手部 `PressureSensor` 增加传感器名称数组。
- 机器人新增 `/zj_humanoid/robot/hardware/set_led` Service。
- 语音补齐麦克风、扬声器状态和 PCM 流接口，媒体播放改为 Action，TTS 增加单轮 profile。
- 地图管理新增 `/zj_humanoid/navigation/map_server_version`。

::: info 底盘版本接口
v1.5.0 运行包实际提供 `/zj_humanoid/chassis/agv_version`（`std_srvs/Trigger`），继续沿用该名称以保持兼容。
:::

## v1.4.0

- 上肢新增雅可比矩阵、笛卡尔规划参数读写接口。
- FK、IK、MoveJByPose、MoveL、MoveLByPath 和 IsSingular 增加全身控制入口。
- 新增全身 `speedl` 话题，并在运动学和规划请求中加入 `arm_type`。

## v1.3.0

- 上肢新增拖动示教记录与回放 Action，以及轨迹停止、列表查询接口。
- 新增上肢碰撞检测启停和参数读写接口。
- 新增上肢仿真数据输入、输出话题。
- 导航接口升级为 Action，并增加模块状态码话题。
