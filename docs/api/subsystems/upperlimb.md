---
title: UPPERLIMB 子系统
description: UPPERLIMB 子系统的所有ROS接口
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

# 🦾 UPPERLIMB 子系统
## 📦 Services (39)
- stop_robot
- versions
- FK
  - left_arm
  - right_arm
- IK
  - left_arm
  - right_arm
- go_down
  - dual_arm
  - left_arm
  - right_arm
- go_home
  - dual_arm
  - left_arm
  - lifting
  - neck
  - right_arm
  - waist
  - whole_body
- movej
  - dual_arm
  - left_arm
  - lift
  - neck
  - right_arm
  - waist
  - whole_body
- movej_by_path
  - dual_arm
  - left_arm
  - right_arm
- movej_by_pose
  - dual_arm
  - left_arm
  - right_arm
- movel
  - dual_arm
  - left_arm
  - right_arm
- servoj
  - clear_params
  - set_params
- servol
  - clear_params
  - set_params
- speedl
  - enable_speedl
- teach_mode
  - enter
  - exit
## 📡 Topics (23)
- cmd_states
- joint_states
- left_arm
- neck
- right_arm
- waist
- whole_body
- dual_arm
- left_arm
- right_arm
- enable_speedj
- left_arm
- lift
- neck
- right_arm
- waist
- whole_body
- dual_arm
- left_arm
- right_arm
- ... 还有 3 个话题`
</script>

---

## 📦 Services (39)

### 1. `left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/FK/left_arm |
| **Type** | [upperlimb/FK](../../zj_humanoid_types#FK) |
| **Description** | 左臂正解 |

### 2. `right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/FK/right_arm |
| **Type** | [upperlimb/FK](../../zj_humanoid_types#FK) |
| **Description** | 右臂正解 |

### 3. `left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/IK/left_arm |
| **Type** | [upperlimb/IK](../../zj_humanoid_types#IK) |
| **Description** | 左臂逆解 |
| **Note** | 左臂逆解 |

### 4. `right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/IK/right_arm |
| **Type** | [upperlimb/IK](../../zj_humanoid_types#IK) |
| **Description** | 右臂逆解 |

### 5. `dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_down/dual_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 双臂放下 |

### 6. `left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_down/left_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 左臂放下 |

### 7. `right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_down/right_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 右臂放下 |

### 8. `dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/dual_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 双臂回到home点 |
| **Note** | 双臂回到内置设置的home点 |

### 9. `left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/left_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 左臂回到home点 |
| **Note** | 左臂回到内置设置的home点 |

### 10. `lifting`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/lifting |
| **Type** | std_srvs/Trigger |
| **Description** | 升降回到内home点 |
| **Note** | 升降回到内置设置的home点 |

### 11. `neck`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/neck |
| **Type** | std_srvs/Trigger |
| **Description** | 脖子回到home点 |
| **Note** | 脖子回到内置设置的home点 |

### 12. `right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/right_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 右臂回到home点 |
| **Note** | 右臂回到内置设置的home点 |

### 13. `waist`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/waist |
| **Type** | std_srvs/Trigger |
| **Description** | 腰部回到home点 |
| **Note** | 腰部回到内置设置的home点 |

### 14. `whole_body`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/whole_body |
| **Type** | [upperlimb/ArmType](../../zj_humanoid_types#ArmType) |
| **Description** | 全身回到home点 |
| **Note** | 全身指定部位回到内置设置的home点 |

### 15. `dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/dual_arm |
| **Type** | [upperlimb/MoveJ](../../zj_humanoid_types#MoveJ) |
| **Description** | 双臂movej |
| **Note** | 关节空间下,双臂点到点运动 |

### 16. `left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/left_arm |
| **Type** | [upperlimb/MoveJ](../../zj_humanoid_types#MoveJ) |
| **Description** | 左臂movej |
| **Note** | 关节空间下,左臂点到点运动 |

### 17. `lift`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/lift |
| **Type** | [upperlimb/MoveJ](../../zj_humanoid_types#MoveJ) |
| **Description** | 升降movej |
| **Note** | 关节空间下,升降点到点运动 |

### 18. `neck`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/neck |
| **Type** | [upperlimb/MoveJ](../../zj_humanoid_types#MoveJ) |
| **Description** | 脖子movej |
| **Note** | 关节空间下,脖子点到点运动 |

### 19. `right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/right_arm |
| **Type** | [upperlimb/MoveJ](../../zj_humanoid_types#MoveJ) |
| **Description** | 右臂movej |
| **Note** | 关节空间下,右臂点到点运动 |

### 20. `waist`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/waist |
| **Type** | [upperlimb/MoveJ](../../zj_humanoid_types#MoveJ) |
| **Description** | 腰部movej |
| **Note** | 关节空间下,腰部点到点运动 |

### 21. `whole_body`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/whole_body |
| **Type** | [upperlimb/MoveJ](../../zj_humanoid_types#MoveJ) |
| **Description** | 全身movej |
| **Note** | 关节空间下,全身各部位点到点运动 |

### 22. `dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/dual_arm |
| **Type** | [upperlimb/MoveJByPath](../../zj_humanoid_types#MoveJByPath) |
| **Description** | 双臂轨迹movej |
| **Note** | 关节空间下,双臂轨迹点路径运动 |

### 23. `left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/left_arm |
| **Type** | [upperlimb/MoveJByPath](../../zj_humanoid_types#MoveJByPath) |
| **Description** | 左臂轨迹movej |
| **Note** | 关节空间下,左臂轨迹点路径运动 |

### 24. `right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/right_arm |
| **Type** | [upperlimb/MoveJByPath](../../zj_humanoid_types#MoveJByPath) |
| **Description** | 右臂轨迹movej |
| **Note** | 关节空间下,右臂轨迹点路径运动 |

### 25. `dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_pose/dual_arm |
| **Type** | [upperlimb/MoveJByPose](../../zj_humanoid_types#MoveJByPose) |
| **Description** | 双臂末端movej |
| **Note** | tcp末端空间下,双臂末端位姿movej |

### 26. `left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_pose/left_arm |
| **Type** | [upperlimb/MoveJByPose](../../zj_humanoid_types#MoveJByPose) |
| **Description** | 左臂末端movej |
| **Note** | tcp末端空间下,左臂末端位姿movej |

### 27. `right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_pose/right_arm |
| **Type** | [upperlimb/MoveJByPose](../../zj_humanoid_types#MoveJByPose) |
| **Description** | 右臂末端movej |
| **Note** | tcp末端空间下,右臂末端位姿movej |

### 28. `dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movel/dual_arm |
| **Type** | [upperlimb/MoveL](../../zj_humanoid_types#MoveL) |
| **Description** | 双臂movel |
| **Note** | 关节空间下,双臂直线轨迹点运动 |

### 29. `left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movel/left_arm |
| **Type** | [upperlimb/MoveL](../../zj_humanoid_types#MoveL) |
| **Description** | 左臂movel |
| **Note** | 关节空间下,左臂直线轨迹点运动 |

### 30. `right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movel/right_arm |
| **Type** | [upperlimb/MoveL](../../zj_humanoid_types#MoveL) |
| **Description** | 右臂movel |
| **Note** | 关节空间下,右臂直线轨迹点运动 |

### 31. `clear_params`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/servoj/clear_params |
| **Type** | [upperlimb/Servo](../../zj_humanoid_types#Servo) |
| **Description** | 退出servoj |
| **Note** | 退出笛卡尔空间 高频位置跟随控制 |

### 32. `set_params`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/servoj/set_params |
| **Type** | [upperlimb/Servo](../../zj_humanoid_types#Servo) |
| **Description** | 设置servoj参数 |
| **Note** | 设置关节空间 高频位置跟随控制参数 |

### 33. `clear_params`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/servol/clear_params |
| **Type** | [upperlimb/Servo](../../zj_humanoid_types#Servo) |
| **Description** | 退出servol |
| **Note** | 退出笛卡尔空间 高频位置跟随控制 |

### 34. `set_params`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/servol/set_params |
| **Type** | [upperlimb/Servo](../../zj_humanoid_types#Servo) |
| **Description** | 设置servol参数 |
| **Note** | 设置笛卡尔空间 高频位置跟随控制参数 |

### 35. `enable_speedl`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/speedl/enable_speedl |
| **Type** | std_srvs/SetBool |
| **Description** | 启用speedl |
| **Note** | 启用笛卡尔空间 速度控制 |

### 36. `stop_robot`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/stop_robot |
| **Type** | std_srvs/Trigger |
| **Description** | 停止上肢运动 |
| **Note** | 停止机器人运动 |

### 37. `enter`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/teach_mode/enter |
| **Type** | [upperlimb/ArmType](../../zj_humanoid_types#ArmType) |
| **Description** | 进入示教模式 |

### 38. `exit`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/teach_mode/exit |
| **Type** | [upperlimb/ArmType](../../zj_humanoid_types#ArmType) |
| **Description** | 退出示教模式 |

### 39. `versions`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/versions |
| **Type** | std_srvs/Trigger |
| **Description** | 上肢模块版本 |
| **Note** | 查询当前上肢子系统的软件版本号 应回复软件版本号 |

## 📡 Topics (23)

### 1. `cmd_states`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/cmd_states |
| **Type** | [upperlimb/CmdState](../../zj_humanoid_types#CmdState) |
| **Direction** | 📤 Publish |
| **Description** | 上肢运行模式 |
| **Note** | 当前上肢运行模式是什么 回复应处于停止状态 |

### 2. `joint_states`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/joint_states |
| **Type** | sensor_msgs/JointState |
| **Direction** | 📤 Publish |
| **Description** | 上肢关节位置状态 |
| **Note** | 机器人上肢关节position状态值发布，查询当前机器人颈部pitch的角度 回复应处于+-42度间 |

### 3. `left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/left_arm |
| **Type** | [upperlimb/Joints](../../zj_humanoid_types#Joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂servoj |
| **Note** | 关节空间 高频位置控制 |

### 4. `neck`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/neck |
| **Type** | [upperlimb/Joints](../../zj_humanoid_types#Joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 颈部servoj |
| **Note** | 关节空间 高频位置控制 |

### 5. `right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/right_arm |
| **Type** | [upperlimb/Joints](../../zj_humanoid_types#Joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂servoj |
| **Note** | 关节空间 高频位置控制 |

### 6. `waist`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/waist |
| **Type** | [upperlimb/Joints](../../zj_humanoid_types#Joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 腰部servoj |
| **Note** | 关节空间 高频位置控制 |

### 7. `whole_body`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/whole_body |
| **Type** | [upperlimb/Joints](../../zj_humanoid_types#Joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 全身servoj |
| **Note** | 关节空间 高频位置控制 |

### 8. `dual_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servol/dual_arm |
| **Type** | geometry_msgs/Pose |
| **Direction** | 📥 Subscribe |
| **Description** | 双臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### 9. `left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servol/left_arm |
| **Type** | geometry_msgs/Pose |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### 10. `right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servol/right_arm |
| **Type** | geometry_msgs/Pose |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### 11. `enable_speedj`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/enable_speedj |
| **Type** | [upperlimb/TcpSpeed](../../zj_humanoid_types#TcpSpeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 启用speedj |
| **Note** | 启用关节空间速度控制 |

### 12. `left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/left_arm |
| **Type** | [upperlimb/TcpSpeed](../../zj_humanoid_types#TcpSpeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂speedj |
| **Note** | 关节空间速度控制 |

### 13. `lift`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/lift |
| **Type** | [upperlimb/TcpSpeed](../../zj_humanoid_types#TcpSpeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 升降speedj |
| **Note** | 关节空间速度控制 |

### 14. `neck`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/neck |
| **Type** | [upperlimb/TcpSpeed](../../zj_humanoid_types#TcpSpeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 脖子speedj |
| **Note** | 关节空间速度控制 |

### 15. `right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/right_arm |
| **Type** | [upperlimb/TcpSpeed](../../zj_humanoid_types#TcpSpeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂speedj |
| **Note** | 关节空间速度控制 |

### 16. `waist`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/waist |
| **Type** | [upperlimb/TcpSpeed](../../zj_humanoid_types#TcpSpeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 腰speedj |
| **Note** | 关节空间速度控制 |

### 17. `whole_body`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/whole_body |
| **Type** | [upperlimb/TcpSpeed](../../zj_humanoid_types#TcpSpeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 全身speedj |
| **Note** | 关节空间速度控制 |

### 18. `dual_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedl/dual_arm |
| **Type** | [upperlimb/SpeedL](../../zj_humanoid_types#SpeedL) |
| **Direction** | 📥 Subscribe |
| **Description** | 双臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### 19. `left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedl/left_arm |
| **Type** | [upperlimb/SpeedL](../../zj_humanoid_types#SpeedL) |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### 20. `right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedl/right_arm |
| **Type** | [upperlimb/SpeedL](../../zj_humanoid_types#SpeedL) |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### 21. `left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/tcp_pose/left_arm |
| **Type** | [upperlimb/Pose](../../zj_humanoid_types#Pose) |
| **Direction** | 📤 Publish |
| **Description** | 左臂tcp位姿控制 |
| **Note** | 左手臂末端位姿 |

### 22. `right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/tcp_pose/right_arm |
| **Type** | [upperlimb/Pose](../../zj_humanoid_types#Pose) |
| **Direction** | 📤 Publish |
| **Description** | 右臂tcp位姿控制 |
| **Note** | 右手臂末端位姿 |

### 23. `tcp_speed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/tcp_speed |
| **Type** | [upperlimb/TcpSpeed](../../zj_humanoid_types#TcpSpeed) |
| **Direction** | 📤 Publish |
| **Description** | 双臂tcp速度控制 |
| **Note** | 左右手臂末端速度 |

