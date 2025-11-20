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
## 📦 Services (43)
- clear_servo_params
- enable_speedj
- set_servo_params
- stop
- version
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
  - neck
  - right_arm
  - waist
  - whole_body
- movej_by_pose
  - dual_arm
  - left_arm
  - right_arm
- movel
  - dual_arm
  - left_arm
  - right_arm
- servol
  - clear_params
  - set_params
- speedl
  - enable_speedl
- teach_mode
  - enter
  - exit
## 📡 Topics (28)
- joint_states
- occupancy_state
- tcp_speed
- uplimb_occupation
- uplimb_state
- servoj
  - dual_arm
  - left_arm
  - neck
  - right_arm
  - waist
  - whole_body
- servol
  - dual_arm
  - left_arm
  - right_arm
- speedj
  - dual_arm
  - enable_speedj
  - left_arm
  - lift
  - neck
  - right_arm
  - waist
  - whole_body
- speedl
  - dual_arm
  - left_arm
  - right_arm
- tcp_pose
  - left_arm
  - right_arm
- tcp_speed
  - dual_arm`
</script>

---

## 📦 Services (43)

### 1. `FK/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/FK/left_arm |
| **Type** | [upperlimb/FK](../zj_humanoid_types#fk) |
| **Description** | 左臂正解 |

### 2. `FK/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/FK/right_arm |
| **Type** | [upperlimb/FK](../zj_humanoid_types#fk) |
| **Description** | 右臂正解 |

### 3. `IK/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/IK/left_arm |
| **Type** | [upperlimb/IK](../zj_humanoid_types#ik) |
| **Description** | 左臂逆解 |
| **Note** | 左臂逆解 |

### 4. `IK/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/IK/right_arm |
| **Type** | [upperlimb/IK](../zj_humanoid_types#ik) |
| **Description** | 右臂逆解 |

### 5. `clear_servo_params`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/clear_servo_params |
| **Type** | [upperlimb/Servo](../zj_humanoid_types#servo) |
| **Description** | 清除伺服参数 |
| **Note** | 清除上肢伺服参数配置 |

### 6. `enable_speedj`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/enable_speedj |
| **Type** | std_srvs/SetBool |
| **Description** | 启用/禁用关节速度控制模式 |
| **Note** | 启用或禁用上肢speedj速度控制模式 |

### 7. `go_down/dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_down/dual_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 双臂放下 |

### 8. `go_down/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_down/left_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 左臂放下 |

### 9. `go_down/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_down/right_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 右臂放下 |

### 10. `go_home/dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/dual_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 双臂回到home点 |
| **Note** | 双臂回到内置设置的home点 |

### 11. `go_home/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/left_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 左臂回到home点 |
| **Note** | 左臂回到内置设置的home点 |

### 12. `go_home/lifting`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/lifting |
| **Type** | std_srvs/Trigger |
| **Description** | 升降回到内home点 |
| **Note** | 升降回到内置设置的home点 |

### 13. `go_home/neck`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/neck |
| **Type** | std_srvs/Trigger |
| **Description** | 脖子回到home点 |
| **Note** | 脖子回到内置设置的home点 |

### 14. `go_home/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/right_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 右臂回到home点 |
| **Note** | 右臂回到内置设置的home点 |

### 15. `go_home/waist`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/waist |
| **Type** | std_srvs/Trigger |
| **Description** | 腰部回到home点 |
| **Note** | 腰部回到内置设置的home点 |

### 16. `go_home/whole_body`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/whole_body |
| **Type** | [upperlimb/ArmType](../zj_humanoid_types#armtype) |
| **Description** | 全身回到home点 |
| **Note** | 全身指定部位回到内置设置的home点 |

### 17. `movej/dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/dual_arm |
| **Type** | [upperlimb/MoveJ](../zj_humanoid_types#movej) |
| **Description** | 双臂movej |
| **Note** | 关节空间下,双臂点到点运动 |

### 18. `movej/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/left_arm |
| **Type** | [upperlimb/MoveJ](../zj_humanoid_types#movej) |
| **Description** | 左臂movej |
| **Note** | 关节空间下,左臂点到点运动 |

### 19. `movej/lift`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/lift |
| **Type** | [upperlimb/MoveJ](../zj_humanoid_types#movej) |
| **Description** | 升降movej |
| **Note** | 关节空间下,升降点到点运动 |

### 20. `movej/neck`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/neck |
| **Type** | [upperlimb/MoveJ](../zj_humanoid_types#movej) |
| **Description** | 脖子movej |
| **Note** | 关节空间下,脖子点到点运动 |

### 21. `movej/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/right_arm |
| **Type** | [upperlimb/MoveJ](../zj_humanoid_types#movej) |
| **Description** | 右臂movej |
| **Note** | 关节空间下,右臂点到点运动 |

### 22. `movej/waist`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/waist |
| **Type** | [upperlimb/MoveJ](../zj_humanoid_types#movej) |
| **Description** | 腰部movej |
| **Note** | 关节空间下,腰部点到点运动 |

### 23. `movej/whole_body`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/whole_body |
| **Type** | [upperlimb/MoveJ](../zj_humanoid_types#movej) |
| **Description** | 全身movej |
| **Note** | 关节空间下,全身各部位点到点运动 |

### 24. `movej_by_path/dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/dual_arm |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 双臂轨迹movej |
| **Note** | 关节空间下,双臂轨迹点路径运动 |

### 25. `movej_by_path/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/left_arm |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 左臂轨迹movej |
| **Note** | 关节空间下,左臂轨迹点路径运动 |

### 26. `movej_by_path/neck`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/neck |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 关节空间下,脖子轨迹点路径运动 |
| **Note** | 控制颈部按照关节空间路径运动 |

### 27. `movej_by_path/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/right_arm |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 右臂轨迹movej |
| **Note** | 关节空间下,右臂轨迹点路径运动 |

### 28. `movej_by_path/waist`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/waist |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 关节空间下,腰部轨迹点路径运动 |
| **Note** | 控制腰部按照关节空间路径运动 |

### 29. `movej_by_path/whole_body`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/whole_body |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 关节空间下,全身轨迹点路径运动 |
| **Note** | 控制全身按照关节空间路径运动 |

### 30. `movej_by_pose/dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_pose/dual_arm |
| **Type** | [upperlimb/MoveJByPose](../zj_humanoid_types#movejbypose) |
| **Description** | 双臂末端movej |
| **Note** | tcp末端空间下,双臂末端位姿movej |

### 31. `movej_by_pose/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_pose/left_arm |
| **Type** | [upperlimb/MoveJByPose](../zj_humanoid_types#movejbypose) |
| **Description** | 左臂末端movej |
| **Note** | tcp末端空间下,左臂末端位姿movej |

### 32. `movej_by_pose/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_pose/right_arm |
| **Type** | [upperlimb/MoveJByPose](../zj_humanoid_types#movejbypose) |
| **Description** | 右臂末端movej |
| **Note** | tcp末端空间下,右臂末端位姿movej |

### 33. `movel/dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movel/dual_arm |
| **Type** | [upperlimb/MoveL](../zj_humanoid_types#movel) |
| **Description** | 双臂movel |
| **Note** | 关节空间下,双臂直线轨迹点运动 |

### 34. `movel/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movel/left_arm |
| **Type** | [upperlimb/MoveL](../zj_humanoid_types#movel) |
| **Description** | 左臂movel |
| **Note** | 关节空间下,左臂直线轨迹点运动 |

### 35. `movel/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movel/right_arm |
| **Type** | [upperlimb/MoveL](../zj_humanoid_types#movel) |
| **Description** | 右臂movel |
| **Note** | 关节空间下,右臂直线轨迹点运动 |

### 36. `servol/clear_params`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/servol/clear_params |
| **Type** | [upperlimb/Servo](../zj_humanoid_types#servo) |
| **Description** | 退出servol |
| **Note** | 退出笛卡尔空间 高频位置跟随控制 |

### 37. `servol/set_params`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/servol/set_params |
| **Type** | [upperlimb/Servo](../zj_humanoid_types#servo) |
| **Description** | 设置servol参数 |
| **Note** | 设置笛卡尔空间 高频位置跟随控制参数 |

### 38. `set_servo_params`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/set_servo_params |
| **Type** | [upperlimb/Servo](../zj_humanoid_types#servo) |
| **Description** | 设置伺服参数 |
| **Note** | 设置上肢伺服参数配置,包括时间和增益参数 |

### 39. `speedl/enable_speedl`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/speedl/enable_speedl |
| **Type** | std_srvs/SetBool |
| **Description** | 启用speedl |
| **Note** | 启用笛卡尔空间 速度控制 |

### 40. `stop`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/stop |
| **Type** | std_srvs/Trigger |
| **Description** | 停止机器人运动 |
| **Note** | 立即停止上肢所有运动 |

### 41. `teach_mode/enter`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/teach_mode/enter |
| **Type** | [upperlimb/ArmType](../zj_humanoid_types#armtype) |
| **Description** | 进入示教模式 |

### 42. `teach_mode/exit`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/teach_mode/exit |
| **Type** | [upperlimb/ArmType](../zj_humanoid_types#armtype) |
| **Description** | 退出示教模式 |

### 43. `version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/version |
| **Type** | std_srvs/Trigger |
| **Description** | 获取上肢版本信息 |
| **Note** | 查询上肢控制模块的版本信息 |

## 📡 Topics (28)

### 1. `joint_states`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/joint_states |
| **Type** | sensor_msgs/JointState |
| **Direction** | 📤 Publish |
| **Description** | 上肢关节位置状态 |
| **Note** | 机器人上肢关节position状态值发布，查询当前机器人颈部pitch的角度 回复应处于+-42度间 |

### 2. `occupancy_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/occupancy_state |
| **Type** | std_msgs/Int8 |
| **Direction** | 📥 Subscribe |
| **Description** | 订阅上肢占用状态 |
| **Note** | 该话题发布上肢的当前占用状态,用于防止多个控制源同时控制机器人 |

### 3. `servoj/dual_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/dual_arm |
| **Type** | [upperlimb/Joints](../zj_humanoid_types#joints) |
| **Direction** | 📤 Publish |
| **Description** | 关节空间下,双臂高频位置控制接口 |
| **Note** | 双臂关节空间伺服控制,不要使用定时sleep,该接口执行需要准确的时间戳会达到更好的效果 |

### 4. `servoj/left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/left_arm |
| **Type** | [upperlimb/Joints](../zj_humanoid_types#joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂servoj |
| **Note** | 关节空间 高频位置控制 |

### 5. `servoj/neck`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/neck |
| **Type** | [upperlimb/Joints](../zj_humanoid_types#joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 颈部servoj |
| **Note** | 关节空间 高频位置控制 |

### 6. `servoj/right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/right_arm |
| **Type** | [upperlimb/Joints](../zj_humanoid_types#joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂servoj |
| **Note** | 关节空间 高频位置控制 |

### 7. `servoj/waist`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/waist |
| **Type** | [upperlimb/Joints](../zj_humanoid_types#joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 腰部servoj |
| **Note** | 关节空间 高频位置控制 |

### 8. `servoj/whole_body`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/whole_body |
| **Type** | [upperlimb/Joints](../zj_humanoid_types#joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 全身servoj |
| **Note** | 关节空间 高频位置控制 |

### 9. `servol/dual_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servol/dual_arm |
| **Type** | geometry_msgs/Pose |
| **Direction** | 📥 Subscribe |
| **Description** | 双臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### 10. `servol/left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servol/left_arm |
| **Type** | geometry_msgs/Pose |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### 11. `servol/right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servol/right_arm |
| **Type** | geometry_msgs/Pose |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### 12. `speedj/dual_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/dual_arm |
| **Type** | [upperlimb/SpeedJ](../zj_humanoid_types#speedj) |
| **Direction** | 📤 Publish |
| **Description** | 双臂关节速度控制 |
| **Note** | 双臂关节空间速度控制 |

### 13. `speedj/enable_speedj`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/enable_speedj |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 启用speedj |
| **Note** | 启用关节空间速度控制 |

### 14. `speedj/left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/left_arm |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂speedj |
| **Note** | 关节空间速度控制 |

### 15. `speedj/lift`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/lift |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 升降speedj |
| **Note** | 关节空间速度控制 |

### 16. `speedj/neck`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/neck |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 脖子speedj |
| **Note** | 关节空间速度控制 |

### 17. `speedj/right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/right_arm |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂speedj |
| **Note** | 关节空间速度控制 |

### 18. `speedj/waist`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/waist |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 腰speedj |
| **Note** | 关节空间速度控制 |

### 19. `speedj/whole_body`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/whole_body |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 全身speedj |
| **Note** | 关节空间速度控制 |

### 20. `speedl/dual_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedl/dual_arm |
| **Type** | [upperlimb/SpeedL](../zj_humanoid_types#speedl) |
| **Direction** | 📥 Subscribe |
| **Description** | 双臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### 21. `speedl/left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedl/left_arm |
| **Type** | [upperlimb/SpeedL](../zj_humanoid_types#speedl) |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### 22. `speedl/right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedl/right_arm |
| **Type** | [upperlimb/SpeedL](../zj_humanoid_types#speedl) |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### 23. `tcp_pose/left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/tcp_pose/left_arm |
| **Type** | [upperlimb/Pose](../zj_humanoid_types#pose) |
| **Direction** | 📤 Publish |
| **Description** | 左臂tcp位姿控制 |
| **Note** | 左手臂末端位姿 |

### 24. `tcp_pose/right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/tcp_pose/right_arm |
| **Type** | [upperlimb/Pose](../zj_humanoid_types#pose) |
| **Direction** | 📤 Publish |
| **Description** | 右臂tcp位姿控制 |
| **Note** | 右手臂末端位姿 |

### 25. `tcp_speed`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/tcp_speed |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📤 Publish |
| **Description** | 双臂tcp速度控制 |
| **Note** | 左右手臂末端速度 |

### 26. `tcp_speed/dual_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/tcp_speed/dual_arm |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 订阅双臂TCP速度 |
| **Note** | 该话题发布双臂末端执行器(TCP)的实时速度信息 |

### 27. `uplimb_occupation`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/uplimb_occupation |
| **Type** | std_msgs/Int8 |
| **Direction** | 📤 Publish |
| **Description** | 发布上肢占用状态 |
| **Note** | 用于发布上肢占用状态信息 |

### 28. `uplimb_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/uplimb_state |
| **Type** | [upperlimb/UplimbState](../zj_humanoid_types#uplimbstate) |
| **Direction** | 📥 Subscribe |
| **Description** | 订阅上肢机器人命令状态 |
| **Note** | 该话题发布上肢机器人的当前命令状态信息 |

