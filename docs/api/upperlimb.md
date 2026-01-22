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
## 📦 Services (53)
- clear_servo_params
- enable_speedj
- enable_speedl
- safety_lock
- set_servo_params
- stop
- unlock
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
- is_singular
  - left_arm
  - right_arm
- motion
  - lists
  - load
  - loaded_lists
  - unload
- movej
  - dual_arm
  - left_arm
  - lifting
  - neck
  - right_arm
  - waist
  - whole_body
- movej_by_path
  - dual_arm
  - left_arm
  - lifting
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
- movel_by_path
  - dual_arm
  - left_arm
  - right_arm
- teach_mode
  - enter
  - exit
## 📡 Topics (29)
- joint_states
- occupancy_state
- uplimb_occupation
- uplimb_state
- jacobian
  - left_arm
  - right_arm
- servoj
  - dual_arm
  - left_arm
  - lifting
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
  - left_arm
  - lifting
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

## 📦 Services (53)

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
| **Description** | 关节速度控制开关 |
| **Note** | 启用或禁用上肢speedj速度控制模式 |

### 7. `enable_speedl`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/enable_speedl |
| **Type** | std_srvs/SetBool |
| **Description** | 启用笛卡尔空间速度控制 |
| **Note** | 启用或禁用上肢speedl笛卡尔空间速度控制模式 |

### 8. `go_down/dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_down/dual_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 双臂放下 |

### 9. `go_down/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_down/left_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 左臂放下 |

### 10. `go_down/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_down/right_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 右臂放下 |

### 11. `go_home/dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/dual_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 双臂回到home点 |
| **Note** | 双臂回到内置设置的home点 |

### 12. `go_home/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/left_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 左臂回到home点 |
| **Note** | 左臂回到内置设置的home点 |

### 13. `go_home/lifting`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/lifting |
| **Type** | std_srvs/Trigger |
| **Description** | 升降回到home点 |
| **Note** | 升降回到内置设置的home点 |

### 14. `go_home/neck`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/neck |
| **Type** | std_srvs/Trigger |
| **Description** | 脖子回到home点 |
| **Note** | 脖子回到内置设置的home点 |

### 15. `go_home/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/right_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 右臂回到home点 |
| **Note** | 右臂回到内置设置的home点 |

### 16. `go_home/waist`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/waist |
| **Type** | std_srvs/Trigger |
| **Description** | 腰部回到home点 |
| **Note** | 腰部回到内置设置的home点 |

### 17. `go_home/whole_body`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/go_home/whole_body |
| **Type** | [upperlimb/ArmType](../zj_humanoid_types#armtype) |
| **Description** | 全身回到home点 |
| **Note** | 全身指定部位回到内置设置的home点 |

### 18. `is_singular/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/is_singular/left_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 检查左臂奇异点 |
| **Note** | 检查左臂当前位置是否处于奇异点配置 |

### 19. `is_singular/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/is_singular/right_arm |
| **Type** | std_srvs/Trigger |
| **Description** | 检查右臂奇异点 |
| **Note** | 检查右臂当前位置是否处于奇异点配置 |

### 20. `motion/lists`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/motion/lists |
| **Type** | [upperlimb/MotionLists](../zj_humanoid_types#motionlists) |
| **Description** | 获取动作列表 |
| **Note** | 获取可用的预定义动作列表 |

### 21. `motion/load`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/motion/load |
| **Type** | [upperlimb/MotionLoad](../zj_humanoid_types#motionload) |
| **Description** | 加载动作文件 |
| **Note** | 从文件加载预定义动作到内存 |

### 22. `motion/loaded_lists`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/motion/loaded_lists |
| **Type** | [upperlimb/MotionLoadedLists](../zj_humanoid_types#motionloadedlists) |
| **Description** | 获取已加载动作列表 |
| **Note** | 获取当前已加载到内存的动作列表 |

### 23. `motion/unload`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/motion/unload |
| **Type** | [upperlimb/MotionUnload](../zj_humanoid_types#motionunload) |
| **Description** | 卸载动作文件 |
| **Note** | 从内存中卸载指定的预定义动作 |

### 24. `movej/dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/dual_arm |
| **Type** | [upperlimb/MoveJ](../zj_humanoid_types#movej) |
| **Description** | 双臂movej |
| **Note** | 关节空间下,双臂点到点运动 |

### 25. `movej/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/left_arm |
| **Type** | [upperlimb/MoveJ](../zj_humanoid_types#movej) |
| **Description** | 左臂movej |
| **Note** | 关节空间下,左臂点到点运动 |

### 26. `movej/lifting`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/lifting |
| **Type** | [upperlimb/MoveJ](../zj_humanoid_types#movej) |
| **Description** | 升降movej |
| **Note** | 关节空间下,升降点到点运动 |

### 27. `movej/neck`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/neck |
| **Type** | [upperlimb/MoveJ](../zj_humanoid_types#movej) |
| **Description** | 脖子movej |
| **Note** | 关节空间下,脖子点到点运动 |

### 28. `movej/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/right_arm |
| **Type** | [upperlimb/MoveJ](../zj_humanoid_types#movej) |
| **Description** | 右臂movej |
| **Note** | 关节空间下,右臂点到点运动 |

### 29. `movej/waist`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/waist |
| **Type** | [upperlimb/MoveJ](../zj_humanoid_types#movej) |
| **Description** | 腰部movej |
| **Note** | 关节空间下,腰部点到点运动 |

### 30. `movej/whole_body`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej/whole_body |
| **Type** | [upperlimb/MoveJ](../zj_humanoid_types#movej) |
| **Description** | 全身movej |
| **Note** | 关节空间下,全身各部位点到点运动 |

### 31. `movej_by_path/dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/dual_arm |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 双臂轨迹movej |
| **Note** | 关节空间下,双臂轨迹点路径运动 |

### 32. `movej_by_path/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/left_arm |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 左臂轨迹movej |
| **Note** | 关节空间下,左臂轨迹点路径运动 |

### 33. `movej_by_path/lifting`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/lifting |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 关节空间下,脖子轨迹点路径运动 |
| **Note** | 控制颈部按照关节空间路径运动 |

### 34. `movej_by_path/neck`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/neck |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 关节空间下,脖子轨迹点路径运动 |
| **Note** | 控制颈部按照关节空间路径运动 |

### 35. `movej_by_path/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/right_arm |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 右臂轨迹movej |
| **Note** | 关节空间下,右臂轨迹点路径运动 |

### 36. `movej_by_path/waist`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/waist |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 关节空间下,腰部轨迹点路径运动 |
| **Note** | 控制腰部按照关节空间路径运动 |

### 37. `movej_by_path/whole_body`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_path/whole_body |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 关节空间下,全身轨迹点路径运动 |
| **Note** | 控制全身按照关节空间路径运动 |

### 38. `movej_by_pose/dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_pose/dual_arm |
| **Type** | [upperlimb/MoveJByPose](../zj_humanoid_types#movejbypose) |
| **Description** | 双臂末端movej |
| **Note** | tcp末端空间下,双臂末端位姿movej |

### 39. `movej_by_pose/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_pose/left_arm |
| **Type** | [upperlimb/MoveJByPose](../zj_humanoid_types#movejbypose) |
| **Description** | 左臂末端movej |
| **Note** | tcp末端空间下,左臂末端位姿movej |

### 40. `movej_by_pose/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movej_by_pose/right_arm |
| **Type** | [upperlimb/MoveJByPose](../zj_humanoid_types#movejbypose) |
| **Description** | 右臂末端movej |
| **Note** | tcp末端空间下,右臂末端位姿movej |

### 41. `movel/dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movel/dual_arm |
| **Type** | [upperlimb/MoveL](../zj_humanoid_types#movel) |
| **Description** | 双臂movel |
| **Note** | 关节空间下,双臂直线轨迹点运动 |

### 42. `movel/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movel/left_arm |
| **Type** | [upperlimb/MoveL](../zj_humanoid_types#movel) |
| **Description** | 左臂movel |
| **Note** | 关节空间下,左臂直线轨迹点运动 |

### 43. `movel/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movel/right_arm |
| **Type** | [upperlimb/MoveL](../zj_humanoid_types#movel) |
| **Description** | 右臂movel |
| **Note** | 关节空间下,右臂直线轨迹点运动 |

### 44. `movel_by_path/dual_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movel_by_path/dual_arm |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 双臂轨迹movej |
| **Note** | 关节空间下,双臂轨迹点路径运动 |

### 45. `movel_by_path/left_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movel_by_path/left_arm |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 左臂轨迹movej |
| **Note** | 关节空间下,左臂轨迹点路径运动 |

### 46. `movel_by_path/right_arm`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/movel_by_path/right_arm |
| **Type** | [upperlimb/MoveJByPath](../zj_humanoid_types#movejbypath) |
| **Description** | 右臂轨迹movej |
| **Note** | 关节空间下,右臂轨迹点路径运动 |

### 47. `safety_lock`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/safety_lock |
| **Type** | std_srvs/Trigger |
| **Description** | 安全锁定 |
| **Note** | 启用上肢安全锁定,防止意外运动 |

### 48. `set_servo_params`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/set_servo_params |
| **Type** | [upperlimb/Servo](../zj_humanoid_types#servo) |
| **Description** | 设置伺服参数 |
| **Note** | 设置上肢伺服参数配置,包括时间和增益参数 |

### 49. `stop`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/stop |
| **Type** | std_srvs/Trigger |
| **Description** | 停止上肢运动 |
| **Note** | 立即停止上肢所有运动 |

### 50. `teach_mode/enter`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/teach_mode/enter |
| **Type** | [upperlimb/ArmType](../zj_humanoid_types#armtype) |
| **Description** | 进入示教模式 |

### 51. `teach_mode/exit`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/teach_mode/exit |
| **Type** | [upperlimb/ArmType](../zj_humanoid_types#armtype) |
| **Description** | 退出示教模式 |

### 52. `unlock`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/unlock |
| **Type** | std_srvs/Trigger |
| **Description** | 解除锁定 |
| **Note** | 解除上肢安全锁定,允许运动控制 |

### 53. `version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/upperlimb/version |
| **Type** | std_srvs/Trigger |
| **Description** | 上肢版本信息 |
| **Note** | 查询上肢控制模块的版本信息 |

## 📡 Topics (29)

### 1. `jacobian/left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/jacobian/left_arm |
| **Type** | std_msgs/Float64MultiArray |
| **Direction** | 📤 Publish |
| **Description** | 左臂雅可比矩阵 |
| **Note** | 发布左臂当前位置的雅可比矩阵,用于速度和力的映射 |

### 2. `jacobian/right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/jacobian/right_arm |
| **Type** | std_msgs/Float64MultiArray |
| **Direction** | 📤 Publish |
| **Description** | 右臂雅可比矩阵 |
| **Note** | 发布右臂当前位置的雅可比矩阵,用于速度和力的映射 |

### 3. `joint_states`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/joint_states |
| **Type** | sensor_msgs/JointState |
| **Direction** | 📤 Publish |
| **Description** | 上肢关节位置 |
| **Note** | 机器人上肢关节position状态值发布，查询当前机器人颈部pitch的角度 回复应处于+-42度间 |

### 4. `occupancy_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/occupancy_state |
| **Type** | std_msgs/Int8 |
| **Direction** | 📥 Subscribe |
| **Description** | 上肢占用控制 |
| **Note** | 该话题发布上肢的当前占用状态,用于防止多个控制源同时控制机器人 |

### 5. `servoj/dual_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/dual_arm |
| **Type** | [upperlimb/Joints](../zj_humanoid_types#joints) |
| **Direction** | 📤 Publish |
| **Description** | 双臂servoj |
| **Note** | 双臂关节空间伺服控制,不要使用定时sleep,该接口执行需要准确的时间戳会达到更好的效果 |

### 6. `servoj/left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/left_arm |
| **Type** | [upperlimb/Joints](../zj_humanoid_types#joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂servoj |
| **Note** | 关节空间 高频位置控制 |

### 7. `servoj/lifting`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/lifting |
| **Type** | [upperlimb/Joints](../zj_humanoid_types#joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 全身servoj |
| **Note** | 关节空间 高频位置控制 |

### 8. `servoj/neck`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/neck |
| **Type** | [upperlimb/Joints](../zj_humanoid_types#joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 颈部servoj |
| **Note** | 关节空间 高频位置控制 |

### 9. `servoj/right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/right_arm |
| **Type** | [upperlimb/Joints](../zj_humanoid_types#joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂servoj |
| **Note** | 关节空间 高频位置控制 |

### 10. `servoj/waist`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/waist |
| **Type** | [upperlimb/Joints](../zj_humanoid_types#joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 腰部servoj |
| **Note** | 关节空间 高频位置控制 |

### 11. `servoj/whole_body`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servoj/whole_body |
| **Type** | [upperlimb/Joints](../zj_humanoid_types#joints) |
| **Direction** | 📥 Subscribe |
| **Description** | 全身servoj |
| **Note** | 关节空间 高频位置控制 |

### 12. `servol/dual_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servol/dual_arm |
| **Type** | geometry_msgs/Pose |
| **Direction** | 📥 Subscribe |
| **Description** | 双臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### 13. `servol/left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servol/left_arm |
| **Type** | geometry_msgs/Pose |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### 14. `servol/right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/servol/right_arm |
| **Type** | geometry_msgs/Pose |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂servol |
| **Note** | 笛卡尔空间 高频位置跟随控制 |

### 15. `speedj/dual_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/dual_arm |
| **Type** | [upperlimb/SpeedJ](../zj_humanoid_types#speedj) |
| **Direction** | 📤 Publish |
| **Description** | 双臂关节speedj |
| **Note** | 双臂关节空间速度控制 |

### 16. `speedj/left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/left_arm |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂speedj |
| **Note** | 关节空间速度控制 |

### 17. `speedj/lifting`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/lifting |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 升降speedj |
| **Note** | 关节空间速度控制 |

### 18. `speedj/neck`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/neck |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 脖子speedj |
| **Note** | 关节空间速度控制 |

### 19. `speedj/right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/right_arm |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂speedj |
| **Note** | 关节空间速度控制 |

### 20. `speedj/waist`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/waist |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 腰speedj |
| **Note** | 关节空间速度控制 |

### 21. `speedj/whole_body`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedj/whole_body |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 全身speedj |
| **Note** | 关节空间速度控制 |

### 22. `speedl/dual_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedl/dual_arm |
| **Type** | [upperlimb/SpeedL](../zj_humanoid_types#speedl) |
| **Direction** | 📥 Subscribe |
| **Description** | 双臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### 23. `speedl/left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedl/left_arm |
| **Type** | [upperlimb/SpeedL](../zj_humanoid_types#speedl) |
| **Direction** | 📥 Subscribe |
| **Description** | 左臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### 24. `speedl/right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/speedl/right_arm |
| **Type** | [upperlimb/SpeedL](../zj_humanoid_types#speedl) |
| **Direction** | 📥 Subscribe |
| **Description** | 右臂speedl |
| **Note** | 笛卡尔空间 速度控制 |

### 25. `tcp_pose/left_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/tcp_pose/left_arm |
| **Type** | [upperlimb/Pose](../zj_humanoid_types#pose) |
| **Direction** | 📤 Publish |
| **Description** | 左臂tcp位姿控制 |
| **Note** | 左手臂末端位姿 |

### 26. `tcp_pose/right_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/tcp_pose/right_arm |
| **Type** | [upperlimb/Pose](../zj_humanoid_types#pose) |
| **Direction** | 📤 Publish |
| **Description** | 右臂tcp位姿控制 |
| **Note** | 右手臂末端位姿 |

### 27. `tcp_speed/dual_arm`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/tcp_speed/dual_arm |
| **Type** | [upperlimb/TcpSpeed](../zj_humanoid_types#tcpspeed) |
| **Direction** | 📥 Subscribe |
| **Description** | 双臂TCP速度 |
| **Note** | 该话题发布双臂末端执行器(TCP)的实时速度信息 |

### 28. `uplimb_occupation`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/uplimb_occupation |
| **Type** | std_msgs/Int8 |
| **Direction** | 📤 Publish |
| **Description** | 上肢占用状态 |
| **Note** | 用于发布上肢占用状态信息 |

### 29. `uplimb_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/upperlimb/uplimb_state |
| **Type** | [upperlimb/UplimbState](../zj_humanoid_types#uplimbstate) |
| **Direction** | 📥 Subscribe |
| **Description** | 当前上肢命令 |
| **Note** | 该话题发布上肢机器人的当前命令状态信息 |

