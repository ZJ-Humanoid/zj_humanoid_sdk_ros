# zj_humanoid


## manipulation

> version:1.0.0


### MSG

#### `Grasp6d`

```bash
# ros message for one 6d grasp planning result

string label    # 物品名称
float32 score   # 评分
float32 width   # 抓取宽度
float32 depth   # 抓取深度
float32[] rotation_matrix  # 预测抓取在相机坐标系下的旋转矩阵，按行排列 [T_00, T_01, T_02, T_10, T_11, ...]，长度=9x1
float32[] translation  # 预测抓取在相机坐标系下的平移，按行排列 [T_x, T_y, T_z]，长度=3x1
```
#### `ObjPose`

```bash
# ros message for object pose 

string label  # 物品名称
geometry_msgs/Pose pose  # 物体位姿
```
#### `DetItem`

```bash
# ros message for one detect result

string label   # 搜索到的物品名称
float32 confidence   # 检测置信度
int16[4] bbox   # 检测边界框, [x1, y1, x2, y2]
sensor_msgs/Image mask   # 掩码图像

```

### SRV

#### `JointSpaceTrajPlan`

```bash
string which_arm                  # 手臂选择, "left" or "right"
sensor_msgs/JointState[] joint_states # 一系列的关节状态目标点（航点）
---
bool success                      # 是否成功
string message                    # 反馈信息
trajectory_msgs/JointTrajectory ros_trajectory # 规划生成的关节空间轨迹
```
#### `GraspTeach`

```bash
string which_arm  # 手臂选择，可选值：["left", "right"]
string object_label  # 物体名称
---
bool success  # 是否成功
string message  # 反馈信息
```
#### `InstSeg`

```bash
# instance segmentation service

string[] labels   # 物品名称列表
sensor_msgs/Image color_image    # 场景彩色图像
sensor_msgs/Image depth_image    # 场景深度图像
---
bool success  # 是否成功
string message  # 反馈信息
bool have_objs    # 是否检测到物体的标志
manipulation/DetItem[] items    # 分割消息，每个项目包括标签、置信度、边界框和掩码

```
#### `PoseEst`

```bash
# object pose estimation service 

sensor_msgs/Image color_image   # 场景彩色图像
sensor_msgs/Image depth_image   # 场景深度图像
manipulation/DetItem[] items    # 分割消息，每个项目包括标签、置信度、边界框和掩码
---
bool success  # 是否成功
string message  # 反馈信息
manipulation/ObjPose[] obj_poses   # 估计的物体位姿列表，包括物体标签和估计的位姿

```
#### `GetScenePose`

```bash
int16 scene_id  # 场景ID
---
bool[] success  # 是否成功
string message  # 反馈信息
string which_scene  # 场景名称
geometry_msgs/Pose[] scene_poses  # 场景位姿

```
#### `CameraCalibration`

```bash
string camera_name  # 相机名称，可选值：["sacrum_to_hand", "left_in_hand", "right_in_hand"]
string purpose  # 目的，可选值：["extrinsic", "intrinsic"]
string mode  # 模式，可选值：["from_folder", "run_trajectory"]
---
bool success  # 是否成功
string message  # 反馈信息
```
#### `SceneUpdate`

```bash
string[] obstacle_names  # 加载的模型名称
---
bool success  # 是否成功
string message  # 反馈信息
```
#### `PoseSpaceTrajPlan`

```bash
string which_arm               # 手臂选择, "left" or "right"
geometry_msgs/Pose[] waypoints # 目标路点（笛卡尔空间）
---
bool success                      # 是否成功
string message                    # 反馈信息
trajectory_msgs/JointTrajectory ros_trajectory # 规划生成的关节空间轨迹
```
#### `ExecutePickTask`

```bash
# Request
string target_label  # 要抓取的目标物体的标签，例如 "chip_can_ori"
---
# Response
bool success  # 是否成功
string message  # 反馈信息
```

### ACTION

#### `Pick`

```bash
# Goal
manipulation/ObjPose obj_pose  # 物体位姿
---
# Result
bool success  # 是否成功
string hand  # 哪只手抓取，right/left
string message  # 反馈信息
---
# Feedback
string message  # 反馈信息

```
#### `LoosenHand`

```bash
# Goal
string task_goal   # 任意传
---
# Result
bool success  # 是否成功
string message  # 反馈信息
---
# Feedback
bool loosen_hand_flag  # True表示机器人末端附近 xx cm 内有 人的手，false则表示没有

```
#### `Place`

```bash
# Goal 
manipulation/ObjPose obj_pose  # 物体位姿
---
# Result
bool success  # 是否成功
string message  # 反馈信息
---
# Feedback
string message  # 反馈信息

```
#### `SearchObject`

```bash
# Goal
string[] labels  # 要搜索的物品名称
---
# Result
bool success  # 是否成功
string message  # 反馈信息
---
# Feedback
string message  # 反馈信息

```
#### `InstSeg`

```bash
# Goal
string[] labels   # 要实例分割的物品名称
---
# Result
bool success  # 是否成功
string message  # 反馈信息
---
# Feedback
# bool have_objs
int32 status  # 状态
sensor_msgs/Image color_image    # 彩色图像
sensor_msgs/Image depth_image   # 深度图像
manipulation/DetItem[] items    # 检测到的物品
```
#### `Track`

```bash
# Goal
string label   # 要跟踪的物品名称
---
# Result
bool success  # 是否成功
string message  # 反馈信息
---
# Feedback
string message  # 反馈信息

```

## navigation

> version:1.0.0


### MSG

#### `NavigationStatus`

```bash
# NavigationStatus.msg

std_msgs/Header header

PlanState state

```
#### `TaskInfo`

```bash
# TaskInfo.msg

std_msgs/Header header

Waypoint[] waypoints
```
#### `PlanState`

```bash
# PlanState.msg


uint8 NONE = 0
uint8 STANDBY = 1
uint8 PLANNING = 2
uint8 RUNNING = 3
uint8 STOPPING = 4
uint8 FINISHED = 5
uint8 FAILURE = 6
uint8 value
```
#### `LocalMap`

```bash
# LocalMap.msg

std_msgs/Header header
nav_msgs/MapMetaData info
LocalMapData[] data
```
#### `Waypoint`

```bash
# Waypoint.msg

int32 id
geometry_msgs/Pose pose
int32 action
int32 audio 
```
#### `LocalMapData`

```bash
# LocalMapData.msg

bool occupancy # 是否占用
int8 semantic # 语义
bool dynamic # 是否是动态
float64 speed # 移动速度 m/s
float64 direction # 移动方向 [-pi, pi]
```

## hand

> version:1.0.0


### MSG

#### `PressureSensor`

```bash
Header header 
float64[] pressure          # 指尖压力传感器压力值 顺序依次为:[none,拇指,食指,中指,无名指,none] 0.1N
```

### SRV

#### `Gesture`

```bash
string[] gesture_name                   # 手势名称,大小写不敏感(当控制一只手时,索引0生效,当控制两只手时,索引0为左手,索引1为右手)
---
bool success                            # 执行结果,该结果只反映命令的调用结果,并不能代表动作是否执行到位   
string message                          # 提示信息







# -----------------------------------------------------------------
# gesture_name

# RESET
# ROCK
# ONE
# TWO
# THREE
# FOUR
# FIVE
# YEAH
# POINTING_UP
# THUMPS_UP
# ILOVEYOU
# BIU
# FUCK

# Tips
# 可能会存在当前手的状态如果直接控制手势运动的话,会出现无法运动到指定手势的情况
# 该数据只用来进行单次控制运动,无法通过设置手势数组状态进行连续的控制

# -----------------------------------------------------------------

```
#### `HandJoint`

```bash
float32[] q    # 关节数组;     [拇指弯曲,拇指摆动,食指弯曲,中指弯曲,无名指弯曲,小指弯曲] 单位:弧度 当使用双手
---
bool success   # indicate successful run of triggered service
string message # informational













# -----------------------------------------------------------------
# 手掌关节角度限位,如果发送超过关节角度限位的数据会被限制到限位内,并进行提示
# -----------------------------------------------------------------

```

## sensor

> version:1.0.0


### SRV

#### `CameraInfo`

```bash
---
bool success   # 是否成功
string message # 返回消息
string camera_name  # 相机名称/ID, ["CAM_A", "CAM_B", "CAM_C", "CAM_D"]
uint32 height  # 图像分辨率
uint32 width   # 图像分辨率
string distortion_model  # 失真模型, e.g., "plumb_bob", "radtan"
float64[] D  # 失真系数 D (k1, k2, p1, p2, k3, ...)
# [fx, 0,  cx]
# [0,  fy, cy]
# [0,  0,  1 ]
float64[9] K  # 内参矩阵 K (3x3), 以行主序展开为9个元素的数组
geometry_msgs/Transform T_cam_body  # 外参: 相机在机器人本体坐标系下的位姿 (T_cam_body: Transform from body frame to camera frame)
```

## audio

> version:1.0.0


### MSG

#### `AudioData`

```bash
# Audio_Data
std_msgs/Header header
int32 channel
int32 vad_status
bool is_final         # 是否是最终一帧
uint8[] audio_data    # 语音数据
```

### SRV

#### `GetVolume`

```bash
---
bool success
string message   # 消息
int32 status     # 状态码
int32 volume     # 音量
```
#### `GetDeviceList`

```bash

---
bool success
string message # 消息
int32 status   # 状态码
string[] devicelist  # 设备列表



```
#### `TTS`

```bash
string[] text # 文本
bool isPlay  # 是否播放
---
bool success
string message # 消息
int32 status # 状态码
string file_path # 文件路径

```
#### `LLMNLU`

```bash
# NLUService.srv

# 请求部分
string raw_input        # 原始输入文本
bool enable_context     # 启用上下文理解
bool enable_save        # 是否记录
string context_id       # 会话上下文ID

---
# 响应部分
bool success
string message   # 消息
int32 status            # 返回的状态码
nlu_result[] nlu_result  # NLU 识别结果

```
#### `SetVolume`

```bash
int32 volume   # 音量
---
bool success
string message # 消息
int32 status   # 状态码

```
#### `Listen`

```bash
int8 operation      #0:stop 1:start 2:wakeup
---
bool success
string message # 消息
int32 status

```
#### `SetDevice`

```bash
string name      # 设备名称
---
bool success
string message   # 消息
int32 status     # 状态码


```
#### `MediaPlay`

```bash
string file_path # 文件路径
---
bool success
string message   # 消息
int32 status     # 状态码

```
#### `LLMChat`

```bash
string raw_input          # 原始输入文本
bool   enable_context     # 启用上下文理解
bool   enable_save        # 是否记录
string context_id         # 会话上下文ID
---
bool success
string message   # 消息
int32 status     #标准接口状态
string response #文本生成结果

```

## upperlimb

> version:1.0.0


### MSG

#### `Pose`

```bash
Header header                          # 当前时间戳
geometry_msgs/Point position           # [x, y, z]
geometry_msgs/Quaternion quaternion    # 四元素
float64[3] rpy_rad                     # [r, p, y]     弧度
float64[3] rpy_deg                     # [r, p, y]     角度



# RPY  定轴X-Y-Z旋转  绕动轴Z-Y-X旋转
```
#### `SpeedL`

```bash
float64[] tcp_speed    # 目标末端速度，末端平动速度[m/s]，末端转动角速度[rad/s]
float64 acc            # 最大加速度




# vx1, vy1, vz1, vrx1, vry1, vrz1
# vx2, vy2, vz2, vrx2, vry2, vrz2
# vx1, vy1, vz1, vrx1, vry1, vrz1, vx2, vy2, vz2, vrx2, vry2, vrz2
```
#### `UplimbState`

```bash
Header header                      # 当前时间戳
int8 cmd_num                       # 当前运行状态码
string cmd_name                    # 当前运行状态名称
bool left_arm_is_singular          # 当前左臂是否奇异
bool right_arm_is_singular         # 当前右臂是否奇异







# -----------------------------------------------------------------
# 0为停止状态，
# 1为MOVEJ，
# 2为MOVEJ_PATH，
# 3为MOVEL_NULLSPACE，
# 4为MOVEL， 
# 5为MOVEL_PATH，
# 6为SPEEDJ，
# 7为SPEEDL，
# 8为SPEED_STOP，
# 9为MOVECSVFILE，
# 10为MOVEFOURIER，
# 11为MOVEJ_SPLINE，
# 12为TEACH，
# 13为SERVOJ，
# 14为SERVOL。
# -----------------------------------------------------------------

```
#### `Joints`

```bash
float64[] joint         # 由于会存在多个关节的同时运动，所以该数据长度不能为7
```
#### `TcpSpeed`

```bash
Header header
float64[6] left_arm          # 左臂TCP速度信息
float64[6] right_arm         # 右臂TCP速度信息






# -----------------------------------------------
# left_arm:  [vx, vy, vz, ωx, ωy, ωz] 
# right_arm: [vx, vy, vz, ωx, ωy, ωz]


# 其中 vx, vy, vz 单位为[m/s]
# 其中 ωx, ωy, ωz 单位为[rad/s]
```
#### `DualPose`

```bash
geometry_msgs/Pose left_arm_pose        # 左臂数据
geometry_msgs/Pose right_arm_pose       # 右臂数据
```
#### `SpeedJ`

```bash
float64[] joint_speed    # 目标关节速度
float64 acc              # 最大加速度       
int8 arm_type            # 机器人身体部位







# -----------------------------------------------------------------
# joints数据维度定义:
# 左臂:7维
# 右臂:7维
# 颈部:2维
# 腰部:有1维和2维,具体看实际的机器人型号
# 升降:1维

# 机器人型号维度定义:
# 仅双臂:14维
# 双臂+颈部:2维
# 双臂+颈部+腰部1维:17
# 双臂+颈部+腰部2维:18
# 双臂+颈部+腰部2维+升降:19
# -----------------------------------------------------------------


# -----------------------------------------------------------------
# arm_type 8421码定义:
# 左臂为1,右臂为2,颈部为4,半身腰为8,轮臂腰上下平移为16
# 
# 
# 不同机型可以使用的部位:
# - 半身带腰：左臂为1，右臂为2，颈部为4， 半身腰为8。
# - 半身不带腰：左臂为1，右臂为2，颈部为4。
# - 轮臂：左臂为1，右臂为2，颈部为4，轮臂腰为8，轮臂腰上下平移为16。
# -----------------------------------------------------------------
```

### SRV

#### `MoveJ`

```bash
float64[] joints                       # 对应的关节数据维度,详情参考下方
float64 v                              # 最大速度
float64 acc                            # 最大加速度
float64 t                              # 从当前位置到目标位置的总时长;当给定该参数的时候,忽略v和acc参数.
bool is_async                             # 执行方式 true:立即返回,并且可以被打断;false:阻塞执行,等待执行完成后返回.
int8 arm_type                          # 机器人身体部位。 仅当使用V1版本且全身控制时有用,用来指定当前要控制的目标部位,可以通过8421码进行叠加
---
bool success                            # 执行结果,该结果只反映命令的调用结果,并不能代表动作是否执行到位   
string message                          # 提示信息





# -----------------------------------------------------------------
# joints数据维度定义:
# 左臂:7维
# 右臂:7维
# 颈部:2维
# 腰部:有1维和2维,具体看实际的机器人型号
# 升降:1维

# 机器人型号维度定义:
# 仅双臂:14维
# 双臂+颈部:2维
# 双臂+颈部+腰部1维:17
# 双臂+颈部+腰部2维:18
# 双臂+颈部+腰部2维+升降:19
# -----------------------------------------------------------------


# -----------------------------------------------------------------
# arm_type 8421码定义:
# 左臂为1,右臂为2,颈部为4,半身腰为8,轮臂腰上下平移为16
# 
# 
# 不同机型可以使用的部位:
# - 半身带腰：左臂为1，右臂为2，颈部为4， 半身腰为8。
# - 半身不带腰：左臂为1，右臂为2，颈部为4。
# - 轮臂：左臂为1，右臂为2，颈部为4，轮臂腰为16，轮臂腰上下平移为8。
# -----------------------------------------------------------------
```
#### `MoveL`

```bash
geometry_msgs/Pose[2] pose              # 位姿数组,当控制单臂时始终使用索引为0的数据,当控制双臂时索引0为左臂,索引1为右臂 
float64 v                               # 最大关节速度  [rad/s]
float64 acc                             # 最大加速度    [rad/s^2]
bool is_async                              # 执行方式 true:立即返回,并且可以被打断;false:阻塞执行,等待执行完成后返回.
---
bool success                            # 执行结果,该结果只反映命令的调用结果,并不能代表动作是否执行到位   
string message                          # 提示信息





# 
# 注意
#   - 基坐标系位于URDF中的第一个连杆的坐标系，左臂末端坐标系位于“TCP_L”，右臂末端坐标系位于“TCP_R”。

```
#### `MoveJByPose`

```bash
geometry_msgs/Pose[2] pose                # 对应的目标位姿     当调用左臂或右臂服务时,始终使用索引为0的数据;当调用双臂服务时，索引0为左臂,索引1为右臂 。
float64[2] q7                             # 第七关节角度       当调用左臂或右臂服务时,始终使用索引为0的数据;当调用双臂服务时，索引0为左臂,索引1为右臂。
float64 v                                 # 最大速度
float64 acc                               # 最大加速度
bool is_async                                # 执行方式 true:立即返回,并且可以被打断;false:阻塞执行,等待执行完成后返回.
---
bool success                              # 执行结果,该结果只反映命令的调用结果,并不能代表动作是否执行到位   
string message                            # 提示信息





```
#### `SpeedL`

```bash
float64[] tcp_speed    # 目标末端速度，末端平动速度[m/s]，末端转动角速度[rad/s]
float64 acc            # 最大加速度
---
bool success
string message



# vx1, vy1, vz1, vrx1, vry1, vrz1
# vx2, vy2, vz2, vrx2, vry2, vrz2
# vx1, vy1, vz1, vrx1, vry1, vrz1, vx2, vy2, vz2, vrx2, vry2, vrz2
```
#### `IsSingular`

```bash
float64[7] joints                     # 手臂关节
---
bool success
string message
bool is_singular                      # 请求的手臂关节是否在奇异位置
```
#### `IK`

```bash
geometry_msgs/Pose pose           # tcp的位姿
float64 q7                        # 第七关节角
---
bool success                      # 是否有解
string message                    # 提示信息
int32 nums                        # 解的数量
upperlimb/Joints[] joints       # 关节角
float64[] phi                     # 臂角,索引与逆解后的关节角对应


```
#### `ArmType`

```bash
int8 arm_type               # 机器人类型,使用8421叠加使用
---
bool success
string message



# -----------------------------------------------------------------
# arm_type 8421码定义:
# 左臂为1,右臂为2,颈部为4,半身腰为8,轮臂腰上下平移为16
# 
# 
# 不同机型可以使用的部位:
# - 半身带腰：左臂为1，右臂为2，颈部为4， 半身腰为8。
# - 半身不带腰：左臂为1，右臂为2，颈部为4。
# - 轮臂：左臂为1，右臂为2，颈部为4，轮臂腰为16，轮臂腰上下平移为8。
# -----------------------------------------------------------------
```
#### `MoveJByPath`

```bash
upperlimb/Joints[] path               # 关节角路径,
float64 time                            # 运行总时间。  [s]    到达最后一个路径点所需的时间. 当指定总时间时 忽略时间戳参数
float64[] timestamp                     # 时间戳。     [s]    每一个关节角路径对应的时间戳
bool is_async                              # 是否同步运行
int8 arm_type                           # 机器人身体部位。 仅当使用V1版本且全身控制时有用,用来指定当前要控制的目标部位,可以通过8421码进行叠加
---
bool success                            # 执行结果,该结果只反映命令的调用结果,并不能代表动作是否执行到位   
string message                          # 提示信息





# -----------------------------------------------------------------
# arm_type 8421码定义:
# 左臂为1,右臂为2,颈部为4,半身腰为8,轮臂腰上下平移为16
# 
# 
# 不同机型可以使用的部位:
# - 半身带腰：左臂为1，右臂为2，颈部为4， 半身腰为8。
# - 半身不带腰：左臂为1，右臂为2，颈部为4。
# - 轮臂：左臂为1，右臂为2，颈部为4，轮臂腰为16，轮臂腰上下平移为8。
# -----------------------------------------------------------------

```
#### `FK`

```bash
float64[] joints                # 关节角
---
bool success
string message
geometry_msgs/Pose pose         # tcp的位姿



# 关节角需要不全所有的数据
```
#### `Servo`

```bash
float64 v                       # 最大速度, 最大速度，此版本无效，默认写入0.0。  [rad/s]
float64 acc                     # 最大加速度，此版本无效，默认写入0.0。 [rad/s^2]
float64 time                    # 执行时间, 间隔点之间的运行时间
float64 lookahead_time          # 前瞻时间, 此版本无效, 默认写入0.2
int32 gain                      # 位置跟踪参数，该参数越大，跟踪越慢，超调量越小，范围[100, 1000]
int8 arm_type                   # 机器人身体部位。 仅当使用V1版本且全身控制时有用,用来指定当前要控制的目标部位,可以通过8421码进行叠加
---
bool success
string message





# -----------------------------------------------------------------
# arm_type 8421码定义:
# 左臂为1,右臂为2,颈部为4,半身腰为8,轮臂腰上下平移为16
# 
# 
# 不同机型可以使用的部位:
# - 半身带腰：左臂为1，右臂为2，颈部为4， 半身腰为8。
# - 半身不带腰：左臂为1，右臂为2，颈部为4。
# - 轮臂：左臂为1，右臂为2，颈部为4，轮臂腰为8，轮臂腰上下平移为16。
# -----------------------------------------------------------------
```
#### `MoveLByPath`

```bash
geometry_msgs/Pose[] left_arm_path              # 位姿数组,当控制单臂时始终使用索引为0的数据,当控制双臂时索引0为左臂,索引1为右臂 
geometry_msgs/Pose[] right_arm_path             # 位姿数组,当控制单臂时始终使用索引为0的数据,当控制双臂时索引0为左臂,索引1为右臂 
float64 time                                    # 运行总时间 [s]    当指定总时间时 忽略时间戳参数
float64[] timestamp                             # 每一个关节角路径对应的时间戳 [s]
bool is_async                                      # 是否同步运行
---
bool success
string message
```

## robot

> version:1.0.0


### MSG

#### `Errors`

```bash
bool over_temp
bool over_cpu
bool over_mem
bool over_disk
```
#### `ModulesMonitor`

```bash
# 上肢模块状态
bool upper_limb_ok
# 灵巧手模块状态
bool dexterous_hand_ok
# 下肢模块状态
bool lower_limb_ok
# 四目相机（CAM_A - CAM_D）状态
bool cam_a_ok
bool cam_b_ok
bool cam_c_ok
bool cam_d_ok
# 深度相机（realsense_up、realsense_down）状态
bool realsense_up_ok
bool realsense_down_ok
# 定位模块状态
bool localization_module_ok
# 导航模块状态
bool navigation_module_ok
# 语音模块状态
bool voice_module_ok
```
#### `BatteryInfo`

```bash
# 电池基本信息
uint8 battery_type          # 电池类型  1 - 鼎力2014b, 2 - 鼎力2015型, 3 - 云帆

# LED状态
uint8 led1_status           # 0x01 - 绿灯常亮，0x02 - 红灯常亮，0x03 - 绿灯闪烁，0x04 - 红灯闪烁, 0x05 - 红绿闪烁，其他 - 灭
uint8 led2_status           # 同上

# 电压电流信息
float32 total_voltage       # 电池组总电压，单位V
float32 total_current       # 电池组总电流，单位A

# 容量信息
uint16 soc                  # 剩余容量 0-1000表示0%-100%
float32 remaining_capacity  # 剩余容量，单位Ah
uint32 remaining_time       # 剩余充电时间, 单位分钟

# 单体电池电压信息
float32 max_cell_voltage    # 最高单体电压, 单位V
float32 min_cell_voltage    # 最低单体电压, 单位V
uint8 max_voltage_cell_num  # 最高单体电压电池位置
uint8 min_voltage_cell_num  # 最低单体电压电池位置
float32 voltage_diff        # 单体最高最低电芯压差，单位V

# 温度信息
int16 max_cell_temperature  # 最高电芯温度，单位摄氏度
int16 min_cell_temperature  # 最低电芯温度，单位摄氏度
int16 avg_cell_temperature  # 电芯平均温度，单位摄氏度
uint8 max_temp_cell_num     # 最高温度电池位置
uint8 min_temp_cell_num     # 最低温度电池位置
float32 temperature_diff    # 单体最高最低温度差，单位摄氏度

# 状态信息
uint8 operation_status      # 充放电状态： 0 - 静置， 1 - 充电， 2 - 放电
uint16 cycle_count          # 电池循环次数

# 报警和状态标志
bool is_temperature_high    # 电池温度是否过高
bool is_battery_low         # 电池电量是否过低
uint32[16] warnings         # 报警信息位

# 版本信息
string version              # 版本号
```
#### `WorkStatus`

```bash
int32 remain_work_time  # 剩余工作时长，单位分钟
int32 work_time         # 运行时间，单位分钟
float32 walk_distance   # 行进里程
```
#### `Resource`

```bash
float32 cpu_usage
float32 temperature
float32 memory_usage
float32 disk_usage
```
#### `RobotState`

```bash
uint8 state         # State enums ; state: 5
string state_info   # 打印目前获取的状态 ；state_info: "STATE_ROBOT_RUN"

# State enums
uint8 STATE_ROBOT_NULL = 0
uint8 STATE_ROBOT_CONFIG = 1
uint8 STATE_ROBOT_ON = 2
uint8 STATE_ROBOT_START = 3
uint8 STATE_ROBOT_INIT = 4
uint8 STATE_ROBOT_RUN = 5
uint8 STATE_ROBOT_HALT = 6
uint8 STATE_ROBOT_STOP = 7
uint8 STATE_ROBOT_OFF = 8
uint8 STATE_ROBOT_ERR = 9
```

### SRV

#### `WifiList`

```bash
# 请求部分（无参数）
---
# 响应部分
bool success             # 操作是否成功
string message           # 状态描述（成功/错误信息）
int32 wifi_count         # WiFi热点数量
string[] wifi_names      # WiFi信息数组，格式："BSSID|SSID"（例如："14:D8:64:73:95:0C|teleoperation_1"）
```
#### `FaceScreen`

```bash
# text_display.srv
string text           # 要展示的内容，可以为空
bool force_stop       # 是否强制终止当前显示
---
bool success          # 是否成功
string message        # 返回提示或异常信息
int32 status_code     #状态码（0 = 成功，1 = 无活跃显示，2 = 显示冲突，3 = 初始化失败）

```
#### `BasicInfo`

```bash
---
bool success             # 是否成功
string message           # 返回消息（如成功或失败的原因）
string robot_version     # 机器人版本
string hardware_version  # 硬件版本
string software_version  # 软件版本
string ip_addr           # IP地址
```
#### `ConnectWifi`

```bash
string wifi_name
string wifi_password
---
bool success
string message
```
#### `SetZero`

```bash
# 标零服务消息定义
# Request
int32[] joint_ids    # 要标零的关节ID数组 (0-28)
---
# Response
bool success         # 标零是否成功
string message       # 详细信息



# =============================================================================
# 关节名称到算法编号的映射 (注释，供参考)
# =============================================================================
# leftShoulderPitch: 0,    leftShoulderRoll: 1,     leftShoulderYaw: 2
# leftElbow: 3,           leftWristYaw: 4,         leftWristPitch: 5,        leftWristRoll: 6
# rightShoulderPitch: 7,   rightShoulderRoll: 8,    rightShoulderYaw: 9
# rightElbow: 10,         rightWristYaw: 11,       rightWristPitch: 12,      rightWristRoll: 13
# neckYaw: 14,            neckPitch: 15,           waist: 16
# leftHipYaw: 17,         leftHipRoll: 18,         leftHipPitch: 19,         leftKnee: 20
# leftAnklePitch: 21,     leftAnkleRoll: 22
# rightHipYaw: 23,        rightHipRoll: 24,        rightHipPitch: 25,        rightKnee: 26
# rightAnklePitch: 27,    rightAnkleRoll: 28
```
#### `FaceShow`

```bash
# 请求部分：指定要播放的媒体信息
string media_path    # 媒体文件路径，可以是图片或视频
bool loop            # 是否循环播放，true为循环，false为播放一次
float32 duration       # 播放时长(秒)，0表示直到结束
---
# 响应部分：返回播放操作的结果
bool success         # 操作是否成功
string message       # 状态信息，如成功或错误原因
int32 status_code    # 状态码，0表示成功，非0表示错误类型

```
