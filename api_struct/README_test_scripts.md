# ZJ Humanoid ROS 测试脚本使用指南

## 概述

本目录包含为每个ROS接口自动生成的Python测试脚本：
- **topic_test.py**: 用于测试ROS Topics（以10Hz频率持续发布）
- **service_test.py**: 用于测试ROS Services（单次调用）

## 生成统计

- **Topic测试脚本**: 80个
- **Service测试脚本**: 82个
- **总计**: 162个测试脚本

## 文件结构

```
api_struct/
├── generate_ros_tests.py          # 生成器脚本（可修改）
├── zj_humanoid_interfaces.yaml    # 接口定义（数据源）
└── zj_humanoid/
    ├── audio/
    │   ├── asr_text/
    │   │   ├── topic.yaml         # 接口定义
    │   │   └── topic_test.py      # 自动生成的测试脚本
    │   └── LLM_chat/
    │       ├── service.yaml
    │       └── service_test.py
    └── upperlimb/
        └── movej/
            └── left_arm/
                ├── service.yaml
                ├── service_test.py
                └── left_arm_t_case1.yaml  # Demo数据文件
```

## 使用方法

### 1. Topic 测试脚本

Topic测试脚本会以10Hz频率持续发布消息，直到按Ctrl+C停止。

**使用示例**：

```bash
# 进入接口目录
cd api_struct/zj_humanoid/audio/asr_text

# 创建测试数据文件 test_data.yaml
cat > test_data.yaml << EOF
data: "Hello, this is a test message"
EOF

# 运行测试脚本
python3 topic_test.py test_data.yaml

# 输出示例：
# [INFO] Publishing to /zj_humanoid/audio/asr_text at 10Hz...
# [INFO] Message type: std_msgs/String
# [INFO] Data: data: "Hello, this is a test message"
# 
# (持续发布，按Ctrl+C停止)
```

### 2. Service 测试脚本

Service测试脚本会调用一次服务并显示结果。

**使用示例**：

```bash
# 进入接口目录
cd api_struct/zj_humanoid/upperlimb/movej/left_arm

# 使用已有的demo数据文件
python3 service_test.py left_arm_t_case1.yaml

# 或创建自定义请求数据
cat > my_request.yaml << EOF
joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: 0.5
acceleration: 0.3
EOF

python3 service_test.py my_request.yaml

# 输出示例：
# [INFO] Waiting for service /zj_humanoid/upperlimb/movej/left_arm...
# [INFO] Calling service /zj_humanoid/upperlimb/movej/left_arm...
# [INFO] Request: joint_positions: [0.0, 0.0, ...]
# [INFO] Response: success: True, message: "Motion completed"
#
# Service call successful!
# Response: success: True, message: "Motion completed"
```

### 3. 使用现有的Demo数据

许多接口目录已包含demo数据文件（如 `left_arm_t_case1.yaml`），可以直接使用：

```bash
# 查找某个接口的demo数据
cd api_struct/zj_humanoid/upperlimb/movej/left_arm
ls *.yaml

# 输出：
# service.yaml              # 接口定义
# left_arm_t_case1.yaml     # Demo数据1
# left_arm_t_case2.yaml     # Demo数据2

# 使用demo数据测试
python3 service_test.py left_arm_t_case1.yaml
```

## 数据格式说明

### Topic 数据格式

YAML文件应包含消息的所有字段：

```yaml
# 示例：std_msgs/String
data: "your message here"

# 示例：geometry_msgs/Twist
linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5
```

### Service 数据格式

YAML文件应包含服务请求的所有字段：

```yaml
# 示例：upperlimb/MoveJ
joint_positions: [0.0, -0.5, 0.3, 0.0, 1.2, 0.0, 0.0]
velocity: 0.5
acceleration: 0.3
time_from_start: 2.0
```

## 重新生成测试脚本

如果需要重新生成所有测试脚本（例如修改模板后）：

```bash
cd api_struct

# 运行生成器
python3 generate_ros_tests.py

# 输出：
# ============================================================
# ZJ Humanoid ROS Test Script Generator
# ============================================================
# 
# Loading interfaces from zj_humanoid_interfaces.yaml...
# Loaded 171 interfaces
# 
# Scanning zj_humanoid...
# ✓ Generated topic_test.py for /zj_humanoid/audio/asr_text
# ✓ Generated service_test.py for /zj_humanoid/upperlimb/movej/left_arm
# ...
# ============================================================
# Generation Summary:
#   Topic test scripts:   80
#   Service test scripts: 82
#   Total generated:      162
# ============================================================
```

## 自定义生成器

`generate_ros_tests.py` 是一个可修改的Python脚本，包含：

- **TOPIC_TEST_TEMPLATE**: Topic测试脚本的模板
- **SERVICE_TEST_TEMPLATE**: Service测试脚本的模板
- **生成逻辑**: 扫描目录和生成文件的逻辑

你可以根据需要修改模板或生成逻辑，然后重新运行生成器。

## 常见问题

### 1. 如何知道某个接口需要什么数据格式？

查看接口定义文件或参考 `types_package/zj_humanoid_types.md`：

```bash
# 查看接口定义
cat zj_humanoid/upperlimb/movej/left_arm/service.yaml

# 查看消息类型定义
grep -A 20 "upperlimb/MoveJ" types_package/zj_humanoid_types.md
```

### 2. 测试脚本报错 "No module named 'upperlimb'"

确保ROS工作空间已正确设置并source：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash  # 替换为你的工作空间路径
```

### 3. Service调用超时

确保ROS master正在运行且服务已启动：

```bash
# 检查ROS master
rosnode list

# 检查服务是否存在
rosservice list | grep zj_humanoid

# 查看服务信息
rosservice info /zj_humanoid/upperlimb/movej/left_arm
```

### 4. 如何批量测试多个接口？

可以创建一个批量测试脚本：

```bash
#!/bin/bash
# batch_test.sh

# 测试所有audio相关的topic
for dir in zj_humanoid/audio/*/; do
    if [ -f "$dir/topic_test.py" ] && [ -f "$dir/topic.yaml" ]; then
        echo "Testing $dir"
        timeout 5 python3 "$dir/topic_test.py" "$dir/test_data.yaml" || true
    fi
done
```

## 接口分类

### Audio 音频模块
- Topics: `asr_text`, `listen_state`, `microphone/audio_data`
- Services: `LLM_chat`, `listen`, `tts_service`, `media_play`

### Upperlimb 上肢模块
- Topics: `joint_states`, `tcp_pose/*`, `servoj/*`, `speedj/*`
- Services: `movej/*`, `movel/*`, `go_home/*`, `IK/*`, `FK/*`

### Lowerlimb 下肢模块
- Topics: `cmd_vel/*`, `debug_info`, `body_imu`
- Services: `versions`

### Hand 手部模块
- Topics: `joint_states`, `finger_pressures/*`, `wrist_force_sensor/*`
- Services: `gesture_switch/*`, `joint_switch/*`, `task_switch/*`

### Sensor 传感器模块
- Topics: `CAM_*/image_raw`, `realsense_*/color/image_raw`, `head_imu`
- Services: `CAM_*/camera_info`

### Navigation 导航模块
- Topics: `odom_info`, `map`, `navigation_status`
- Services: `version`

### Manipulation 操作模块
- Services: `execute_pick_task`, `pose_estimation_service`, `camera_calibration`

### Robot 机器人状态
- Topics: `robot_state`, `battery_info`, `joint_motor/temperatures`
- Services: `basic_info`, `set_robot_state/*`, `face_show/*`

## 相关文件

- `zj_humanoid_interfaces.yaml` - 所有接口的汇总定义
- `zj_humanoid_interfaces.json` - JSON格式的接口定义
- `zj_humanoid_interfaces.md` - Markdown格式的接口文档
- `types_package/zj_humanoid_types.md` - ROS消息类型定义
- `demos_test_shell.sh` - Shell版本的测试脚本

## 技术细节

### Topic测试脚本特性
- 使用 `rospy.Rate(10)` 实现10Hz发布频率
- 支持从YAML文件加载消息数据
- 自动将字典转换为ROS消息对象
- 优雅处理Ctrl+C中断

### Service测试脚本特性
- 使用 `rospy.wait_for_service()` 等待服务就绪
- 支持从YAML文件加载请求数据
- 自动将字典转换为服务请求对象
- 显示详细的请求和响应信息

### 生成器特性
- 从 `zj_humanoid_interfaces.yaml` 读取接口元数据
- 自动解析消息类型（package/MessageName）
- 生成可执行的Python脚本（chmod +x）
- 包含详细的文档字符串和注释

## 更新日志

- **2025-10-22**: 初始版本，生成162个测试脚本
  - 80个Topic测试脚本
  - 82个Service测试脚本
  - 支持10Hz Topic发布
  - 支持YAML数据文件输入


