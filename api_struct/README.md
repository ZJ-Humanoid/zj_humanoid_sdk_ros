# ZJ Humanoid ROS API 测试工具

这个目录包含了用于测试 ZJ Humanoid ROS API 的工具和脚本。

## 文件说明

- `zj_humanoid_interfaces.json` - ROS API 接口定义文件
- `zj_humanoid_ros_test.py` - 主要的测试脚本（功能完整）
- `simple_ros_caller.py` - 简化版本的服务调用脚本
- `zj_humanoid/` - 包含所有 ROS API 的 YAML 配置和示例数据

## 使用方法

### 1. 基本服务调用

```bash
python zj_humanoid_ros_test.py <service_name> <yaml_file>
```

**示例：**
```bash
# 调用左手关节控制服务
python zj_humanoid_ros_test.py /zj_humanoid/hand/joint_switch/left left_hand_joint_reset.yaml

# 调用音频服务
python zj_humanoid_ros_test.py /zj_humanoid/audio/listen hello_world.yaml
```

### 2. 使用简化版本

```bash
python simple_ros_caller.py <service_name> <yaml_file>
```

## 功能特性

### 🔍 智能文件查找
脚本会自动在以下位置查找 YAML 文件：
- 当前目录
- 服务对应的目录
- 递归搜索整个 `zj_humanoid/` 目录

### 📊 数据验证
- 自动加载和验证 YAML 数据格式
- 显示即将发送的数据内容
- 检查服务类型和描述信息

### 🎭 模拟模式
当 ROS 环境不可用时，脚本会自动切换到模拟模式：
- 显示服务调用的详细信息
- 验证 YAML 数据格式
- 模拟服务调用过程

### 🔧 ROS 集成
在 ROS 环境可用时：
- 检查服务是否在线
- 获取实际的服务类型
- 执行真实的服务调用

## 常用服务示例

### 手部控制

```bash
# 左手关节重置
python zj_humanoid_ros_test.py /zj_humanoid/hand/joint_switch/left left_hand_joint_reset.yaml

# 右手手势控制
python zj_humanoid_ros_test.py /zj_humanoid/hand/gesture_switch/right right_hand_gesture_switch.yaml

# 双手关节控制
python zj_humanoid_ros_test.py /zj_humanoid/hand/joint_switch/dual dual_hand_joint_reset.yaml
```

### 上肢控制

```bash
# 左臂回家位置
python zj_humanoid_ros_test.py /zj_humanoid/upperlimb/go_home/left_arm go_home_left_arm.yaml

# 右臂关节运动
python zj_humanoid_ros_test.py /zj_humanoid/upperlimb/movej/right_arm movej_right_arm_case1.yaml
```

### 音频服务

```bash
# 语音识别
python zj_humanoid_ros_test.py /zj_humanoid/audio/listen hello_world.yaml

# 语音合成
python zj_humanoid_ros_test.py /zj_humanoid/audio/tts_service tts_data.yaml
```

## 错误处理

脚本包含完善的错误处理机制：

- **文件未找到**: 会提示可能的文件位置
- **YAML 格式错误**: 显示具体的解析错误
- **服务不可用**: 自动切换到模拟模式
- **ROS 环境问题**: 提供详细的错误信息和建议

## 依赖要求

### 基本依赖
- Python 3.6+
- PyYAML
- 标准库模块 (json, os, sys, argparse)

### ROS 依赖（可选）
- rospy
- rosservice
- std_srvs

如果没有安装 ROS，脚本会自动使用模拟模式。

## 安装依赖

```bash
pip install pyyaml
```

对于 ROS 依赖，请按照 ROS 官方文档进行安装。

## 开发和扩展

脚本采用模块化设计，易于扩展：

- `ZJHumanoidROSTester` 类包含所有核心功能
- 支持自定义接口定义文件
- 可以轻松添加新的服务类型支持
- 支持插件式的数据处理器

## 故障排除

### 常见问题

1. **编码错误**: 脚本会自动尝试多种编码格式
2. **路径问题**: 确保在正确的目录中运行脚本
3. **权限问题**: 确保有读取文件的权限

### 调试建议

检查 `zj_humanoid_interfaces.json` 文件确认服务名称是否正确，或查看相应的 YAML 配置文件。

## 联系信息

如有问题或建议，请联系开发团队。
