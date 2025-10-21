# ZJ Humanoid ROS API 测试工具

这个目录包含了用于测试 ZJ Humanoid ROS API 的工具和脚本。

## 文件说明

- `zj_humanoid/` - 包含所有 ROS API 的 YAML 配置和示例数据, 文件规则如下：
    - 路径表示service或topic name
    - 如路经下包含topic.yaml这表示该ros api是topic，同时文件内存储了topic的信息
    - 如路经下包含service.yaml这表示该ros api是service，同时文件内存储了service的信息
- `zj_humanoid_interfaces.json` - ROS API 接口定义文件，它是由j_humanoid/下的topic和service的yaml转换为

- `demos_ros_test.py` - API demos test， by python3，包含 `ZJHumanoidROSTester` 和 `ROSServiceCaller` 两个核心类

## 使用命令行调用API Demos

### rosservice
    rosservice call /zj_humanoid/upperlimb/movej/left_arm "$(cat zj_humanoid/upperlimb/movej/left_arm/left_arm_t_case1.yaml)"
### demos_shell_test.sh
    sh demos_shell_test.sh zj_humanoid/upperlimb/movej/left_arm/left_arm_t_case1.yaml

## demos_ros_test.py 使用方法

### 1. 基本服务调用

python3 demos_ros_test.py <yaml_path> 
或
python3 demos_ros_test.py <yaml_path> --service <service_name>

`yaml_file` 是必选参数，表示 `rosservice call` 的请求数据；
YAML 文件所在的路径包含隐含的 `service_name`。
如 `zj_humanoid/audio/listen/hello_world.yaml` 表示调用 `/zj_humanoid/audio/listen` 服务。
`--service` 是可选参数，如果该参数存在，表示显式指定 `service_name`，覆盖隐式指定的服务名。

**示例：**
```bash
# 调用左手手掌控制服务
python3 demos_ros_test.py zj_humanoid/hand/joint_switch/left/left_hand_joint_reset.yaml

# 调用左手手臂控制服务
python3 demos_ros_test.py zj_humanoid/upperlimb/movej/left_arm/left_arm_t_case1.yaml

# 调用音频服务
python3 demos_ros_test.py zj_humanoid/audio/listen/hello_world.yaml --service /zj_humanoid/audio/listen
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


### 🔧 ROS 集成
在 ROS 环境可用时：
- 检查服务是否在线
- 获取实际的服务类型
- 通过 `ROSServiceCaller` 执行真实的服务调用


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


对于 ROS 依赖，请按照 ROS 官方文档进行安装。

## 开发和扩展

脚本采用模块化设计，易于扩展：

- `ZJHumanoidROSTester` 类包含所有核心功能
- 支持自定义接口定义文件
- 可以轻松添加新的服务类型支持
- 支持插件式的数据处理器
