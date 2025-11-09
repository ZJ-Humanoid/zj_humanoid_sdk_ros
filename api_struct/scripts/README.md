# API Structure Scripts

本目录包含用于生成和测试 ZJ Humanoid ROS API 的脚本。

## 📁 目录结构

```
api_struct/
├── scripts/                          # 所有生成脚本（本目录）
│   ├── generate_whole_yaml.py       # 1️⃣ 从 zj_humanoid/ 生成聚合 YAML
│   ├── generate_json_from_yaml.py   # 2️⃣ 生成 JSON 和 Markdown 文档
│   ├── generate_ros_tests.py        # 3️⃣ 生成 ROS 测试脚本
│   ├── run_all_generators.py        # 🚀 一键运行所有生成器
│   ├── demos_ros_test.py            # Python API 测试工具
│   ├── demos_test_shell.sh          # Shell 测试脚本
│   ├── demos_test.ps1               # PowerShell 测试脚本（Windows）
│   └── README.md                    # 本文档
├── generated/                        # 生成的文件
│   ├── zj_humanoid_interfaces.yaml
│   ├── zj_humanoid_interfaces_*.json
│   └── zj_humanoid_interfaces.md
└── zj_humanoid/                      # API 定义（源文件）
    └── */topic.yaml, service.yaml
```

## 🚀 快速开始

### 一键生成所有文件

```bash
cd api_struct/scripts/
python3 run_all_generators.py
```

这将按顺序运行：
1. 生成聚合 YAML 文件
2. 生成 JSON 和 Markdown 文档
3. 生成 ROS 测试脚本

## 📋 各脚本说明

### 1. generate_whole_yaml.py

**功能**：遍历 `zj_humanoid/` 目录，读取所有 `service.yaml` 和 `topic.yaml`，生成聚合文件。

**输入**：
- `../zj_humanoid/**/*.yaml` (service.yaml, topic.yaml)

**输出**：
- `../generated/zj_humanoid_interfaces.yaml`

**用法**：
```bash
python3 generate_whole_yaml.py
```

---

### 2. generate_json_from_yaml.py

**功能**：从聚合 YAML 生成 JSON 和 Markdown 文档，按机器人型号分类。

**输入**：
- `../generated/zj_humanoid_interfaces.yaml`

**输出**：
- `../generated/zj_humanoid_interfaces_H1.json`
- `../generated/zj_humanoid_interfaces_I2.json`
- `../generated/zj_humanoid_interfaces_WA1.json`
- `../generated/zj_humanoid_interfaces_WA2.json`
- `../generated/zj_humanoid_interfaces.md`

**用法**：
```bash
python3 generate_json_from_yaml.py
```

---

### 3. generate_ros_tests.py

**功能**：为每个接口生成 `topic_test.py` 和 `service_test.py` 测试脚本。

**输入**：
- `../generated/zj_humanoid_interfaces.yaml`
- `../zj_humanoid/**/topic.yaml`
- `../zj_humanoid/**/service.yaml`

**输出**：
- `../zj_humanoid/**/topic_test.py` (约 80 个)
- `../zj_humanoid/**/service_test.py` (约 82 个)

**用法**：
```bash
python3 generate_ros_tests.py
```

---

## 🧪 测试脚本

### demos_ros_test.py (Python)

**功能**：通过 Python 调用 ROS 服务或发布话题。

**用法**：
```bash
# 调用服务
python3 demos_ros_test.py ../zj_humanoid/upperlimb/movej/left_arm/left_arm_t_case1.yaml

# 显式指定服务名
python3 demos_ros_test.py ../zj_humanoid/audio/listen/hello_world.yaml --service /zj_humanoid/audio/listen
```

---

### demos_test_shell.sh (Shell)

**功能**：通过 shell 脚本调用 ROS 命令。

**用法**：
```bash
# Linux/Mac
sh demos_test_shell.sh ../zj_humanoid/upperlimb/movej/left_arm/left_arm_t_case1.yaml
```

---

### demos_test.ps1 (PowerShell)

**功能**：Windows PowerShell 版本的测试脚本。

**用法**：
```powershell
# Windows PowerShell
.\demos_test.ps1 ..\zj_humanoid\upperlimb\movej\left_arm\left_arm_t_case1.yaml
```

---

## 🔄 工作流程

### 添加新的 API 接口

1. **创建目录结构**
   ```bash
   mkdir -p ../zj_humanoid/module_name/api_name
   ```

2. **创建定义文件**（创建 service.yaml 或 topic.yaml）

3. **添加示例数据**（创建 demo 数据文件）

4. **重新生成所有文件**
   ```bash
   cd scripts/
   python3 run_all_generators.py
   ```

5. **测试新的接口**
   ```bash
   cd ../zj_humanoid/module_name/api_name
   python3 service_test.py case1.yaml
   ```

## 📝 注意事项

1. **路径问题** - 所有脚本都应该从 `api_struct/scripts/` 目录运行
2. **生成顺序** - 使用 `run_all_generators.py` 可以确保正确的执行顺序
3. **Windows 用户** - 推荐使用 PowerShell 脚本 (`demos_test.ps1`)

## 🛠️ 依赖要求

### Python 依赖
```bash
pip install PyYAML
```

### ROS 依赖
- rospy, rosservice, std_srvs
- 各模块的消息类型包 (upperlimb, audio, etc.)

## 📚 相关文档

- [主 README](../README.md) - 项目总体说明
- [测试脚本使用指南](./README_test_scripts.md) - 详细测试说明
- [生成的 API 文档](../generated/zj_humanoid_interfaces.md) - API 接口文档
