# Docker 使用指南

本目录包含用于在 Docker 容器中运行 `zj_humanoid_mock` 的配置文件。

## 前置要求

- Docker (版本 20.10+)
- Docker Compose (可选，但推荐)

## 快速测试

运行测试脚本验证 Docker 设置：

```bash
cd mock_packages
./test_docker.sh
```

这将：
1. 检查 Docker 和 docker compose 是否安装
2. 构建 Docker 镜像
3. 测试容器启动（5秒）

## 构建说明

**重要**: Docker 构建上下文已设置为项目根目录，以便访问 `api_struct/zj_humanoid_types_25_R3.run` 文件。

构建时会自动：
1. 安装 `zj_humanoid_types_25_R3.run` 数据类型包（包含所有自定义消息类型）
2. 构建 mock ROS 包
3. 配置运行环境

## 快速开始

### 方法 1: 使用 Docker Compose (推荐)

```bash
# 构建并启动容器
cd mock_packages
docker compose up --build

# 后台运行
docker compose up -d --build

# 查看日志
docker compose logs -f

# 停止容器
docker compose down
```

### 方法 2: 使用 Docker 命令

```bash
# 构建镜像
cd mock_packages
docker build -t zj_humanoid_mock:latest .

# 运行容器（使用 host 网络模式以便与外部 ROS 节点通信）
docker run -it --rm \
  --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  zj_humanoid_mock:latest

# 后台运行
docker run -d --rm \
  --name zj_humanoid_mock \
  --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  zj_humanoid_mock:latest
```

## 自定义运行参数

### 限制模块

只运行特定子系统的 mock 服务：

```bash
docker run -it --rm \
  --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  zj_humanoid_mock:latest \
  /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    source /catkin_ws/devel/setup.bash && \
    rosrun zj_humanoid_mock mock_server.py --modules upperlimb hand"
```

### 调整发布频率

```bash
docker run -it --rm \
  --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  zj_humanoid_mock:latest \
  /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    source /catkin_ws/devel/setup.bash && \
    rosrun zj_humanoid_mock mock_server.py --publish-rate-scale 0.5"
```

## 与外部 ROS 节点通信

### 场景 1: 容器内运行 roscore 和 mock server

容器默认会启动 roscore 和 mock server。外部节点需要连接到容器的 ROS master：

```bash
# 在外部机器上
export ROS_MASTER_URI=http://<container-ip>:11311
export ROS_IP=<your-ip>
rosrun your_package your_node
```

### 场景 2: 使用外部 roscore

如果已有外部 roscore 在运行：

```bash
# 修改 docker-compose.yml 或运行命令
docker run -it --rm \
  --network host \
  -e ROS_MASTER_URI=http://<roscore-host>:11311 \
  zj_humanoid_mock:latest \
  /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    source /catkin_ws/devel/setup.bash && \
    rosrun zj_humanoid_mock mock_server.py"
```

## 进入容器调试

```bash
# 使用 docker compose
docker compose exec zj_humanoid_mock /bin/bash

# 使用 docker
docker exec -it zj_humanoid_mock /bin/bash
```

在容器内可以：
- 查看 ROS 话题: `rostopic list`
- 查看 ROS 服务: `rosservice list`
- 测试服务调用: `rosservice call /zj_humanoid/robot/basic_info`
- 查看话题数据: `rostopic echo /zj_humanoid/robot/robot_state`

## 依赖说明

### 标准 ROS 消息包

以下包已包含在基础镜像中：
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`

### 自定义消息包

Mock 包依赖以下自定义消息包：
- `audio`
- `hand`
- `manipulation`
- `navigation`
- `upperlimb`
- `zj_robot`
- `sensor`

**✅ 自动安装**: Dockerfile 已配置自动安装 `zj_humanoid_types_25_R3.run` 数据类型包，该包包含了所有必需的自定义消息类型定义。构建镜像时会自动执行安装。

**注意**: 如果构建时安装失败，mock server 在尝试加载这些消息类型时会显示警告，但不会影响标准消息类型的服务/话题。

## 故障排除

### 问题: 无法连接到 ROS master

**解决方案**: 确保使用 `--network host` 模式，或正确设置 `ROS_MASTER_URI`。

### 问题: 消息类型导入失败

**解决方案**: 确保所需的自定义消息包已安装。检查容器日志：
```bash
docker compose logs zj_humanoid_mock
```

### 问题: 端口冲突

**解决方案**: 如果 11311 端口被占用，可以：
1. 停止其他 roscore 实例
2. 使用外部 roscore（见场景 2）

## 开发模式

如果需要修改 mock 包代码后重新构建：

```bash
# 重新生成 mock 包（在项目根目录）
cd api_struct/scripts
python3 generate_mock_package.py

# 重新构建 Docker 镜像
cd ../../mock_packages
docker compose build --no-cache
```

## 示例：完整测试流程

```bash
# 1. 启动 mock server
cd mock_packages
docker compose up -d

# 2. 等待几秒让服务启动
sleep 5

# 3. 在另一个终端测试服务
export ROS_MASTER_URI=http://localhost:11311
rosservice call /zj_humanoid/robot/basic_info

# 4. 查看话题
rostopic echo /zj_humanoid/robot/robot_state

# 5. 停止容器
docker compose down
```

