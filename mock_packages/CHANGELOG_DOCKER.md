# Docker 配置更新日志

## 数据类型包自动安装

### 更新内容

1. **Dockerfile 更新**
   - 添加了 `zj_humanoid_types_25_R3.run` 的自动安装步骤
   - 安装过程在构建镜像时自动执行
   - 包含安装验证和清理步骤

2. **构建上下文调整**
   - `docker-compose.yml` 的构建上下文改为项目根目录 (`..`)
   - 允许 Dockerfile 访问 `api_struct/` 目录中的 `.run` 文件
   - 所有 COPY 路径已相应更新

3. **Docker Compose 命令更新**
   - 所有脚本和文档已更新为使用 `docker compose` (V2) 而不是 `docker-compose` (V1)
   - 测试脚本已更新检测逻辑以支持 Docker Compose V2

3. **安装步骤说明**
   ```dockerfile
   # 复制 .run 文件到容器
   COPY api_struct/zj_humanoid_types_25_R3.run /tmp/
   
   # 非交互式安装
   # 安装程序会解压 .deb 文件并通过 dpkg 安装
   # 自动处理所有提示
   ```

### 使用方法

#### 使用 docker compose（推荐）

```bash
cd mock_packages
docker compose build
docker compose up
```

#### 使用 docker 命令

```bash
# 从项目根目录构建
cd /path/to/zj_humanoid_sdk_ros
docker build -f mock_packages/Dockerfile -t zj_humanoid_mock:latest .
```

### 验证安装

构建完成后，可以在容器中验证数据类型包是否已安装：

```bash
# 进入容器
docker compose exec zj_humanoid_mock bash

# 验证包是否可用
source /opt/ros/noetic/setup.bash
rospack find audio
rospack find hand
rospack find upperlimb
# ... 其他包
```

### 注意事项

1. **构建上下文**: 必须从项目根目录构建，或使用 `docker compose`（已配置正确上下文）

2. **安装时间**: 数据类型包的安装会增加构建时间，但只需构建一次

3. **网络要求**: 安装过程不需要网络连接，所有依赖都在 `.run` 文件中

4. **错误处理**: 如果安装失败，构建仍会继续，但会在日志中显示警告

### 故障排除

如果遇到安装问题：

1. **检查 .run 文件是否存在**
   ```bash
   ls -lh api_struct/zj_humanoid_types_25_R3.run
   ```

2. **查看构建日志**
   ```bash
   docker compose build --progress=plain 2>&1 | grep -A 10 "zj_humanoid_types"
   ```

3. **手动测试安装**
   ```bash
   # 在容器中手动运行
   docker run -it --rm zj_humanoid_mock:latest bash
   # 然后手动执行安装步骤
   ```

### 相关文件

- `Dockerfile`: 包含安装步骤
- `docker-compose.yml`: 配置构建上下文
- `README_DOCKER.md`: 完整使用文档
- `test_docker.sh`: 测试脚本（已更新）

