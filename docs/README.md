# VitePress API 文档使用指南

## 📋 项目结构

```
docs/
├── index.md                    # 首页
├── develop_guides.md          # 开发指南
├── api/                       # API文档（自动生成）
│   ├── zj_humanoid_ros_api.md      # 完整API文档
│   └── subsystems/            # 各子系统文档
│       ├── audio.md
│       ├── hand.md
│       ├── lowerlimb.md
│       ├── manipulation.md
│       ├── navigation.md
│       ├── robot.md
│       ├── sensor.md
│       └── upperlimb.md
└── .vitepress/                # VitePress配置
    ├── config.ts              # 配置文件
    └── theme/
        └── custom.css        # 自定义样式
```

## 🚀 快速开始

### 1. 安装依赖

```bash
# 在项目根目录执行
npm install
```

### 2. 生成API文档

在更新 `api_struct/zj_humanoid/` 下的接口后，需要重新生成文档：

```bash
# 生成YAML汇总文件
cd api_struct/scripts
python3 generate_whole_yaml.py

# 生成VitePress文档
python3 generate_vitepress_docs.py
```

### 3. 启动开发服务器

```bash
# 在项目根目录执行
npm run docs:dev
```

访问 http://localhost:5173/zj_humanoid_sdk_ros/ 查看文档

### 4. 构建生产版本

```bash
npm run docs:build
```

构建产物在 `.vitepress/dist/` 目录

### 5. 预览生产版本

```bash
npm run docs:preview
```

## 📖 文档页面说明

### 首页 (`/`)
- 项目介绍
- 快速导航链接
- 功能特性展示

### 完整API文档 (`/api/zj_humanoid_ros_api`)
- 所有Services和Topics的完整列表
- 按子系统分组显示
- 包含统计信息

### 子系统文档 (`/api/subsystems/{subsystem}`)
- 每个子系统的独立页面
- 包含该子系统的所有Services和Topics
- 便于快速查找特定子系统的接口

## 🔄 更新文档流程

1. **修改接口定义**
   - 编辑 `api_struct/zj_humanoid/` 下的 `service.yaml` 或 `topic.yaml`

2. **重新生成文档**
   ```bash
   cd api_struct/scripts
   python3 generate_whole_yaml.py      # 生成YAML汇总
   python3 generate_vitepress_docs.py  # 生成VitePress文档
   ```

3. **查看更新**
   - 开发服务器会自动热重载
   - 刷新浏览器即可看到更新

## 🎨 自定义样式

样式文件位于 `docs/.vitepress/theme/custom.css`，可以自定义：
- 表格样式
- 标题样式
- 代码块样式
- 统计卡片样式

## 📝 侧边栏配置

侧边栏配置在 `docs/.vitepress/config.ts` 中，可以：
- 添加新的导航项
- 调整菜单顺序
- 修改菜单图标和文字

## 🌐 部署

### GitHub Pages

1. 构建文档：
   ```bash
   npm run docs:build
   ```

2. 将 `.vitepress/dist/` 目录内容推送到 `gh-pages` 分支

3. 在GitHub仓库设置中启用Pages，选择 `gh-pages` 分支

### 其他平台

构建后的静态文件可以部署到任何静态网站托管服务：
- Netlify
- Vercel
- Cloudflare Pages
- 等

## 🐛 常见问题

### Q: 文档没有更新？
A: 确保已运行 `generate_vitepress_docs.py` 重新生成文档

### Q: 样式没有生效？
A: 检查 `custom.css` 文件是否正确加载，可能需要重启开发服务器

### Q: 侧边栏链接404？
A: 检查 `config.ts` 中的链接路径是否正确，确保文件存在

## 📚 相关文件

- **生成脚本**: `api_struct/scripts/generate_vitepress_docs.py`
- **配置文件**: `docs/.vitepress/config.ts`
- **样式文件**: `docs/.vitepress/theme/custom.css`
- **数据源**: `api_struct/generated/zj_humanoid_interfaces.yaml`

