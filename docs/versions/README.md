# 版本管理说明

## 目录结构

```
docs/
├── src/              # 当前/最新版本的文档
│   ├── index.md
│   ├── api/
│   ├── demos/
│   └── ...
└── versions/         # 归档的旧版本文档
    ├── 1.0/         # v1.0 版本的文档（从 Git 标签自动同步）
    │   ├── index.md
    │   ├── api/
    │   └── ...
    └── 2.0/         # v2.0 版本的文档
        └── ...
```

## 与 Git 标签/分支关联

版本管理已与 Git 标签和分支集成，支持自动同步：

### 自动同步（推荐）

**当创建 Git 标签时**：
```bash
# 1. 创建并推送标签
git tag v1.0.0
git push origin v1.0.0

# 2. GitHub Actions 会自动：
#    - 检测到标签推送
#    - 归档当前 docs/src 到 docs/versions/1.0.0
#    - 从标签对应的提交中提取文档
#    - 部署到 GitHub Pages
```

**当推送到 main 分支时**：
- GitHub Actions 会自动同步所有 Git 标签对应的版本
- 确保所有已发布的版本文档都是最新的

### 手动同步

如果需要手动同步版本：

```bash
# 同步所有 Git 标签对应的版本
npm run docs:sync-versions

# 从特定标签创建版本
npm run docs:sync-version:tag v1.0.0

# 从特定分支创建版本（用于开发分支）
npm run docs:sync-version:branch feature/new-api

# 查看所有可用版本
npm run docs:get-versions
```

## 版本命名规则

- **Git 标签格式**：`v1.0.0`, `v2.1.0` 等（推荐使用语义化版本）
- **版本目录名**：自动去除 `v` 前缀，例如 `v1.0.0` → `1.0.0`
- **版本显示名**：在版本切换器中显示为 `v1.0.0`, `v2.1.0` 等

## 工作流程示例

### 发布新版本

1. **更新文档**：
   ```bash
   # 在 docs/src/ 中更新文档内容
   # 提交更改
   git add docs/src/
   git commit -m "docs: update for v2.0.0"
   git push origin main
   ```

2. **创建版本标签**：
   ```bash
   git tag v2.0.0
   git push origin v2.0.0
   ```

3. **自动归档**：
   - GitHub Actions 检测到标签推送
   - 自动将当前 `docs/src/` 归档到 `docs/versions/2.0.0`
   - 从标签对应的提交中提取文档内容
   - 部署更新后的文档站点

### 查看版本列表

```bash
# 获取所有可用版本（JSON 格式）
npm run docs:get-versions
```

## 注意事项

- ✅ **推荐使用 Git 标签**：版本与代码版本保持一致
- ✅ **语义化版本**：使用 `v1.0.0` 格式的标签
- ✅ **自动同步**：GitHub Actions 会自动处理版本归档
- ⚠️ **文档结构**：每个版本的文档应该保持相对路径链接
- ⚠️ **版本特定配置**：可以在 `config.ts` 中为每个版本配置不同的导航和侧边栏
