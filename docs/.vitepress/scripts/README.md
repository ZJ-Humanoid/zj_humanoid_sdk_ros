# 版本管理脚本说明

## 脚本列表

### 1. `sync-versions-from-git.js`

从 Git 标签同步版本文档到 `docs/versions/` 目录。

**用法**：
```bash
# 同步所有 Git 标签对应的版本
node docs/.vitepress/scripts/sync-versions-from-git.js
# 或使用 npm 脚本
npm run docs:sync-versions

# 从特定标签创建版本
node docs/.vitepress/scripts/sync-versions-from-git.js --tag v1.0.0
npm run docs:sync-version:tag v1.0.0

# 从特定分支创建版本（用于开发分支）
node docs/.vitepress/scripts/sync-versions-from-git.js --branch feature/new-api
npm run docs:sync-version:branch feature/new-api
```

**功能**：
- 读取所有 Git 标签（格式：`v*.*.*`）
- 为每个标签创建对应的版本目录
- 从标签对应的提交中提取 `docs/src/` 目录的内容
- 保存到 `docs/versions/{version}/` 目录

### 2. `get-versions-from-git.js`

获取所有可用版本的列表（JSON 格式）。

**用法**：
```bash
node docs/.vitepress/scripts/get-versions-from-git.js
# 或使用 npm 脚本
npm run docs:get-versions
```

**输出示例**：
```json
[
  "v2.0.0",
  "v1.0.0"
]
```

**功能**：
- 从 Git 标签获取版本列表
- 从本地 `docs/versions/` 目录获取版本列表
- 合并并去重
- 按版本号降序排列

## 工作流程

### 自动工作流（GitHub Actions）

1. **推送到 main 分支**：
   - 自动同步所有 Git 标签对应的版本
   - 构建并部署文档

2. **创建 Git 标签**：
   - 检测到标签推送（`v*` 格式）
   - 自动归档当前 `docs/src/` 到 `docs/versions/{version}/`
   - 从标签对应的提交中提取文档
   - 构建并部署文档

### 手动工作流

1. **更新文档**：
   ```bash
   # 在 docs/src/ 中更新文档
   git add docs/src/
   git commit -m "docs: update for v2.0.0"
   git push origin main
   ```

2. **创建版本标签**：
   ```bash
   git tag v2.0.0
   git push origin v2.0.0
   ```

3. **手动同步（如果需要）**：
   ```bash
   npm run docs:sync-versions
   ```

## 版本命名规则

- **Git 标签**：推荐使用语义化版本，如 `v1.0.0`, `v2.1.0`
- **版本目录**：自动去除 `v` 前缀，如 `v1.0.0` → `1.0.0`
- **版本显示**：在版本切换器中显示为 `v1.0.0`, `v2.1.0` 等

## 注意事项

- ✅ 脚本会自动处理版本号格式转换
- ✅ 支持语义化版本排序
- ✅ 自动去重（Git 标签和本地版本）
- ⚠️ 确保 Git 仓库有完整的标签历史
- ⚠️ 版本目录中的文档会从标签对应的提交中提取，确保标签时文档已提交
