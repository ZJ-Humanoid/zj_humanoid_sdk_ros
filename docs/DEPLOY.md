# 部署到 GitHub Pages

本文档说明如何将 VitePress 站点部署到 GitHub Pages。

## 前置条件

1. 确保你的 GitHub 仓库已创建
2. 确保代码已推送到 GitHub

## 配置说明

### 1. 检查 base 配置

当前配置的 `base` 是 `/zj_humanoid_sdk_ros/`，与仓库名匹配。

站点 URL 将是：`https://你的用户名.github.io/zj_humanoid_sdk_ros/`

如果将来需要修改仓库名，记得同步更新 `docs/.vitepress/config.ts` 中的 `base` 配置：

```typescript
base: '/你的仓库名/',
```

### 2. 启用 GitHub Pages

1. 进入 GitHub 仓库设置页面
2. 点击左侧菜单的 **Pages**
3. 在 **Source** 部分，选择：
   - **Source**: `GitHub Actions`
4. 保存设置

## 部署步骤

### 方法一：自动部署（推荐）

1. **推送代码到 main 分支**
   ```bash
   git add .
   git commit -m "准备部署到 GitHub Pages"
   git push origin main
   ```

2. **查看部署状态**
   - 进入 GitHub 仓库页面
   - 点击 **Actions** 标签页
   - 查看工作流运行状态
   - 等待部署完成（通常需要 2-5 分钟）

3. **访问站点**
   - 部署完成后，在仓库设置 > Pages 页面可以看到站点 URL
   - 或者访问：`https://你的用户名.github.io/zj_humanoid_sdk_ros/`

### 方法二：手动部署

如果需要手动触发部署：

1. 进入 GitHub 仓库页面
2. 点击 **Actions** 标签页
3. 选择 **Deploy VitePress to GitHub Pages** 工作流
4. 点击 **Run workflow** 按钮
5. 选择分支（通常是 `main`）
6. 点击 **Run workflow**

## 本地测试构建

在部署前，建议先在本地测试构建：

```bash
# 生成 API 文档
cd api_struct/scripts
python3 generate_vitepress_docs.py

# 构建站点
cd ../..
npm run docs:build

# 预览构建结果
npm run docs:preview
```

## 常见问题

### 1. 部署失败

- 检查 GitHub Actions 日志，查看错误信息
- 确保所有依赖都已正确安装
- 确保 Python 脚本可以正常运行

### 2. 页面 404

- 检查 `base` 配置是否与仓库名匹配
- 确保 GitHub Pages 的 Source 设置为 `GitHub Actions`

### 3. 资源加载失败

- 检查所有资源路径是否使用了相对路径或正确的 base 路径
- 确保 `public` 目录中的文件路径正确

## 更新站点

每次推送到 `main` 分支时，GitHub Actions 会自动重新构建和部署站点。无需手动操作。

## 自定义域名（可选）

如果需要使用自定义域名：

1. 在仓库根目录创建 `CNAME` 文件，内容为你的域名：
   ```
   docs.example.com
   ```

2. 在域名 DNS 设置中添加 CNAME 记录，指向 `你的用户名.github.io`

3. 在 GitHub Pages 设置中启用自定义域名

