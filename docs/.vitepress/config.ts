import { defineVersionedConfig } from '@viteplus/versions';
import fs from 'fs';
import path from 'path';

const BASE = '/navi_sdk_documents/' // 根据你的 config.base 修改

function getAutoSidebar(dir) {
  // 检查目录是否存在
  if (!fs.existsSync(dir)) {
    return []
  }

  try {
    const entries = fs.readdirSync(dir, { withFileTypes: true })

    return entries.map(entry => {
      const fullPath = path.join(dir, entry.name)

      if (entry.isDirectory()) {
        const children = getAutoSidebar(fullPath)
        return {
          text: `📂 ${entry.name}`,
          collapsed: true,
          items: children
        }
      } else {
        const ext = path.extname(entry.name)
        if (['.md', '.yaml', '.yml'].includes(ext)) {
          return {
            text: `📄 ${entry.name.replace(ext, '')}`,
            // ⚠️ 这里去掉 BASE，让 VitePress 自动加
            link:
              '/' +
              path
                .relative('./docs/src', fullPath)
                .replace(/\\/g, '/')
                .replace(/\.md$/, '') // 去掉 .md 扩展名
          }
        }
      }
    }).filter(Boolean)
  } catch (error) {
    console.warn(`Warning: Could not read directory ${dir}:`, error.message)
    return []
  }
}

// 尝试读取 zj_humanoid 目录（如果存在），否则返回空数组
const autoSidebar = getAutoSidebar('./docs/src/zj_humanoid/');
// console.log('自动生成的侧边栏:', JSON.stringify(autoSidebar, null, 1));
const repositorySlug = process.env.GITHUB_REPOSITORY || '';
const repositoryName = repositorySlug.includes('/') ? repositorySlug.split('/')[1] : repositorySlug;
const isGitHubActions = process.env.GITHUB_ACTIONS === 'true';

// If deploying to <user>.github.io, base must be '/'
const isUserOrOrgSite = repositoryName.endsWith('.github.io');

export default defineVersionedConfig({
  title: 'Navi机器人SDK开发指南',
  description: '这是浙江人形机器人Navi系列的SDK和编程指引文档站点。',
  base: '/zj_humanoid_sdk_ros/',
  outDir: '../dist',          // 构建输出到项目根目录的 dist 文件夹
  ignoreDeadLinks: true,      // 忽略死链接检查，允许构建继续

  // 版本管理配置
  versionsConfig: {
    current: 'main',          // 当前版本的标签（main 分支）
    sources: 'src',           // 当前版本文档目录（相对于 docs/）
    archive: 'versions',      // 旧版本归档目录（相对于 docs/）
    versionSwitcher: {
      text: '版本',           // 版本切换器显示文本
      includeCurrentVersion: true  // 在版本列表中包含当前版本
    }
  },

  themeConfig: {
    nav: [
      { text: '首页', link: '/' },
      { component: 'VersionSwitcher' }  // 版本切换器组件
    ],
    sidebar: [
      {
        items: [
          { text: '开发指南', link: '/',
            items: [          
              { text: '概述', link: '/#概述' },
              { text: '快速开始', link: '/#快速开始' },]
           },
          {
            text: 'ROS API',
            items: [          
              { text: '完整API文档', link: '/api/zj_humanoid_ros_api' },
              { 
                text: '子系统',
                collapsed: true,
                items: [
                  { text: '🔊 Audio', link: '/api/audio' },
                  { text: '🖐️ Hand', link: '/api/hand' },
                  { text: '🦵 Lowerlimb', link: '/api/lowerlimb' },
                  { text: '🔧 Manipulation', link: '/api/manipulation' },
                  { text: '🧭 Navigation', link: '/api/navigation' },
                  { text: '🤖 Robot', link: '/api/robot' },
                  { text: '📷 Sensor', link: '/api/sensor' },
                  { text: '🦾 Upperlimb', link: '/api/upperlimb' },
                ]
              }

            ]
          },
          {
            text: 'Message Type',
            items: [
              { text: '导图', link: '/markmap_message_type' },
              { text: '文档', link: '/zj_humanoid_types' },
            ]
          },
          {
            text: '开发示例',
            items: [          
              { 
                text: '子系统示例',
                collapsed: true,
                items: [
                  { text: '🔊 Audio', link: '/demos/audio_interfaces' },
                  { text: '🖐️ Hand', link: '/demos/dexhand_interface' },
                  { text: '🦵 Lowerlimb', link: '/demos/lowerlimb' },
                  { text: '🔧 Manipulation', link: '/demos/manipulation' },
                  { text: '🧭 Navigation', link: '/demos/navigation' },
                  { text: '🤖 Robot', link: '/demos/robot_interfaces' },
                  { text: '📷 Sensor', link: '/demos/sensor' },
                  { text: '🦾 Upperlimb', link: '/demos/uplimb_interface' },
                ]
              },
              { text: '综合示例', link: '/demos/Combined_Example.md' },
            ]
          },
          {
            text: '调试开发工具',
            items: [
              { text: 'WEB 遥控器', link: 'tools/web_telec' },
              { text: 'WEB 示教器', link: 'tools/web_tech' },
              { text: '大屏展示软件', link: 'tools/data_display' },
              { text: 'HOS 安装', link: 'tools/hos_install' },
              { text: 'HOS 开发', link: 'tools/hos_dev' },
            ]
          },
        ]
      }
    ],

    outline: { 
      level: [2, 4],  // 包含h2到h4，显示更详细的导航
      label: '本页导航'  // 自定义标签
    },

  },

  vite: {
    optimizeDeps: {
      include: ['markmap-lib', 'markmap-view']
    },
    assetsInclude: ['**/*.py'],
    server: {
      host: '0.0.0.0', // 监听所有网络接口，允许局域网访问
      port: 5173,      // 端口号
      fs: {
        allow: ['..']
      }
    }
  }
});


