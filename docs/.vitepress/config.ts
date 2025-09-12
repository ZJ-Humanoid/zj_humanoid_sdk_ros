import { defineConfig } from 'vitepress';
import fs from 'fs';
import path from 'path';

const BASE = '/navi_sdk_documents/' // 根据你的 config.base 修改

function getAutoSidebar(dir) {
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
              .relative('./docs', fullPath)
              .replace(/\\/g, '/')
              .replace(/\.md$/, '') // 去掉 .md 扩展名
        }
      }
    }
  }).filter(Boolean)
}

const autoSidebar = getAutoSidebar('./docs/zj_humanoid/');
// console.log('自动生成的侧边栏:', JSON.stringify(autoSidebar, null, 1));
const repositorySlug = process.env.GITHUB_REPOSITORY || '';
const repositoryName = repositorySlug.includes('/') ? repositorySlug.split('/')[1] : repositorySlug;
const isGitHubActions = process.env.GITHUB_ACTIONS === 'true';

// If deploying to <user>.github.io, base must be '/'
const isUserOrOrgSite = repositoryName.endsWith('.github.io');

export default defineConfig({
  title: 'Navi机器人SDK开发指南',
  description: '这是浙江人形机器人Navi系列的SDK和编程指引文档站点。',
  base: '/navi_sdk_documents/',

  themeConfig: {
    sidebar: [
      {
        items: [
          { text: '快速开始', link: '/' },
          {
            text: 'ROS API',
            items: [          
              { text: '导图', link: '/markmap_ros_api' },
              { text: '文档', link: '/zj_humanoid_ros_api' },
              { text: '文件列表',
                collapsed: true,   // ✅ 默认折叠
                items: autoSidebar
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
        ]
      }
    ],

    outline: { level: [2, 3], label: 'On this page' },

  },

  vite: {
    optimizeDeps: {
      include: ['markmap-lib', 'markmap-view']
    }
  }
});


