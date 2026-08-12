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

const rawConfig = defineVersionedConfig({
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
    // 关闭内置版本切换器，使用自定义导航
    versionSwitcher: false
  },

  themeConfig: {
    nav: {
      root: (() => {
        const items = [{ text: 'main', link: '/' }]
        
        // 扫描 docs/versions 下的版本目录，生成 /versions/<name>/ 链接
        try {
          const versionsRoot = path.resolve('./docs/versions')
          if (fs.existsSync(versionsRoot)) {
            const versions = fs
              .readdirSync(versionsRoot, { withFileTypes: true })
              .filter((e) => e.isDirectory())
              .map((e) => e.name)
              .filter((name) => !name.startsWith('.'))
              .sort((a, b) => a.localeCompare(b, undefined, { numeric: true, sensitivity: 'base' }))
            
            for (const v of versions) {
              // 为版本添加 "v" 前缀显示，但链接使用实际目录名
              const displayName = v.match(/^\d+\.\d+/) ? `v${v}` : v
              items.push({ text: displayName, link: `/versions/${v}/` })
            }
          }
        } catch (e) {
          // ignore
        }
        
        return [
          { text: '版本', items, skipVersioning: true }
        ]
      })(),
      // 为每个版本配置导航（确保 main 链接正确）
      ...(() => {
        const versionNavs = {}
        try {
          const versionsRoot = path.resolve('./docs/versions')
          if (fs.existsSync(versionsRoot)) {
            const versions = fs
              .readdirSync(versionsRoot, { withFileTypes: true })
              .filter((e) => e.isDirectory())
              .map((e) => e.name)
              .filter((name) => !name.startsWith('.'))
            
            for (const v of versions) {
              const items = [
                { text: 'main', link: '/' },  // 相对于 base 的路径
                { text: v.match(/^\d+\.\d+/) ? `v${v}` : v, link: `/versions/${v}/` }  // 相对于 base 的路径
              ]
              
              // 添加其他版本
              for (const otherV of versions) {
                if (otherV !== v) {
                  items.push({ 
                    text: otherV.match(/^\d+\.\d+/) ? `v${otherV}` : otherV, 
                    link: `/versions/${otherV}/`  // 相对于 base 的路径
                  })
                }
              }
              
              versionNavs[v] = [
                { text: '版本', items, skipVersioning: true }
              ]
            }
          }
        } catch (e) {
          // ignore
        }
        return versionNavs
      })()
    },
    sidebar: (() => {
      // 通用侧边栏（链接使用 /path 形式，保持 VitePress 原有语义，由库自己处理重写）
      const commonSidebar = [
        {
          items: [
            {
              text: '开发指南', link: '/',
              items: [
                { text: '概述', link: '/#概述' },
                { text: '快速开始', link: '/#快速开始' },
              ]
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
                    { text: '🚙 Chassis', link: '/api/chassis' },
                    { text: '🖐️ Hand', link: '/api/hand' },
                    { text: '🦵 Lowerlimb', link: '/api/lowerlimb' },
                    { text: '🔧 Manipulation', link: '/api/manipulation' },
                    { text: '🧭 Navigation', link: '/api/navigation' },
                    { text: '🗺️ Perception', link: '/api/perception' },
                    { text: '🤖 Robot', link: '/api/robot' },
                    { text: '📷 Sensor', link: '/api/sensor' },
                    { text: '🦾 Upperlimb', link: '/api/upperlimb' },
                  ]
                }
              ]
            },
            {
              text: '版本变更',
              link: '/release_notes'
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
                { text: 'WEB 遥控器', link: '/tools/web_telec' },
                { text: 'WEB 示教器', link: '/tools/web_tech' },
                { text: '大屏展示软件', link: '/tools/data_display' },
                { text: 'HOS 安装', link: '/tools/hos_install' },
                { text: 'HOS 开发', link: '/tools/hos_dev' },
              ]
            },
          ]
        }
      ]

      // VitePress 的 sidebar key 匹配是基于 page.relativePath（源文件路径）：
      //   - main 分支文档在 docs/src/xxx.md -> relativePath = "api/hand.md"
      //     -> 需要匹配 sidebar key "/" 或 "/api/"
      //   - 版本文档在 docs/src/versions/1.2.0/xxx.md -> relativePath = "versions/1.2.0/api/hand.md"
      //     -> 需要匹配 sidebar key "/versions/1.2.0/" 或 "/versions/"
      // 因此我们需要额外为 /versions/<v>/ 路径提供一份相同结构的 sidebar
      // （VitePress 版本库会通过 rewritesHook 处理链接的渲染时 base）
      const sidebarConfig: Record<string, any> = {
        '/': commonSidebar
      }

      try {
        const versionsRoot = path.resolve('./docs/versions')
        if (fs.existsSync(versionsRoot)) {
          const versions = fs
            .readdirSync(versionsRoot, { withFileTypes: true })
            .filter((e) => e.isDirectory())
            .map((e) => e.name)
            .filter((name) => !name.startsWith('.'))

          for (const v of versions) {
            sidebarConfig[`/versions/${v}/`] = commonSidebar
          }
        }
      } catch (e) {
        // ignore
      }

      return sidebarConfig
    })(),

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

export default rawConfig


