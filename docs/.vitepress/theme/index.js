import DefaultTheme from 'vitepress/theme'
import Markmap from '../components/Markmap.vue'
import MarkdownInclude from '../components/MarkdownInclude.vue'
import './custom.css'

export default {
  extends: DefaultTheme,
  enhanceApp({ app }) {
    app.component('Markmap', Markmap)
    app.component('MarkdownInclude', MarkdownInclude)
  },
  setup() {
    // 在客户端修复版本切换链接
    if (typeof window !== 'undefined') {
      const handleClick = (e) => {
        const link = e.target.closest('a')
        if (link) {
          const href = link.getAttribute('href')
          const currentPath = window.location.pathname
          
          // 如果当前在版本化路径中（/versions/xxx/）
          if (currentPath.includes('/versions/')) {
            const linkText = link.textContent?.trim()
            
            // 修复 main 链接：检测所有可能的错误路径模式
            if (linkText === 'main') {
              // 检测错误的路径模式
              if (href && (
                href.includes('/zj_humanoid_sdk_ros/zj_humanoid_sdk_ros/') ||
                href.match(/\/\d+\.\d+\.\d+\/zj_humanoid_sdk_ros\//) ||
                href.match(/\/versions\/\d+\.\d+\.\d+\/zj_humanoid_sdk_ros\//)
              )) {
                e.preventDefault()
                e.stopPropagation()
                window.location.href = '/zj_humanoid_sdk_ros/'
                return false
              }
              // 如果链接是相对路径 ../../，确保它正确解析
              if (href && (href === '../../' || href.startsWith('../../'))) {
                // 相对路径应该能正确工作，但为了保险，我们也处理它
                const resolvedPath = new URL(href, window.location.href).pathname
                if (!resolvedPath.endsWith('/zj_humanoid_sdk_ros/') && !resolvedPath.endsWith('/zj_humanoid_sdk_ros')) {
                  e.preventDefault()
                  e.stopPropagation()
                  window.location.href = '/zj_humanoid_sdk_ros/'
                  return false
                }
              }
            }
            
            // 修复版本链接：如果链接指向错误的相对路径（/1.1.0/ 而不是 /versions/1.1.0/）
            if (href && href.match(/^\/zj_humanoid_sdk_ros\/\d+\.\d+\.\d+\//) && !href.includes('/versions/')) {
              if (linkText && linkText.startsWith('v') && linkText.match(/^v\d+\.\d+\.\d+$/)) {
                // 这是版本链接，应该重定向到 /versions/xxx/
                const version = linkText.substring(1) // 去掉 'v' 前缀
                e.preventDefault()
                e.stopPropagation()
                window.location.href = `/zj_humanoid_sdk_ros/versions/${version}/`
                return false
              }
            }
          }
        }
      }
      
      // 使用捕获阶段监听所有链接点击
      document.addEventListener('click', handleClick, true)
    }
  }
}
