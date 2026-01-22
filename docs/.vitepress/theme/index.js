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
          
          // 如果当前在版本化路径中（/versions/xxx/ 或 /develop/）
          const isVersionedPath = currentPath.includes('/versions/') || 
                                  (currentPath.includes('/develop/') && !currentPath.includes('/versions/develop/'))
          
          if (isVersionedPath) {
            const linkText = link.textContent?.trim()
            
            // 修复 main 链接：在版本化页面中，统一使用固定 URL
            if (linkText === 'main') {
              const correctMainUrl = '/zj_humanoid_sdk_ros/'
              // 检测所有错误的路径模式
              if (href && (
                href !== correctMainUrl &&
                !href.endsWith(correctMainUrl) &&
                // 检测错误的版本路径：/zj_humanoid_sdk_ros/1.1.0/ 或 /zj_humanoid_sdk_ros/develop/
                (href.match(/^\/zj_humanoid_sdk_ros\/\d+\.\d+\.\d+\/$/) ||
                 href.match(/^\/zj_humanoid_sdk_ros\/develop\/$/) ||
                 href.match(/^\/zj_humanoid_sdk_ros\/\d+\.\d+\.\d+$/) ||
                 href.match(/^\/zj_humanoid_sdk_ros\/develop$/) ||
                 // 检测相对路径在版本化上下文中的错误解析
                 (href === '/' && currentPath.includes('/versions/')) ||
                 // 检测其他包含版本号的错误路径
                 href.match(/\/\d+\.\d+\.\d+\//) ||
                 href.includes('/develop/')
                )
              )) {
                e.preventDefault()
                e.stopPropagation()
                window.location.href = correctMainUrl
                return false
              }
            }
            
            // 修复版本链接：检测所有错误的版本路径模式
            if (linkText && (linkText.startsWith('v') || linkText === 'develop')) {
              // 确定目标版本
              let targetVersion = linkText
              if (linkText.startsWith('v')) {
                targetVersion = linkText.substring(1) // 去掉 'v' 前缀
              }
              
              // 检测错误的路径模式：包含重复路径或错误的嵌套
              if (href && (
                href.match(/\/versions\/[^/]+\/versions\//) ||
                href.match(/\/\d+\.\d+\.\d+\/versions\//) ||
                href.match(/\/develop\/versions\//) ||
                href.match(/\/versions\/[^/]+\/zj_humanoid_sdk_ros\//) ||
                href.match(/\/\d+\.\d+\.\d+\/zj_humanoid_sdk_ros\//) ||
                href.match(/\/develop\/zj_humanoid_sdk_ros\//) ||
                href.includes(`/${targetVersion}/zj_humanoid_sdk_ros/`) ||
                (href.includes('/versions/') && isVersionedPath && 
                 !href.match(/^\/zj_humanoid_sdk_ros\/versions\/[^/]+\/$/))
              )) {
                e.preventDefault()
                e.stopPropagation()
                window.location.href = `/zj_humanoid_sdk_ros/versions/${targetVersion}/`
                return false
              }
              
              // 检测错误的路径模式：/1.1.0/ 或 /develop/ 而不是 /versions/1.1.0/ 或 /versions/develop/
              if (href && (
                (href.match(/^\/zj_humanoid_sdk_ros\/\d+\.\d+\.\d+\//) && !href.includes('/versions/')) ||
                (href.match(/^\/zj_humanoid_sdk_ros\/develop\//) && !href.includes('/versions/develop/'))
              )) {
                e.preventDefault()
                e.stopPropagation()
                window.location.href = `/zj_humanoid_sdk_ros/versions/${targetVersion}/`
                return false
              }
            }
          }
        }
      }
      
      // 使用捕获阶段监听所有链接点击
      document.addEventListener('click', handleClick, true)
      
      // 修复版本化页面中的图片路径
      const fixImagePaths = () => {
        const currentPath = window.location.pathname
        const isVersionedPath = currentPath.includes('/versions/') || 
                                (currentPath.includes('/develop/') && !currentPath.includes('/versions/develop/'))
        
        if (isVersionedPath) {
          // 提取版本号
          const versionMatch = currentPath.match(/\/versions\/([^/]+)/)
          if (versionMatch) {
            const version = versionMatch[1]
            const baseUrl = '/zj_humanoid_sdk_ros'
            
            // 修复所有图片路径
            const images = document.querySelectorAll('img')
            images.forEach(img => {
              const src = img.getAttribute('src')
              if (src) {
                // 如果图片路径指向根 assets 目录（在版本化页面中需要调整）
                // 例如：/zj_humanoid_sdk_ros/assets/xxx.png
                // 应该改为：/zj_humanoid_sdk_ros/versions/1.1.0/assets/xxx.png
                if (src.startsWith('/zj_humanoid_sdk_ros/assets/') && !src.includes('/versions/')) {
                  const assetPath = src.replace('/zj_humanoid_sdk_ros/assets/', 'assets/')
                  img.setAttribute('src', `${baseUrl}/versions/${version}/${assetPath}`)
                }
                // 如果图片路径是相对路径（以 ./images/ 或 images/ 开头）
                else if (src.startsWith('./images/') || src.startsWith('images/')) {
                  const imagePath = src.replace(/^\.\//, '').replace(/^images\//, 'images/')
                  img.setAttribute('src', `${baseUrl}/versions/${version}/${imagePath}`)
                }
                // 如果图片路径是相对路径（以 ../images/ 开头）
                else if (src.startsWith('../images/')) {
                  const imagePath = src.replace(/^\.\.\//, '')
                  img.setAttribute('src', `${baseUrl}/versions/${version}/${imagePath}`)
                }
                // 如果图片路径是绝对路径但缺少版本前缀
                else if (src.startsWith('/zj_humanoid_sdk_ros/images/') && !src.includes('/versions/')) {
                  const imagePath = src.replace('/zj_humanoid_sdk_ros/images/', 'images/')
                  img.setAttribute('src', `${baseUrl}/versions/${version}/${imagePath}`)
                }
              }
            })
          }
        }
      }
      
      // 页面加载后修复图片路径
      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', fixImagePaths)
      } else {
        fixImagePaths()
      }
      
      // 监听路由变化，修复动态加载的图片
      if (window.router) {
        window.router.onAfterRouteChanged = () => {
          setTimeout(fixImagePaths, 100)
        }
      }
    }
  }
}
