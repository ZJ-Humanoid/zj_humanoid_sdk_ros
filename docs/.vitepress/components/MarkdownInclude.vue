<template>
  <div class="markdown-include" :class="{ 'max-width-limited': maxWidth }" :style="maxWidth ? { maxWidth: maxWidth } : {}">
    <div v-if="loading" class="loading">加载中...</div>
    <div v-else-if="error" class="error">{{ error }}</div>
    <div v-else v-html="renderedContent" class="markdown-body"></div>
  </div>
</template>

<script setup>
import { ref, onMounted, onBeforeMount } from 'vue'
import { useRoute } from 'vitepress'
import MarkdownIt from 'markdown-it'
import markdownItContainer from 'markdown-it-container'

// 调试：组件创建时立即输出
console.log('[MarkdownInclude] Component script loaded')

const route = useRoute()

const props = defineProps({
  src: {
    type: String,
    required: true
  },
  skipFrontmatter: {
    type: Boolean,
    default: true
  },
  skipTitle: {
    type: Boolean,
    default: false
  },
  section: {
    type: String,
    default: '',  // 'services', 'topics', 或 '' (全部)
    validator: (value) => ['', 'services', 'topics'].includes(value)
  },
  maxWidth: {
    type: String,
    default: ''  // 例如: '800px', '60ch', '90%' 等
  }
})

const renderedContent = ref('')
const loading = ref(true)
const error = ref(null)

// 初始化markdown-it，支持VitePress容器语法
const md = new MarkdownIt({
  html: true,
  linkify: true,
  typographer: true
})

// 添加容器插件支持（支持 ::: info, ::: tip, ::: warning, ::: danger 等）
const containerTypes = ['info', 'tip', 'warning', 'danger', 'details']
containerTypes.forEach(type => {
  md.use(markdownItContainer, type, {
    validate: (params) => {
      return params.trim().match(new RegExp(`^${type}(\\s.*)?$`))
    },
    render: (tokens, idx) => {
      const token = tokens[idx]
      
      if (token.nesting === 1) {
        // opening tag
        const title = token.info.trim().slice(type.length).trim()
        return `<div class="vp-custom-container ${type}">\n`
      } else {
        // closing tag
        return '</div>\n'
      }
    }
  })
})

// 文件内容映射（在构建时预加载）
const fileMap = {
  // 根目录文件
  'develop_guides.md': () => import('../../develop_guides.md?raw'),
  // API文档
  'api/zj_humanoid_ros_api.md': () => import('../../api/zj_humanoid_ros_api.md?raw'),
  // 子系统文档（完整路径）
  'api/subsystems/audio.md': () => import('../../api/subsystems/audio.md?raw'),
  'api/subsystems/hand.md': () => import('../../api/subsystems/hand.md?raw'),
  'api/subsystems/lowerlimb.md': () => import('../../api/subsystems/lowerlimb.md?raw'),
  'api/subsystems/manipulation.md': () => import('../../api/subsystems/manipulation.md?raw'),
  'api/subsystems/navigation.md': () => import('../../api/subsystems/navigation.md?raw'),
  'api/subsystems/robot.md': () => import('../../api/subsystems/robot.md?raw'),
  'api/subsystems/sensor.md': () => import('../../api/subsystems/sensor.md?raw'),
  'api/subsystems/upperlimb.md': () => import('../../api/subsystems/upperlimb.md?raw'),
  // 子系统文档（相对路径，从api目录引用）
  'subsystems/audio.md': () => import('../../api/subsystems/audio.md?raw'),
  'subsystems/hand.md': () => import('../../api/subsystems/hand.md?raw'),
  'subsystems/lowerlimb.md': () => import('../../api/subsystems/lowerlimb.md?raw'),
  'subsystems/manipulation.md': () => import('../../api/subsystems/manipulation.md?raw'),
  'subsystems/navigation.md': () => import('../../api/subsystems/navigation.md?raw'),
  'subsystems/robot.md': () => import('../../api/subsystems/robot.md?raw'),
  'subsystems/sensor.md': () => import('../../api/subsystems/sensor.md?raw'),
  'subsystems/upperlimb.md': () => import('../../api/subsystems/upperlimb.md?raw'),
}

onBeforeMount(() => {
  console.log(`[MarkdownInclude] Before mount with src="${props.src}", section="${props.section}"`)
})

onMounted(async () => {
  console.log(`[MarkdownInclude] Mounted with src="${props.src}", section="${props.section}"`)
  try {
    loading.value = true
    error.value = null
    
    let content = ''
    let normalizedSrc = props.src.startsWith('/') ? props.src.substring(1) : props.src
    
    // 根据当前路由智能处理路径
    const currentPath = route.path
    console.log(`[MarkdownInclude] Current route path: "${currentPath}"`)
    console.log(`[MarkdownInclude] Original src: "${props.src}"`)
    
    // 如果路径是相对路径，需要根据当前页面位置解析
    if (!normalizedSrc.startsWith('docs/') && !normalizedSrc.startsWith('api/')) {
      // 如果当前页面在api目录下，且路径是subsystems/开头，路径正确
      if (currentPath.includes('/api/') && normalizedSrc.startsWith('subsystems/')) {
        // 路径已经是正确的
      }
      // 如果当前页面在根目录，且路径是api/开头，路径正确
      else if (!currentPath.includes('/api/') && normalizedSrc.startsWith('api/')) {
        // 路径已经是正确的
      }
      // 如果当前页面在api目录下，且路径不是subsystems/开头，可能需要添加api/前缀
      else if (currentPath.includes('/api/') && !normalizedSrc.startsWith('subsystems/') && !normalizedSrc.startsWith('api/')) {
        // 可能是相对于api目录的路径，保持不变
      }
      // 如果当前页面在根目录，且路径不是api/开头，可能需要添加docs/前缀
      else if (!currentPath.includes('/api/') && !normalizedSrc.startsWith('api/') && !normalizedSrc.startsWith('docs/')) {
        // 路径相对于docs目录，保持不变
      }
    }
    
    console.log(`[MarkdownInclude] Normalized src: "${normalizedSrc}"`)
    
    // 方法1: 尝试使用预定义的导入
    if (fileMap[normalizedSrc]) {
      console.log(`[MarkdownInclude] Trying predefined import for "${normalizedSrc}"`)
      try {
        const module = await fileMap[normalizedSrc]()
        console.log(`[MarkdownInclude] Module loaded:`, module)
        content = module.default || module || ''
        console.log(`[MarkdownInclude] Predefined import succeeded, content length: ${content.length}`)
        if (content.length === 0) {
          console.warn(`[MarkdownInclude] Content is empty after import`)
        }
      } catch (e) {
        console.error(`[MarkdownInclude] Predefined import failed for "${normalizedSrc}":`, e)
        console.error(`[MarkdownInclude] Error stack:`, e.stack)
      }
    } else {
      console.log(`[MarkdownInclude] No predefined import found for "${normalizedSrc}"`)
      console.log(`[MarkdownInclude] Available keys in fileMap:`, Object.keys(fileMap))
    }
    
    // 方法2: 如果预定义导入失败，尝试动态导入
    if (!content) {
      try {
        // 构建相对于.vitepress/components/的路径
        let importSrc = normalizedSrc
        
        // 根据路径类型和当前路由处理路径
        if (importSrc.startsWith('subsystems/')) {
          // subsystems/ 路径需要添加 api/ 前缀
          importSrc = `api/${importSrc}`
        } else if (importSrc.startsWith('api/')) {
          // api/ 路径保持不变
        } else if (!importSrc.startsWith('docs/')) {
          // 其他路径，如果当前在api目录下，可能需要添加api/前缀
          // 否则保持原样（相对于docs目录）
        }
        
        const importPath = `../../${importSrc}?raw`
        console.log(`[MarkdownInclude] Trying dynamic import: "${importPath}"`)
        const module = await import(/* @vite-ignore */ importPath)
        content = module.default || ''
        console.log(`[MarkdownInclude] Dynamic import succeeded, content length: ${content.length}`)
      } catch (importError) {
        console.warn(`[MarkdownInclude] Dynamic import failed:`, importError)
      }
    }
    
    // 方法3: 如果导入都失败，尝试fetch（开发模式下）
    if (!content) {
      // 构建fetch路径，处理子系统文档路径
      let fetchSrc = normalizedSrc
      if (fetchSrc.startsWith('subsystems/')) {
        fetchSrc = `api/${fetchSrc}`
      }
      
      // 构建多个可能的fetch路径
      const fetchPaths = [
        // 带base路径的完整路径
        `/navi_sdk_documents/${fetchSrc}`,
        `/navi_sdk_documents/${fetchSrc}.md`,
        `/navi_sdk_documents/docs/${fetchSrc}`,
        `/navi_sdk_documents/docs/${fetchSrc}.md`,
        // 不带base路径的路径
        `/${fetchSrc}`,
        `/${fetchSrc}.md`,
        `/docs/${fetchSrc}`,
        `/docs/${fetchSrc}.md`,
        // 原始路径
        `/${normalizedSrc}`,
        `/${normalizedSrc}.md`,
        `/docs/${normalizedSrc}`,
        `/docs/${normalizedSrc}.md`
      ]
      
      for (const fetchPath of fetchPaths) {
        try {
          const response = await fetch(fetchPath)
          if (response.ok) {
            content = await response.text()
            console.log('Successfully loaded from:', fetchPath)
            break
          }
        } catch (e) {
          continue
        }
      }
    }
    
    if (!content) {
      throw new Error(`Failed to load ${props.src}. Please check the file path.`)
    }
    
    // 跳过frontmatter
    if (props.skipFrontmatter && content.startsWith('---')) {
      const frontmatterEnd = content.indexOf('---', 3)
      if (frontmatterEnd !== -1) {
        content = content.substring(frontmatterEnd + 3).trim()
      }
    }
    
    console.log(`[MarkdownInclude] Content before section extraction: ${content.length} chars`)
    
    // 移除 script 标签块，使用 Unicode 转义避免 Vite 插件解析问题
    const scriptStart = '\u003Cscript'
    const scriptEnd = '\u003C/' + 'script\u003E'
    while (content.includes(scriptStart)) {
      const startIdx = content.indexOf(scriptStart)
      const endIdx = content.indexOf(scriptEnd, startIdx)
      if (endIdx !== -1) {
        content = content.substring(0, startIdx) + content.substring(endIdx + scriptEnd.length)
      } else {
        break
      }
    }
    console.log(`[MarkdownInclude] Removed script tags, content length: ${content.length} chars`)
    
    // 根据section参数提取特定部分（先提取，再跳过标题）
    if (props.section) {
      console.log(`[MarkdownInclude] Extracting section "${props.section}"`)
      const lines = content.split('\n')
      let sectionStart = -1
      let sectionEnd = -1
      
      // 查找目标section的开始标记（支持多种格式）
      const sectionPatterns = {
        'services': [
          /^##\s*📦\s*Services/i,
          /^##\s*Services/i
        ],
        'topics': [
          /^##\s*📡\s*Topics/i,
          /^##\s*Topics/i
        ]
      }
      
      const patterns = sectionPatterns[props.section] || []
      console.log(`[MarkdownInclude] Looking for patterns:`, patterns.map(p => p.toString()))
      
      // 先打印所有h2标题，方便调试
      const h2Lines = lines.map((l, i) => ({ line: l.trim(), index: i })).filter(({ line }) => line.startsWith('## '))
      console.log(`[MarkdownInclude] Found ${h2Lines.length} h2 headings:`, h2Lines.slice(0, 5).map(({ line, index }) => `${index}: ${line}`))
      
      for (let i = 0; i < lines.length; i++) {
        const line = lines[i].trim()
        
        // 查找section开始
        if (sectionStart === -1) {
          for (const pattern of patterns) {
            if (pattern.test(line)) {
              sectionStart = i
              console.log(`[MarkdownInclude] Found section start at line ${i}: "${line}"`)
              break
            }
          }
        } else {
          // 查找下一个同级别的section（h2）或文档结束
          // 检查是否是另一个section的开始（services或topics）
          if (line.startsWith('## ')) {
            // 检查是否是另一个section类型
            const isOtherSection = Object.keys(sectionPatterns).some(key => {
              if (key === props.section) return false
              return sectionPatterns[key].some(p => p.test(line))
            })
            
            if (isOtherSection) {
              sectionEnd = i
              console.log(`[MarkdownInclude] Found section end at line ${i}: "${line}"`)
              break
            }
          }
        }
      }
      
      if (sectionStart !== -1) {
        if (sectionEnd === -1) {
          sectionEnd = lines.length
        }
        const beforeLength = content.length
        content = lines.slice(sectionStart, sectionEnd).join('\n')
        console.log(`[MarkdownInclude] ✓ Extracted section "${props.section}" from ${props.src}: ${sectionEnd - sectionStart} lines (${beforeLength} -> ${content.length} chars)`)
      } else {
        // 如果找不到section，返回空内容
        console.warn(`[MarkdownInclude] ⚠ Section "${props.section}" not found in ${props.src}. Available sections:`, 
          lines.filter(l => l.trim().startsWith('## ')).map(l => l.trim()).slice(0, 5))
        content = ''
      }
    }
    
    // 跳过第一个标题（H1或H2）- 在section提取之后执行
    if (props.skipTitle) {
      const lines = content.split('\n')
      let startIdx = 0
      for (let i = 0; i < lines.length; i++) {
        const line = lines[i].trim()
        // 跳过第一个标题，无论是 # 还是 ##
        if (line.startsWith('# ') || line.startsWith('## ')) {
          startIdx = i + 1
          while (startIdx < lines.length && lines[startIdx].trim() === '') {
            startIdx++
          }
          console.log(`[MarkdownInclude] Skipping title at line ${i}: "${line}"`)
          break
        }
      }
      content = lines.slice(startIdx).join('\n')
    }
    
    console.log(`[MarkdownInclude] Content after processing: ${content.length} chars`)
    
    // 使用markdown-it渲染
    if (content.length > 0) {
      renderedContent.value = md.render(content)
      console.log(`[MarkdownInclude] ✓ Rendered content, HTML length: ${renderedContent.value.length}`)
      // 检查是否包含表格
      const hasTable = renderedContent.value.includes('<table') || renderedContent.value.includes('<thead')
      console.log(`[MarkdownInclude] Contains table: ${hasTable}`)
      if (hasTable) {
        console.log(`[MarkdownInclude] Table HTML preview:`, renderedContent.value.match(/<table[\s\S]{0,200}/)?.[0])
      }
    } else {
      console.warn(`[MarkdownInclude] ⚠ No content to render`)
      renderedContent.value = ''
    }
    loading.value = false
    console.log(`[MarkdownInclude] ✓ Component loading complete`)
  } catch (err) {
    console.error('Error loading markdown:', err)
    error.value = `Failed to load ${props.src}: ${err.message}`
    loading.value = false
  }
})
</script>

<style scoped>
.markdown-include {
  width: 100%;
}

.markdown-include.max-width-limited {
  margin: 0 auto;
  padding: 0 1rem;
}

.loading {
  padding: 2rem;
  text-align: center;
  color: var(--vp-c-text-2);
}

.error {
  color: var(--vp-c-danger);
  padding: 1rem;
  background: var(--vp-c-bg-soft);
  border-radius: 4px;
  border-left: 4px solid var(--vp-c-danger);
  margin: 1rem 0;
}

.markdown-body {
  width: 100%;
}

/* MarkdownInclude 组件内的表格样式 */
.markdown-body :deep(table) {
  width: 100% !important;
  border-collapse: collapse !important;
  margin: 1.5rem 0 !important;
  background: var(--vp-c-bg) !important;
  border-radius: 8px !important;
  overflow: hidden !important;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08) !important;
  display: table !important;
}

.markdown-body :deep(table thead) {
  background: linear-gradient(135deg, var(--vp-c-brand-1) 0%, var(--vp-c-brand-2) 100%) !important;
}

.markdown-body :deep(table th) {
  background: linear-gradient(135deg, var(--vp-c-brand-1) 0%, var(--vp-c-brand-2) 100%) !important;
  color: white !important;
  padding: 14px 18px !important;
  text-align: left !important;
  font-weight: 600 !important;
  font-size: 0.9em !important;
  letter-spacing: 0.5px !important;
  border: none !important;
}

.markdown-body :deep(table td) {
  padding: 14px 18px !important;
  border-bottom: 1px solid var(--vp-c-divider) !important;
  transition: background-color 0.2s !important;
  word-wrap: break-word !important;
  overflow-wrap: break-word !important;
}

.markdown-body :deep(table tr:last-child td) {
  border-bottom: none !important;
}

.markdown-body :deep(table tr:hover) {
  background: var(--vp-c-bg-soft) !important;
}

.markdown-body :deep(table td:first-child),
.markdown-body :deep(table th:first-child) {
  font-weight: 600 !important;
  color: var(--vp-c-text-2) !important;
  width: 20% !important;
  min-width: 100px !important;
  max-width: 200px !important;
  background: var(--vp-c-bg-soft) !important;
}

.markdown-body :deep(table td:last-child),
.markdown-body :deep(table th:last-child) {
  width: 80% !important;
}

.markdown-body :deep(table code) {
  background: var(--vp-c-bg-alt) !important;
  padding: 3px 8px !important;
  border-radius: 4px !important;
  font-size: 0.9em !important;
  color: var(--vp-c-brand-1) !important;
  font-family: 'Consolas', 'Monaco', 'Courier New', monospace !important;
}

.markdown-body :deep(table strong) {
  font-weight: 600 !important;
  color: var(--vp-c-text-1) !important;
}

/* VitePress 容器样式 - 匹配原生样式 */
.markdown-body :deep(.vp-custom-container) {
  margin: 1rem 0;
  padding: 0.1rem 1.5rem;
  border-radius: 0.4rem;
  border-left: 0.25rem solid;
  position: relative;
  overflow: hidden;
  transition: color 0.5s, background-color 0.5s, border-color 0.5s;
}

.markdown-body :deep(.vp-custom-container.info) {
  border-color: var(--vp-c-brand-1);
  background-color: var(--vp-c-brand-soft);
  color: var(--vp-c-text-1);
}

.markdown-body :deep(.vp-custom-container.tip) {
  border-color: var(--vp-c-tip);
  background-color: var(--vp-c-tip-soft);
  color: var(--vp-c-text-1);
}

.markdown-body :deep(.vp-custom-container.warning) {
  border-color: var(--vp-c-warning);
  background-color: var(--vp-c-warning-soft);
  color: var(--vp-c-text-1);
}

.markdown-body :deep(.vp-custom-container.danger) {
  border-color: var(--vp-c-danger);
  background-color: var(--vp-c-danger-soft);
  color: var(--vp-c-text-1);
}

.markdown-body :deep(.vp-custom-container.details) {
  border-color: var(--vp-c-divider);
  background-color: var(--vp-c-bg-soft);
}

.markdown-body :deep(.vp-custom-container p) {
  margin: 0.5rem 0;
  line-height: 1.75;
}

.markdown-body :deep(.vp-custom-container p:first-child) {
  margin-top: 0;
}

.markdown-body :deep(.vp-custom-container p:last-child) {
  margin-bottom: 0;
}

.markdown-body :deep(.vp-custom-container ul),
.markdown-body :deep(.vp-custom-container ol) {
  margin: 0.5rem 0;
  padding-left: 1.5rem;
}

.markdown-body :deep(.vp-custom-container strong) {
  font-weight: 600;
}
</style>
