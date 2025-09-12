/**
 * Frontmatter parsing utilities for Markdown content
 */

// 改进的 Frontmatter 解析函数，支持嵌套结构
export const parseFrontmatter = (content) => {
  const frontmatterRegex = /^---\s*\n([\s\S]*?)\n---\s*\n/
  const match = content.match(frontmatterRegex)
  
  if (!match) {
    return {
      frontmatter: {},
      content: content
    }
  }
  
  const frontmatterContent = match[1]
  const markdownContent = content.slice(match[0].length)
  
  // 解析嵌套的 YAML 结构
  const frontmatter = parseNestedYaml(frontmatterContent)
  
  return {
    frontmatter,
    content: markdownContent
  }
}

// 解析嵌套的 YAML（简化版本）
export const parseNestedYaml = (yamlContent) => {
  const result = {}
  const lines = yamlContent.split('\n')
  let currentObject = result
  const stack = [result]
  const indentStack = [-1]
  
  try {
    lines.forEach(line => {
      if (!line.trim() || line.trim().startsWith('#')) return
      
      const indent = line.length - line.trimLeft().length
      const trimmed = line.trim()
      
      // 处理缩进变化
      while (indentStack.length > 1 && indent <= indentStack[indentStack.length - 1]) {
        stack.pop()
        indentStack.pop()
      }
      currentObject = stack[stack.length - 1]
      
      const colonIndex = trimmed.indexOf(':')
      if (colonIndex === -1) return
      
      const key = trimmed.slice(0, colonIndex).trim()
      let value = trimmed.slice(colonIndex + 1).trim()
      
      if (value === '') {
        // 这是一个嵌套对象的开始
        const nestedObject = {}
        currentObject[key] = nestedObject
        stack.push(nestedObject)
        indentStack.push(indent)
        currentObject = nestedObject
      } else {
        // 处理值
        value = parseValue(value)
        currentObject[key] = value
      }
    })
  } catch (error) {
    console.warn('Nested YAML parsing error:', error)
    return {}
  }
  
  return result
}

// 解析单个值
export const parseValue = (value) => {
  // 处理引号
  if ((value.startsWith('"') && value.endsWith('"')) || 
      (value.startsWith("'") && value.endsWith("'"))) {
    return value.slice(1, -1)
  }
  
  // 处理布尔值
  if (value === 'true') return true
  if (value === 'false') return false
  
  // 处理 null
  if (value === 'null' || value === '~') return null
  
  // 处理数字
  if (!isNaN(value) && !isNaN(parseFloat(value))) {
    return parseFloat(value)
  }
  
  // 处理数组（简单格式：[item1, item2]）
  if (value.startsWith('[') && value.endsWith(']')) {
    try {
      return JSON.parse(value)
    } catch (e) {
      return value
    }
  }
  
  return value
}
