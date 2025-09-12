/**
 * Markmap utility functions for height calculation, folding, and node operations
 */

// 递归计算当前展开状态下可见节点的高度
export const calculateVisibleContentHeight = (root) => {
  const calculateNodeHeight = (node) => {
    const nodeHeight = 20 // 每个节点的基础高度
    let totalHeight = nodeHeight
    
    const isExpanded = node.payload && node.payload.fold != 1
    if (isExpanded && node.children && node.children.length > 0) {
      node.children.forEach(child => {
        totalHeight += calculateNodeHeight(child)
      })
    }
    return totalHeight
  }

  return calculateNodeHeight(root)
}

// 更新 SVG 高度
export const updateSvgHeight = (root, svgRef, minHeight) => {
  const contentHeight = calculateVisibleContentHeight(root)
  const height = Math.max(contentHeight, minHeight)
  svgRef.style.height = `${height}px`

  const container = svgRef.closest('.markmap-container')
  if (container) {
    container.style.height = `${height}px`
  }
}

// 裁剪 markmap 的树，只保留指定深度
export const setDefaultFold = (node, maxDepth, currentDepth = 0) => {
  if (!node.payload) node.payload = {}
  
  if (currentDepth >= maxDepth) {
    node.payload.fold = 1 // 折叠
  } else {
    node.payload.fold = 0 // 展开
  }

  if (node.children) {
    node.children.forEach(child =>
      setDefaultFold(child, maxDepth, currentDepth + 1)
    )
  }
}

// 从 root 开始获取到 target 的路径（包含 root 和 target）
export const getNodePath = (root, target) => {
  const path = []
  let found = false

  const dfs = (node) => {
    if (found) return
    path.push(node)
    if (node === target) {
      found = true
      return
    }
    if (node.children) {
      for (const child of node.children) {
        dfs(child)
        if (found) return
      }
    }
    path.pop()
  }

  dfs(root)
  return found ? path : []
}

// 查找节点的父节点
export const findParentNode = (root, targetNode) => {
  const findParent = (node, parent = null) => {
    if (node.children) {
      for (const child of node.children) {
        if (child === targetNode) {
          return parent
        }
        const found = findParent(child, child)
        if (found) return found
      }
    }
    return null
  }
  
  return findParent(root)
}