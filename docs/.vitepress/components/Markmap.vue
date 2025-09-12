<template>
  <div class="markmap-container">
    <svg ref="svgRef" class="markmap-svg"></svg>
  </div>
</template>

<script setup>
import { ref, onMounted, watch, onBeforeUnmount } from 'vue'
import { Markmap, deriveOptions } from 'markmap-view'
import { Transformer } from 'markmap-lib'
import ROSLIB from 'roslib'
import { parseFrontmatter } from '../utils/frontmatter.js'
import rosConnection from '../utils/rosConnection.js'
import { 
  updateSvgHeight, 
  setDefaultFold,
  getNodePath,
  findParentNode
} from '../utils/markmap-utils.js'
import yaml from 'js-yaml';

const props = defineProps({
  content: { type: String, required: true },
  jsonOptions: { type: Object, default: () => ({}) },
  minWidth: { type: Number, default: 300 },
  minHeight: { type: Number, default: 200 },
})

const svgRef = ref()
let mm = null

let rosRef = null

const initRosConnection = async () => {
  try {
    await rosConnection.connect()
    rosRef = rosConnection.getRosInstance()
    console.log('ROS 连接已初始化')
    
    window.availableTopics = []
    window.availableServices = []
    window.rosDataLoaded = false
    
    // 并行获取话题和服务列表
    const [topicsResult, servicesResult] = await Promise.all([
      new Promise((resolve, reject) => {
        const topicsClient = new ROSLIB.Service({
          ros: rosRef,
          name: '/rosapi/topics',
          serviceType: 'rosapi/Topics'
        })
        topicsClient.callService(
          new ROSLIB.ServiceRequest({}),
          resolve,
          reject
        )
      }),
      new Promise((resolve, reject) => {
        const servicesClient = new ROSLIB.Service({
          ros: rosRef,
          name: '/rosapi/services',
          serviceType: 'rosapi/Services'
        })
        servicesClient.callService(
          new ROSLIB.ServiceRequest({}),
          resolve,
          reject
        )
      })
    ])

    window.availableTopics = (topicsResult.topics || [])
          .filter(topic => topic.startsWith('/zj_humanoid'))
    // console.log('机器人上所有的话题列表:', window.availableTopics)


    window.availableServices = (servicesResult.services || [])
          .filter(service => service.startsWith('/zj_humanoid'))
    // console.log('机器人上所有的服务列表:', window.availableServices)

    window.rosDataLoaded = true

  } catch (error) {
    console.error('ROS 连接初始化失败:', error)
  }
}

// 在 renderMarkmap 函数中添加这个函数
const applyNodeColors = (root) => {
  const traverse = (node) => {
    // 如果节点有自定义颜色
    if (node.payload?.color) {
      // console.log(node)
      // 查找对应的 SVG 节点并应用颜色
      const targetNode = mm.svg.selectAll('.markmap-node')
        .filter(function(d) {
          // console.log(d.payload.lines, node.payload.lines);
          return d.payload.lines === node.payload.lines; // 直接比较数据对象
        });
      
      if (!targetNode.empty()) {
        // console.log(targetNode);
        // 应用背景颜色到圆圈
        targetNode.select('circle')
          .style('fill', node.payload.color);

        // 应用颜色到下划线
        targetNode.select('line')
          .style('stroke', node.payload.color);
        
        // console.log(`✅ 已应用颜色 ${node.payload.color} 到节点: ${node.content}`);
      }
    }
    // 递归处理子节点
    if (node.children) {
      node.children.forEach(traverse);
    }
  };
  
  traverse(root);
};

const transformer = new Transformer()

// 添加按键样式和点击事件
const addButtonFunctionality = (root) => {
  // 查找所有 demos 节点
  const findAllDemosNodes = (node, demosNodes = []) => {
    if (node.content === 'demos') {
      demosNodes.push(node)
    }
    if (node.children) {
      for (const child of node.children) {
        findAllDemosNodes(child, demosNodes)
      }
    }
    return demosNodes
  }

  const allDemosNodes = findAllDemosNodes(root)

  // 为所有 demos 节点创建 rosbridge 通讯节点，并为其子节点添加按键标识
  allDemosNodes.forEach((demosNode) => {
    // 搜寻兄弟节点的功能
    const parent = findParentNode(root, demosNode)
    if (parent) {
      // 先找到type字段
      const typeNode = parent.children?.find(child => child.content === 'type')

      // 找到msg_type
      const msgTypeNode = parent.children?.find(child => child.content === 'msg_type')
      if (msgTypeNode && msgTypeNode.children) {
        // 计算msg_type的名称
        const path = getNodePath(root, parent)
        const secondLevelName = path.length >= 2 ? path[1].content : 'unknown'
        const childNames = msgTypeNode.children.map(c => c.content)
        const msgType = childNames.map(childName => 
          childName.includes('/') ? childName : `zj_humanoid${secondLevelName}/${childName}`
        )
        // 组装路径（不含自身，无分隔符）
        const pathToMsgType = getNodePath(root, msgTypeNode)
        const toppicName = pathToMsgType.slice(0, -1).map(n => n.content).join('')

        // 为当前 demosNode 自身创建 ROS 通讯对象（不针对子节点）
        const defaultTopicType = 'std_msgs/String'
        const defaultServiceType = 'std_srvs/Trigger'
        const firstType = msgType[0] || defaultTopicType
        const commType = (typeNode && typeNode.children?.[0]?.content) || 'Topic/Publish'
        // 检查是否已经获取到话题和服务列表
        if (!window.rosDataLoaded) {
          console.log('ROS 数据尚未加载，跳过创建 ROS 实体')
          return
        }
        try {
          if (String(commType).toLowerCase().includes('service')) {
            demosNode.payload = demosNode.payload || {}
            demosNode.payload.rosService = new ROSLIB.Service({
              ros: rosRef,
              name: toppicName,
              serviceType: firstType || defaultServiceType,
            })
            // commType是'service'的检查，检查result.services里是否包含toppicName
            const isServiceAvailable = window.availableServices.some(service => service === toppicName)
            
            if (!isServiceAvailable) {
              // console.warn(`服务 ${toppicName} 在机器人上不可用`)
              // 将不可用的节点标记为灰色
              parent.payload.color = '#95a5a6'
            } else {
              // 为可用的服务类型的父节点设置蓝色
              parent.payload.color = '#3498db'
              // console.log(`发现服务 ${toppicName}`)
            }

          } else {
            demosNode.payload = demosNode.payload || {}
            demosNode.payload.rosTopic = new ROSLIB.Topic({
              ros: rosRef,
              name: toppicName,
              messageType: firstType || defaultTopicType,
            })
            //是topic的）检查result.topics里是否包含toppicName
            const isTopicAvailable = window.availableTopics.some(topic => topic === toppicName)

            if (!isTopicAvailable) {
              // console.warn(`话题 ${toppicName} 在机器人上不可用`)
              // 将不可用的节点标记为灰色
              parent.payload.color = '#95a5a6'
            } else {
              // 为可用的主题类型的父节点设置绿色
              parent.payload.color = '#2ecc71'
              // console.log(`发现话题 ${toppicName}`)
            }
          }
          demosNode.payload.color = parent.payload.color
          //console.log(demosNode.payload)
        } catch (e) {
          console.warn('Create ROS entity failed:', e)
        }
      }
    }

    // 子节点仅标记为按钮，不创建 ROS 实体
    if (demosNode.children) {
      demosNode.children.forEach((child) => {
        if (!child.payload) child.payload = {}
        child.payload.isButton = true
      })
    }
  })
}

// 添加按键点击事件
const addButtonClickEvents = (root) => {
  // 遍历所有 isButton 节点
  const traverse = (node) => {
    if (node.payload?.isButton ) {
      // 查找父节点
      const path = getNodePath(root, node)
      const parent = path.length >= 2 ? path[path.length - 2] : null
      if(parent?.payload?.fold){
        return
      }
      
      const targetNode = mm.svg.selectAll('.markmap-node')
        .filter(function() {
          const textContent = this.textContent || ''
          return textContent.trim() === node.content
        })
      
      if (!targetNode.empty()) {
        targetNode.style("cursor", "pointer")
        targetNode.on("click", (event) => {
          event.stopPropagation()
          
          
          // 收集子节点文字
          let text = ""
          if (node.children?.length) {
            text = node.children.map(c => c.content).join("\n")
          } 
          console.log('demos 调用:', node.content,"request:",text)
          // 通过 ROS 发送消息
          if(parent.payload.rosService){
            const yamlObject = yaml.load(text);
            parent.payload.rosService.callService(new ROSLIB.ServiceRequest(yamlObject), (result) => {
              console.log('服务调用成功:', result)
            }, (error) => {
              console.error('服务调用失败:', error)
            })
          } else if(parent.payload.rosTopic){
            parent.payload.rosTopic.publish(new ROSLIB.Message({ data: text }))
          }
        })
        
        // console.log('✅ 已为按键', node.content, '添加样式和点击事件')
      } else {
        console.log('❌ 没有找到匹配的节点:', node.content)
      }
    }
    
    if (node.children) node.children.forEach(traverse)
  }

  traverse(root)
}

// 渲染 markmap
const renderMarkmap = () => {
  if (!svgRef.value || !props.content) return

  try {
    const { frontmatter, content } = parseFrontmatter(props.content)
    const { root } = transformer.transform(content)

    // 如果已有 markmap 实例，销毁它
    if (mm) mm.destroy()

    // 合并配置
    let jsonOptions = { ...props.jsonOptions }
    Object.entries(frontmatter).forEach(([k, v]) => {
      if (k !== 'markmap') jsonOptions[k] = v
    })
    if (frontmatter.markmap) {
      jsonOptions = { ...jsonOptions, ...frontmatter.markmap }
    }

    const markmapOptions = deriveOptions(jsonOptions)
    // console.log('markmapOptions', jsonOptions)

    mm = Markmap.create(svgRef.value, markmapOptions, root)
    addButtonFunctionality(root)
    console.log('Markmap rendered:', root)
    applyNodeColors(root);
    // 初次高度计算
    updateSvgHeight(root, svgRef.value, props.minHeight)
    mm.fit()
    // 添加按键功能
    mm.svg.on('click', () => {
      requestAnimationFrame(() => {
        updateSvgHeight(root, svgRef.value, props.minHeight)
        mm.fit()
        // 添加按键点击事件
        addButtonClickEvents(root)
        applyNodeColors(root);
      })
    })

    // 监听窗口大小变化
    window.addEventListener('resize', () => {
      updateSvgHeight(root, svgRef.value, props.minHeight)
      mm.fit()
    })

  } catch (err) {
    console.error('Markmap rendering error:', err)
  }
}

onMounted(() => {
  // 不等待 ROS 连接，直接渲染 markmap
  renderMarkmap()
  
  // 异步初始化 ROS 连接，不阻塞页面渲染
  initRosConnection().then(() => {
    console.log('ROS 连接已就绪，可以与 ROS 交互')
    renderMarkmap()
    // 如果需要在 ROS 连接就绪后重新渲染某些内容，可以在这里调用
  }).catch(error => {
    console.error('ROS 连接初始化失败:', error)
  })
})

watch(() => props.content, renderMarkmap, { immediate: true })
watch(() => props.jsonOptions, renderMarkmap)
</script>

<style scoped>
.markmap-container {
  /* width: 100%; */
  position: relative;
  display: flex;
}
.markmap-svg {
  width: 100%;
  height: auto;
  display: block;
}
</style>