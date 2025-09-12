<template>
  <div class="ros-connection-config">
    <div class="config-header">
      <h3>ROS 连接配置</h3>
      <div class="connection-status" :class="{ connected: isConnected, disconnected: !isConnected }">
        {{ isConnected ? '已连接' : '未连接' }}
      </div>
    </div>

    <div class="form-group">
      <div class="input-group">
        <label for="ros-url">机器人IP地址:</label>
        <input 
          id="ros-url" 
          v-model="rosUrl" 
          type="text" 
          placeholder="ws://localhost:9090"
          :disabled="isConnected"
        />
        <button 
          @click="toggleConnection" 
          class="action-btn" 
          :class="{ disconnect: isConnected, connect: !isConnected }"
        >
          {{ isConnected ? '断开连接' : '连接' }}
        </button>
      </div>
    </div>

    <div class="config-form">
      <div class="form-group">
        <label for="reconnect-interval">重连间隔 (毫秒):</label>
        <input 
          id="reconnect-interval" 
          v-model.number="reconnectInterval" 
          type="number" 
          min="1000"
          :disabled="isConnected"
        />
      </div>

      <div class="form-group">
        <label for="max-reconnect">最大重连次数:</label>
        <input 
          id="max-reconnect" 
          v-model.number="maxReconnectAttempts" 
          type="number" 
          min="1"
          :disabled="isConnected"
        />
      </div>

      <div class="form-group">
        <label for="timeout">连接超时 (毫秒):</label>
        <input 
          id="timeout" 
          v-model.number="timeout" 
          type="number" 
          min="1000"
          :disabled="isConnected"
        />
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, reactive, onMounted, onBeforeUnmount } from 'vue'
import rosConnection from '../utils/rosConnection.js'
import rosConfig from '../config/rosConfig.js'

// 表单数据
const rosUrl = ref(rosConfig.url)
const reconnectInterval = ref(rosConfig.reconnectInterval)
const maxReconnectAttempts = ref(rosConfig.maxReconnectAttempts)
const timeout = ref(rosConfig.timeout)
const isConnected = ref(false)
const errorMessage = ref('')

// 连接状态监听
const updateConnectionStatus = () => {
  isConnected.value = rosConnection.getConnectionStatus()
}

// 连接/断开连接
const toggleConnection = async () => {
  try {
    if (isConnected.value) {
      rosConnection.disconnect()
    } else {
      // 更新配置
      rosConfig.url = rosUrl.value
      rosConfig.reconnectInterval = reconnectInterval.value
      rosConfig.maxReconnectAttempts = maxReconnectAttempts.value
      rosConfig.timeout = timeout.value

      // 连接 ROS
      await rosConnection.connect()
    }
    updateConnectionStatus()
    errorMessage.value = ''
  } catch (error) {
    errorMessage.value = `连接失败: ${error.message}`
    console.error('ROS 连接错误:', error)
  }
}

// 保存配置
const saveConfig = () => {
  // 在实际应用中，这里可以将配置保存到本地存储或发送到服务器
  localStorage.setItem('rosConfig', JSON.stringify({
    url: rosUrl.value,
    reconnectInterval: reconnectInterval.value,
    maxReconnectAttempts: maxReconnectAttempts.value,
    timeout: timeout.value
  }))

  alert('配置已保存')
}

// 重置配置
const resetConfig = () => {
  rosUrl.value = 'ws://localhost:9090'
  reconnectInterval.value = 3000
  maxReconnectAttempts.value = 10
  timeout.value = 5000
}

// 加载保存的配置
const loadSavedConfig = () => {
  try {
    const savedConfig = localStorage.getItem('rosConfig')
    if (savedConfig) {
      const config = JSON.parse(savedConfig)
      rosUrl.value = config.url || 'ws://localhost:9090'
      reconnectInterval.value = config.reconnectInterval || 3000
      maxReconnectAttempts.value = config.maxReconnectAttempts || 10
      timeout.value = config.timeout || 5000
    }
  } catch (error) {
    console.error('加载配置失败:', error)
  }
}

// 设置事件监听
onMounted(() => {
  // 加载保存的配置
  loadSavedConfig()

  // 更新连接状态
  updateConnectionStatus()

  // 监听连接状态变化
  rosConnection.onConnection(updateConnectionStatus)
  rosConnection.onClose(updateConnectionStatus)
  rosConnection.onError((error) => {
    updateConnectionStatus()
    errorMessage.value = `连接错误: ${error.message || '无法连接到 ROS Bridge'}`
  })
})

onBeforeUnmount(() => {
  // 组件卸载时断开连接
  if (isConnected.value) {
    rosConnection.disconnect()
  }
})
</script>

<style scoped>
.ros-connection-config {
  max-width: 600px;
  margin: 0 auto;
  padding: 20px;
  border: 1px solid #e0e0e0;
  border-radius: 8px;
  background-color: #f9f9f9;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
}

.config-header {
  display: none;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 20px;
  padding-bottom: 10px;
  border-bottom: 1px solid #e0e0e0;
}

.config-header h3 {
  margin: 0;
  color: #333;
}

.connection-status {
  padding: 5px 10px;
  border-radius: 4px;
  font-weight: bold;
  font-size: 14px;
}

.connected {
  background-color: #d4edda;
  color: #155724;
}

.disconnected {
  background-color: #f8d7da;
  color: #721c24;
}

.config-form {
  margin-bottom: 20px;
  display: none;
}

.form-group {
  margin-bottom: 15px;
}

.form-group label {
  display: block;
  margin-bottom: 5px;
  font-weight: 500;
  color: #333;
}

.input-group {
  display: flex;
  gap: 10px;
}

input {
  flex: 1;
  padding: 8px 12px;
  border: 1px solid #ddd;
  border-radius: 4px;
  font-size: 14px;
}

input:disabled {
  background-color: #f0f0f0;
  cursor: not-allowed;
}

.action-btn {
  padding: 8px 15px;
  border: none;
  border-radius: 4px;
  font-weight: 500;
  cursor: pointer;
  transition: background-color 0.2s;
}

.connect {
  background-color: #007bff;
  color: white;
}

.connect:hover {
  background-color: #0069d9;
}

.disconnect {
  background-color: #dc3545;
  color: white;
}

.disconnect:hover {
  background-color: #c82333;
}

.config-actions {
  display: flex;
  gap: 10px;
  margin-bottom: 15px;
}

.save-btn, .reset-btn {
  padding: 8px 15px;
  border: none;
  border-radius: 4px;
  font-weight: 500;
  cursor: pointer;
  transition: background-color 0.2s;
}

.save-btn {
  background-color: #28a745;
  color: white;
}

.save-btn:hover:not(:disabled) {
  background-color: #218838;
}

.reset-btn {
  background-color: #6c757d;
  color: white;
}

.reset-btn:hover:not(:disabled) {
  background-color: #5a6268;
}

.save-btn:disabled, .reset-btn:disabled {
  background-color: #b8b8b8;
  cursor: not-allowed;
}

.error-message {
  padding: 10px;
  margin-top: 10px;
  background-color: #f8d7da;
  color: #721c24;
  border-radius: 4px;
  font-size: 14px;
}
</style>
