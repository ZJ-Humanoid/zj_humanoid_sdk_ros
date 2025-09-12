/**
 * ROS 连接管理器
 * 负责管理与 ROSBridge 的连接
 */

import ROSLIB from 'roslib';
import rosConfig from '../config/rosConfig.js';

class RosConnection {
  constructor() {
    this.ros = null;
    this.isConnected = false;
    this.reconnectAttempts = 0;
    this.connectionCallbacks = [];
    this.errorCallbacks = [];
    this.closeCallbacks = [];
  }

  /**
   * 连接到 ROSBridge
   * @returns {Promise} 连接成功的 Promise
   */
  connect() {
    return new Promise((resolve, reject) => {
      if (this.isConnected && this.ros) {
        resolve(this.ros);
        return;
      }

      // 创建 ROS 实例
      this.ros = new ROSLIB.Ros({
        url: rosConfig.url,
      });

      // 设置连接事件监听
      this.ros.on('connection', () => {
        console.log('Connected to ROSBridge.');
        this.isConnected = true;
        this.reconnectAttempts = 0;
        this.connectionCallbacks.forEach(callback => callback());
        resolve(this.ros);
      });

      this.ros.on('error', (error) => {
        console.error('Error connecting to ROSBridge:', error);
        this.isConnected = false;
        this.errorCallbacks.forEach(callback => callback(error));

        // 尝试重新连接
        this.attemptReconnect();
      });

      this.ros.on('close', () => {
        console.log('Connection to ROSBridge closed.');
        this.isConnected = false;
        this.closeCallbacks.forEach(callback => callback());

        // 尝试重新连接
        this.attemptReconnect();
      });
    });
  }

  /**
   * 尝试重新连接
   */
  attemptReconnect() {
    if (this.reconnectAttempts >= rosConfig.maxReconnectAttempts) {
      console.error('Max reconnection attempts reached.');
      return;
    }

    this.reconnectAttempts++;
    console.log(`Attempting to reconnect (${this.reconnectAttempts}/${rosConfig.maxReconnectAttempts})...`);

    setTimeout(() => {
      this.connect().catch(err => {
        console.error('Reconnection failed:', err);
      });
    }, rosConfig.reconnectInterval);
  }

  /**
   * 断开连接
   */
  disconnect() {
    if (this.ros) {
      this.ros.close();
      this.ros = null;
      this.isConnected = false;
    }
  }

  /**
   * 添加连接成功回调
   * @param {Function} callback 回调函数
   */
  onConnection(callback) {
    if (typeof callback === 'function') {
      this.connectionCallbacks.push(callback);
    }
  }

  /**
   * 添加错误回调
   * @param {Function} callback 回调函数
   */
  onError(callback) {
    if (typeof callback === 'function') {
      this.errorCallbacks.push(callback);
    }
  }

  /**
   * 添加连接关闭回调
   * @param {Function} callback 回调函数
   */
  onClose(callback) {
    if (typeof callback === 'function') {
      this.closeCallbacks.push(callback);
    }
  }

  /**
   * 获取当前连接状态
   * @returns {boolean} 是否已连接
   */
  getConnectionStatus() {
    return this.isConnected;
  }

  /**
   * 获取 ROS 实例
   * @returns {ROSLIB.Ros|null} ROS 实例
   */
  getRosInstance() {
    return this.ros;
  }
}

// 创建单例实例
const rosConnection = new RosConnection();

export default rosConnection;
