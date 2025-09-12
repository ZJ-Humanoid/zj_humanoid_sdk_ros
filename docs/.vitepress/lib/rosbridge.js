import ROSLIB from 'roslib'

// Simple ROSBridge wrapper with reconnect and publisher cache
export const useRosBridge = (config) => {
  let ros = null
  let connected = false
  let lastError = null
  const topicCache = new Map()
  let reconnectTimer = null

  const resolveConfig = (override = {}) => ({
    url: 'ws://localhost:9090',
    topic: '/markmap/button_text',
    messageType: 'std_msgs/String',
    ...config,
    ...override
  })

  const connect = (override = {}) => {
    const cfg = resolveConfig(override)
    // cleanup old
    if (ros) try { ros.close() } catch {}
    ros = new ROSLIB.Ros({ url: cfg.url })

    ros.on('connection', () => {
      connected = true
      lastError = null
      // clear publisher cache on new connection
      topicCache.clear()
      // Prime default topic
      getOrCreateTopic(cfg.topic, cfg.messageType)
      // eslint-disable-next-line no-console
      console.log('[ROS] Connected', cfg.url)
    })

    ros.on('error', (err) => {
      connected = false
      lastError = err
      // eslint-disable-next-line no-console
      console.warn('[ROS] Error', err)
    })

    ros.on('close', () => {
      connected = false
      // eslint-disable-next-line no-console
      console.warn('[ROS] Closed')
      if (reconnectTimer) return
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null
        try { connect(cfg) } catch (e) { /* ignore */ }
      }, 2000)
    })
  }

  const getOrCreateTopic = (name, messageType) => {
    const key = `${name}|${messageType}`
    if (topicCache.has(key)) return topicCache.get(key)
    const t = new ROSLIB.Topic({ ros, name, messageType })
    topicCache.set(key, t)
    return t
  }

  const publishText = (text, override = {}) => {
    const cfg = resolveConfig(override)
    if (!connected || !ros) {
      // eslint-disable-next-line no-console
      console.warn('[ROS] Not connected, skip publish')
      return false
    }
    try {
      const topic = getOrCreateTopic(cfg.topic, cfg.messageType)
      const msg = new ROSLIB.Message({ data: String(text ?? '') })
      topic.publish(msg)
      // eslint-disable-next-line no-console
      console.log('[ROS] Published', { topic: cfg.topic, text })
      return true
    } catch (e) {
      lastError = e
      // eslint-disable-next-line no-console
      console.error('[ROS] Publish error', e)
      return false
    }
  }

  const dispose = () => {
    if (reconnectTimer) {
      clearTimeout(reconnectTimer)
      reconnectTimer = null
    }
    try { if (ros) ros.close() } catch {}
    ros = null
    connected = false
    topicCache.clear()
  }

  return {
    connect,
    publishText,
    dispose,
    get connected() { return connected },
    get lastError() { return lastError }
  }
}


