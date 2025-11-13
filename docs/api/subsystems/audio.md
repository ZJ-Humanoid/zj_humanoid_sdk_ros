---
title: AUDIO 子系统
description: AUDIO 子系统的所有ROS接口
---

## 📊 接口概览

<Markmap :content="markmapContent" />

<script setup>
const markmapContent = `---
markmap:
  initialExpandLevel: 3
  colorFreezeLevel: 3
  maxWidth: 200
---

# 🔊 AUDIO 子系统
## 📦 Services (11)
- LLM_chat
- listen
- media_play
- tts_service
- version
### microphone
- get_devices_list
- select_device
### speaker
- get_devices_list
- get_volume
- select_device
- set_volume
## 📡 Topics (3)
- asr_text
- audio_data
- listen_state`
</script>

---

## 📦 Services (11)

### 1. `/zj_humanoid/audio/LLM_chat`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/audio/LLM_chat` |
| **Type** | `audio/LLM_chat` |
| **Description** | LLM对话服务 |
| **Note** | 语音模块的版本号是多少 |

### 2. `/zj_humanoid/audio/listen`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/audio/listen` |
| **Type** | `audio/Listen` |
| **Description** | 倾听服务 |
| **Note** | 开始倾听 |

### 3. `/zj_humanoid/audio/media_play`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/audio/media_play` |
| **Type** | `audio/MediaPlay` |
| **Description** | 音频文件播放 |
| **Note** | 播放’公司介绍.mp3‘ |

### 4. `/zj_humanoid/audio/microphone/get_devices_list`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/audio/microphone/get_devices_list` |
| **Type** | `audio/GetDeviceList` |
| **Description** | 麦克风列表 |
| **Note** | 检查当前有多少个麦克风设备 回复数量应大于1 |

### 5. `/zj_humanoid/audio/microphone/select_device`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/audio/microphone/select_device` |
| **Type** | `audio/SetDevice` |
| **Description** | 选中麦克风 |
| **Note** | 选择第一个麦克风 |

### 6. `/zj_humanoid/audio/speaker/get_devices_list`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/audio/speaker/get_devices_list` |
| **Type** | `audio/GetDeviceList` |
| **Description** | 获取播放设备 |
| **Note** | 检查当前有多少个喇叭设备 回复数量应大于1 |

### 7. `/zj_humanoid/audio/speaker/get_volume`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/audio/speaker/get_volume` |
| **Type** | `audio/GetVolume` |
| **Description** | 获取当前音量 |
| **Note** | 获取当前的系统音量大小 应回复音量0~100 |

### 8. `/zj_humanoid/audio/speaker/select_device`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/audio/speaker/select_device` |
| **Type** | `audio/SetDevice` |
| **Description** | 选中生效喇叭 |
| **Note** | 选择第一个喇叭 |

### 9. `/zj_humanoid/audio/speaker/set_volume`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/audio/speaker/set_volume` |
| **Type** | `audio/SetVolume` |
| **Description** | 设置音量大小 |
| **Note** | 设置音量为50 |

### 10. `/zj_humanoid/audio/tts_service`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/audio/tts_service` |
| **Type** | `audio/TTS` |
| **Description** | 文字转语音 |
| **Note** | 请让机器人说‘hello world‘ |

### 11. `/zj_humanoid/audio/version`

| 字段 | 值 |
|------|-----|
| **Service Name** | `/zj_humanoid/audio/version` |
| **Type** | `std_srvs/Trigger` |
| **Description** | 语音模块的版本号 |

## 📡 Topics (3)

### 1. `/zj_humanoid/audio/asr_text`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/audio/asr_text` |
| **Type** | `std_msgs/String` |
| **Direction** | 📤 Publish |
| **Description** | 语音转文字 |
| **Note** | 当前机器人听到了什么 |

### 2. `/zj_humanoid/audio/audio_data`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/audio/audio_data` |
| **Type** | `audio/AudioData` |
| **Direction** | 📤 Publish |
| **Description** | 音频流数据 |
| **Note** | 麦克风收音后的音频数据流 |

### 3. `/zj_humanoid/audio/listen_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | `/zj_humanoid/audio/listen_state` |
| **Type** | `audio/ListenInfo` |
| **Direction** | 📤 Publish |
| **Description** | 唤醒倾听状态 |
| **Note** | 当前是否为倾听状态 |

