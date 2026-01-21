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
## 📦 Services (10)
- LLM_chat
- media_play
- tts_service
- version
- microphone
  - get_devices_list
  - select_device
- speaker
  - get_devices_list
  - get_volume
  - select_device
  - set_volume
## 📡 Topics (4)
- asr_text
- audio_data
- listen
- listen_state`
</script>

---

## 📦 Services (10)

### 1. `LLM_chat`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/LLM_chat |
| **Type** | [audio/LLMChat](../zj_humanoid_types#llmchat) |
| **Description** | LLM对话服务 |
| **Note** | 语音模块的版本号是多少 |

### 2. `media_play`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/media_play |
| **Type** | [audio/MediaPlay](../zj_humanoid_types#mediaplay) |
| **Description** | 音频文件播放 |
| **Note** | 播放'公司介绍.mp3' |

### 3. `microphone/get_devices_list`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/microphone/get_devices_list |
| **Type** | [audio/GetDeviceList](../zj_humanoid_types#getdevicelist) |
| **Description** | 麦克风列表 |
| **Note** | 检查当前有多少个麦克风设备 回复数量应大于1 |

### 4. `microphone/select_device`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/microphone/select_device |
| **Type** | [audio/SetDevice](../zj_humanoid_types#setdevice) |
| **Description** | 选中麦克风 |
| **Note** | 选择第一个麦克风 |

### 5. `speaker/get_devices_list`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/speaker/get_devices_list |
| **Type** | [audio/GetDeviceList](../zj_humanoid_types#getdevicelist) |
| **Description** | 获取播放设备 |
| **Note** | 检查当前有多少个喇叭设备 回复数量应大于1 |

### 6. `speaker/get_volume`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/speaker/get_volume |
| **Type** | [audio/GetVolume](../zj_humanoid_types#getvolume) |
| **Description** | 获取当前音量 |
| **Note** | 获取当前的系统音量大小 应回复音量0~100 |

### 7. `speaker/select_device`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/speaker/select_device |
| **Type** | [audio/SetDevice](../zj_humanoid_types#setdevice) |
| **Description** | 选中生效喇叭 |
| **Note** | 选择第一个喇叭 |

### 8. `speaker/set_volume`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/speaker/set_volume |
| **Type** | [audio/SetVolume](../zj_humanoid_types#setvolume) |
| **Description** | 设置音量大小 |
| **Note** | 设置音量为50 |

### 9. `tts_service`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/tts_service |
| **Type** | [audio/TTS](../zj_humanoid_types#tts) |
| **Description** | 文字转语音 |
| **Note** | 请让机器人说'hello world' |

### 10. `version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/version |
| **Type** | std_srvs/Trigger |
| **Description** | 语音模块的版本号 |

## 📡 Topics (4)

### 1. `asr_text`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/asr_text |
| **Type** | std_msgs/String |
| **Direction** | 📤 Publish |
| **Description** | 语音转文字 |
| **Note** | 当前机器人听到了什么 |

### 2. `audio_data`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/audio_data |
| **Type** | [audio/AudioData](../zj_humanoid_types#audiodata) |
| **Direction** | 📤 Publish |
| **Description** | 音频流数据 |
| **Note** | 麦克风收音后的音频数据流 |

### 3. `listen`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/listen |
| **Type** | std_msgs/Bool |
| **Direction** | 📥 Subscribe |
| **Description** | 唤醒控制 |
| **Note** | 手动唤醒/关闭唤醒模式，true=唤醒，false=休眠 |

### 4. `listen_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/listen_state |
| **Type** | std_msgs/Bool |
| **Direction** | 📤 Publish |
| **Description** | 唤醒倾听状态 |
| **Note** | 当前是否为倾听状态，true=正在倾听，false=未倾听 |

