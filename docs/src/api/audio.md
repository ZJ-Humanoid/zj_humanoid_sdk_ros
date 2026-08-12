---
title: AUDIO 子系统
description: AUDIO 子系统的所有ROS接口
---

# 🔊 AUDIO 子系统

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
## 📦 Services (9)
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
  - stop
## 📡 Topics (11)
- asr_text
- listen
- listen_state
- microphone
  - audio_data_raw
  - status
  - wake_data
  - wake_info
- speaker
  - pcm_play
  - playback_status
  - ref_data
  - status
## ⚙️ Actions (2)
- LLM_chat
- media_play`
</script>

---

## 📦 Services (9)

### 1. `microphone/get_devices_list`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/microphone/get_devices_list |
| **Type** | [audio/GetDeviceList](../zj_humanoid_types#getdevicelist) |
| **Description** | 麦克风列表 |
| **Note** | 检查当前有多少个麦克风设备 回复数量应大于1 |

### 2. `microphone/select_device`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/microphone/select_device |
| **Type** | [audio/SetDevice](../zj_humanoid_types#setdevice) |
| **Description** | 选中麦克风 |
| **Note** | 选择第一个麦克风 |

### 3. `speaker/get_devices_list`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/speaker/get_devices_list |
| **Type** | [audio/GetDeviceList](../zj_humanoid_types#getdevicelist) |
| **Description** | 获取播放设备 |
| **Note** | 检查当前有多少个喇叭设备 回复数量应大于1 |

### 4. `speaker/get_volume`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/speaker/get_volume |
| **Type** | [audio/GetVolume](../zj_humanoid_types#getvolume) |
| **Description** | 获取当前音量 |
| **Note** | 获取当前的系统音量大小 应回复音量0~100 |

### 5. `speaker/select_device`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/speaker/select_device |
| **Type** | [audio/SetDevice](../zj_humanoid_types#setdevice) |
| **Description** | 选中生效喇叭 |
| **Note** | 选择第一个喇叭 |

### 6. `speaker/set_volume`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/speaker/set_volume |
| **Type** | [audio/SetVolume](../zj_humanoid_types#setvolume) |
| **Description** | 设置音量大小 |
| **Note** | 设置音量为50 |

### 7. `speaker/stop`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/speaker/stop |
| **Type** | [audio/SpeakerStop](../zj_humanoid_types#speakerstop) |
| **Description** | 停止当前播放并清空播放队列 |
| **Note** | 新一轮 TTS 播放前可调用此服务打断旧队列 |

### 8. `tts_service`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/tts_service |
| **Type** | [audio/TTS](../zj_humanoid_types#tts) |
| **Description** | 文字转语音 |
| **Note** | 请让机器人说'hello world' |

### 9. `version`

| 字段 | 值 |
|------|-----|
| **Service Name** | /zj_humanoid/audio/version |
| **Type** | std_srvs/Trigger |
| **Description** | 语音模块的版本号 |

## 📡 Topics (11)

### 1. `asr_text`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/asr_text |
| **Type** | std_msgs/String |
| **Direction** | 📤 Publish |
| **Description** | 语音转文字 |
| **Note** | 当前机器人听到了什么 |

### 2. `listen`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/listen |
| **Type** | std_msgs/Bool |
| **Direction** | 📥 Subscribe |
| **Description** | 唤醒控制 |
| **Note** | 手动唤醒/关闭唤醒模式，true=唤醒，false=休眠 |

### 3. `listen_state`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/listen_state |
| **Type** | std_msgs/Bool |
| **Direction** | 📤 Publish |
| **Description** | 唤醒倾听状态 |
| **Note** | 当前是否为倾听状态，true=正在倾听，false=未倾听 |

### 4. `microphone/audio_data_raw`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/microphone/audio_data_raw |
| **Type** | [audio/AudioData](../zj_humanoid_types#audiodata) |
| **Direction** | 📤 Publish |
| **Description** | 麦克风原始 PCM 音频流 |
| **Note** | 采样率和声道数以消息字段为准 |

### 5. `microphone/status`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/microphone/status |
| **Type** | [audio/MicStatus](../zj_humanoid_types#micstatus) |
| **Direction** | 📤 Publish |
| **Description** | 麦克风设备与采集状态 |
| **Note** | 默认约 1 Hz 发布，设备切换和错误发生时会及时更新 |

### 6. `microphone/wake_data`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/microphone/wake_data |
| **Type** | [audio/AudioData](../zj_humanoid_types#audiodata) |
| **Direction** | 📤 Publish |
| **Description** | 音频流数据 |
| **Note** | 麦克风收音后的音频数据流 |

### 7. `microphone/wake_info`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/microphone/wake_info |
| **Type** | [audio/AudioData](../zj_humanoid_types#audiodata) |
| **Direction** | 📤 Publish |
| **Description** | 音频流数据 |
| **Note** | 麦克风收音后的音频数据流 |

### 8. `speaker/pcm_play`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/speaker/pcm_play |
| **Type** | [audio/AudioData](../zj_humanoid_types#audiodata) |
| **Direction** | 📥 Subscribe |
| **Description** | PCM 流式播放输入 |
| **Note** | 输入为 S16_LE PCM 数据 |

### 9. `speaker/playback_status`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/speaker/playback_status |
| **Type** | [audio/PlaybackStatus](../zj_humanoid_types#playbackstatus) |
| **Direction** | 📤 Publish |
| **Description** | 播放状态变化通知 |
| **Note** | 状态包含 IDLE、PLAYING 和 STOPPED |

### 10. `speaker/ref_data`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/speaker/ref_data |
| **Type** | [audio/AudioData](../zj_humanoid_types#audiodata) |
| **Direction** | 📤 Publish |
| **Description** | 扬声器软件播放参考数据 |
| **Note** | 用于 AEC 辅助处理 |

### 11. `speaker/status`

| 字段 | 值 |
|------|-----|
| **Topic Name** | /zj_humanoid/audio/speaker/status |
| **Type** | [audio/SpeakerStatus](../zj_humanoid_types#speakerstatus) |
| **Direction** | 📤 Publish |
| **Description** | 扬声器设备与播放状态 |
| **Note** | 默认约 1 Hz 发布 |

## ⚙️ Actions (2)

### 1. `LLM_chat`

| 字段 | 值 |
|------|-----|
| **Action Name** | /zj_humanoid/audio/LLM_chat |
| **Type** | [audio/LLMChat](../zj_humanoid_types#llmchat) |
| **Description** | 大模型对话 |
| **Note** | 支持上下文、语音输出和流式文本反馈 |

### 2. `media_play`

| 字段 | 值 |
|------|-----|
| **Action Name** | /zj_humanoid/audio/media_play |
| **Type** | [audio/MediaPlay](../zj_humanoid_types#mediaplay) |
| **Description** | 播放媒体文件 |
| **Note** | 文件播放倍率范围为 0.5-2.0，播放期间可调用 speaker/stop 中断 |

