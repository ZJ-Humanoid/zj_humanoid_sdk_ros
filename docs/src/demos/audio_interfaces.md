## tts_service

**Description**

```tex
文本转语音服务接收文本输入并将其转换为语音输出，支持多个语音合成引擎（包括腾讯云、讯飞、百度等）。支持中文、英文等常见语言，默认语言为中文。语速范围为0.5-2.0（默认1.0），音量范围为0-100（默认50）。文本长度建议不超过500字，过长文本可能会分段播放
```

**ROS Type:** `Service`

**Data Type:** `audio/TTS.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rosservice call tts_service "text:
- 'hi'
isPlay: true"

# isPlay: false 只生成文件不播放
# isPlay: true 生成文件并播放
```

**Example:**

```python
#!/usr/bin/env python3
"""
ROS Service Test Script - TTS Service
"""
import rospy
import yaml
import sys
from audio.srv import TTS, TTSRequest

def main():
    # Initialize ROS node
    rospy.init_node('tts_service_test', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for service tts_service...")
    rospy.wait_for_service('tts_service')
    
    try:
        # Create service proxy
        service_proxy = rospy.ServiceProxy('tts_service', TTS)
        
        # Create request
        req = TTSRequest()
        req.text = ["hello world"]  # 文本数组
        req.isPlay = True  # 是否播放，true=播放，false=仅生成文件
        
        rospy.loginfo(f"Calling TTS service with text: {req.text}")
        
        # Call service
        response = service_proxy(req)
        
        rospy.loginfo(f"Response: success={response.success}, file_path={response.file_path}")
        print(f"\nService call successful!")
        print(f"Generated audio file: {response.file_path}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## LLM_chat

**Description**

```tex
智能对话服务用于接收用户文本输入，通过大语言模型生成回复文本，并可联动文本转语音服务自动播放回复。输入文本需为自然语言问句或对话内容，支持上下文关联对话。可通过参数控制是否自动播放语音回复（默认自动播放）。回复生成耗时与网络状态、文本复杂度相关，建议预留1-3秒响应时间
```

**ROS Type:** `Service`

**Data Type:** `audio/LLMChat.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rosservice call LLM_chat "raw_input: '语音模块的版本号是多少'
enable_context: true 
enable_save: true 
context_id: 'test_session'"
```

**Example:**

```python
#!/usr/bin/env python3
"""
ROS Service Test Script - LLM Chat Service
"""
import rospy
import sys
from audio.srv import LLMChat, LLMChatRequest

def main():
    # Initialize ROS node
    rospy.init_node('llm_chat_test', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for service LLM_chat...")
    rospy.wait_for_service('LLM_chat')
    
    try:
        # Create service proxy
        service_proxy = rospy.ServiceProxy('LLM_chat', LLMChat)
        
        # Create request
        req = LLMChatRequest()
        req.raw_input = "语音模块的版本号是多少"  # 用户输入的问题
        req.enable_context = True  # 启用上下文理解
        req.enable_save = True  # 是否记录对话
        req.context_id = "test_session"  # 会话上下文ID
        
        rospy.loginfo(f"Calling LLM chat service with input: {req.raw_input}")
        
        # Call service
        response = service_proxy(req)
        
        rospy.loginfo(f"Response: {response.response}")
        print(f"\nService call successful!")
        print(f"LLM Response: {response.response}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## media_play

**Description**

```tex
音频文件播放服务用于播放本地或指定路径的音频文件，支持暂停、继续、停止等控制操作。支持的音频格式包括MP3、WAV、FLAC，其他格式可能无法正常播放。文件路径需为绝对路径(在docker容器中)，网络文件需先下载至本地再播放
```

**ROS Type:** `Service`

**Data Type:** `audio/PcmPlay.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rosservice call media_play "file_path: '/path/to/xxx.wav'"
```

**Example:**

```python
#!/usr/bin/env python3
"""
ROS Service Test Script - Media Play Service
"""
import rospy
import sys
from audio.srv import MediaPlay, MediaPlayRequest

def main():
    # Initialize ROS node
    rospy.init_node('media_play_test', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for service media_play...")
    rospy.wait_for_service('media_play')
    
    try:
        # Create service proxy
        service_proxy = rospy.ServiceProxy('media_play', MediaPlay)
        
        # Create request
        req = MediaPlayRequest()
        req.file_path = "/path/to/company_intro.mp3"  # 音频文件路径，需将文件放置在共享目录下
        req.delete_after_play = False  # 播放后是否删除文件
        
        rospy.loginfo(f"Playing audio file: {req.file_path}")
        
        # Call service
        response = service_proxy(req)
        
        rospy.loginfo(f"Response: success={response.success}, message={response.message}")
        print(f"\nService call successful!")
        print(f"Playback status: {response.message}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## speaker/get_devices_list

**Description**

```tex
用于获取系统中已识别的所有扬声器设备信息，包括设备ID、名称、型号等。返回结果为设备列表数组，每个设备包含唯一ID，用于后续选择播放设备。若未识别到扬声器设备，返回空列表，需检查硬件连接或驱动
```

**ROS Type:** `Service`

**Data Type:** `audio/GetDeviceList.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rosservice call speaker/get_devices_list "{}"
```

**Example:**

```python
#!/usr/bin/env python3
"""
ROS Service Test Script - Get Speaker Devices List
"""
import rospy
import sys
from audio.srv import GetDeviceList, GetDeviceListRequest

def main():
    # Initialize ROS node
    rospy.init_node('get_speaker_devices_test', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for service speaker/get_devices_list...")
    rospy.wait_for_service('speaker/get_devices_list')
    
    try:
        # Create service proxy
        service_proxy = rospy.ServiceProxy('speaker/get_devices_list', GetDeviceList)
        
        # Create empty request (this service has no request parameters)
        req = GetDeviceListRequest()
        
        rospy.loginfo("Calling service to get speaker devices list...")
        
        # Call service
        response = service_proxy(req)
        
        rospy.loginfo(f"Response: success={response.success}")
        print(f"\nService call successful!")
        print(f"Available speaker devices: {len(response.devicelist)}")
        for idx, device in enumerate(response.devicelist):
            print(f"  {idx + 1}. {device}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## speaker/select_device

**Description**

```tex
用于指定当前生效的扬声器设备，通过设备ID选择对应的播放设备。设备ID需从"获取扬声器设备列表"接口的返回结果中获取。设置成功后，后续所有音频播放操作均通过该设备输出。若指定设备ID不存在，返回设置失败，保持当前生效设备不变
```

**ROS Type:** `Service`

**Data Type:** `audio/SetDevice.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rosservice call speaker/select_device "name: 'YUNJI31993Ultra'"
```

**Example:**

```python
#!/usr/bin/env python3
"""
ROS Service Test Script - Select Speaker Device
"""
import rospy
import sys
from audio.srv import SetDevice, SetDeviceRequest

def main():
    # Initialize ROS node
    rospy.init_node('select_speaker_device_test', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for service speaker/select_device...")
    rospy.wait_for_service('speaker/select_device')
    
    try:
        # Create service proxy
        service_proxy = rospy.ServiceProxy('speaker/select_device', SetDevice)
        
        # Create request
        req = SetDeviceRequest()
        req.name = "YUNJI31993Ultra"  # 设备名称，需要先通过get_devices_list获取
        
        rospy.loginfo(f"Selecting speaker device: {req.name}")
        
        # Call service
        response = service_proxy(req)
        
        rospy.loginfo(f"Response: success={response.success}, message={response.message}")
        print(f"\nService call successful!")
        print(f"Device selection status: {response.message}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## speaker/get_volume

**Description**

```tex
用于查询当前生效扬声器的音量大小，返回值为0-100的整数。音量值范围为0（静音）-100（最大音量），默认初始音量为50。若未设置生效扬声器，返回默认设备的音量值
```

**ROS Type:** `Service`

**Data Type:** `audio/GetVolume.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rosservice call speaker/get_volume "{}"
```

**Example:**

```python
#!/usr/bin/env python3
"""
ROS Service Test Script - Get Speaker Volume
"""
import rospy
import sys
from audio.srv import GetVolume, GetVolumeRequest

def main():
    # Initialize ROS node
    rospy.init_node('get_volume_test', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for service speaker/get_volume...")
    rospy.wait_for_service('speaker/get_volume')
    
    try:
        # Create service proxy
        service_proxy = rospy.ServiceProxy('speaker/get_volume', GetVolume)
        
        # Create empty request (this service has no request parameters)
        req = GetVolumeRequest()
        
        rospy.loginfo("Getting current speaker volume...")
        
        # Call service
        response = service_proxy(req)
        
        rospy.loginfo(f"Response: volume={response.volume}%")
        print(f"\nService call successful!")
        print(f"Current volume: {response.volume}%")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## speaker/set_volume

**Description**

```tex
用于调整当前生效扬声器的音量，支持手动输入0-100的音量值。音量值超出0-100范围时，自动截断为最近的有效数值（如输入110则设为100）。设置成功后即时生效，后续播放的音频均使用新音量。支持通过该接口设置静音（输入音量值0）
```

**ROS Type:** `Service`

**Data Type:** `audio/SetVolume.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rosservice call speaker/set_volume "volume: 50"
```

**Example:**

```python
#!/usr/bin/env python3
"""
ROS Service Test Script - Set Speaker Volume
"""
import rospy
import sys
from audio.srv import SetVolume, SetVolumeRequest

def main():
    # Initialize ROS node
    rospy.init_node('set_volume_test', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for service speaker/set_volume...")
    rospy.wait_for_service('speaker/set_volume')
    
    try:
        # Create service proxy
        service_proxy = rospy.ServiceProxy('speaker/set_volume', SetVolume)
        
        # Create request
        req = SetVolumeRequest()
        req.volume = 50  # 音量大小，范围0-100
        
        rospy.loginfo(f"Setting speaker volume to: {req.volume}%")
        
        # Call service
        response = service_proxy(req)
        
        rospy.loginfo(f"Response: success={response.success}, message={response.message}")
        print(f"\nService call successful!")
        print(f"Volume set status: {response.message}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## microphone/get_devices_list

**Description**

```tex
用于获取系统中已识别的所有麦克风设备信息，包括设备ID、名称、采样率等。返回结果包含每个设备的唯一ID和支持的采样率，供后续选择和配置使用。未识别到麦克风设备时返回空列表，需检查硬件连接或驱动安装
```

**ROS Type:** `Service`

**Data Type:** `audio/GetDeviceList.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rosservice call microphone/get_devices_list "{}"
```

**Example:**

```python
#!/usr/bin/env python3
"""
ROS Service Test Script - Get Microphone Devices List
"""
import rospy
import sys
from audio.srv import GetDeviceList, GetDeviceListRequest

def main():
    # Initialize ROS node
    rospy.init_node('get_microphone_devices_test', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for service microphone/get_devices_list...")
    rospy.wait_for_service('microphone/get_devices_list')
    
    try:
        # Create service proxy
        service_proxy = rospy.ServiceProxy('microphone/get_devices_list', GetDeviceList)
        
        # Create empty request (this service has no request parameters)
        req = GetDeviceListRequest()
        
        rospy.loginfo("Calling service to get microphone devices list...")
        
        # Call service
        response = service_proxy(req)
        
        rospy.loginfo(f"Response: success={response.success}")
        print(f"\nService call successful!")
        print(f"Available microphone devices: {len(response.devicelist)}")
        for idx, device in enumerate(response.devicelist):
            print(f"  {idx + 1}. {device}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## microphone/select_device

**Description**

```tex
用于指定当前用于音频采集的麦克风设备，通过设备ID进行选择。设备ID需从"获取麦克风设备列表"接口的返回结果中获取。设置成功后，后续音频采集、语音识别等操作均通过该设备进行。若指定设备不存在，返回设置失败，保持当前生效设备不变
```

**ROS Type:** `Service`

**Data Type:** `audio/SetDevice.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rosservice call microphone/select_device "name: 'YUNJI31993Ultra'"
```

**Example:**

```python
#!/usr/bin/env python3
"""
ROS Service Test Script - Select Microphone Device
"""
import rospy
import sys
from audio.srv import SetDevice, SetDeviceRequest

def main():
    # Initialize ROS node
    rospy.init_node('select_microphone_device_test', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for service microphone/select_device...")
    rospy.wait_for_service('microphone/select_device')
    
    try:
        # Create service proxy
        service_proxy = rospy.ServiceProxy('microphone/select_device', SetDevice)
        
        # Create request
        req = SetDeviceRequest()
        req.name = "YUNJI31993Ultra"  # 设备名称，需要先通过get_devices_list获取
        
        rospy.loginfo(f"Selecting microphone device: {req.name}")
        
        # Call service
        response = service_proxy(req)
        
        rospy.loginfo(f"Response: success={response.success}, message={response.message}")
        print(f"\nService call successful!")
        print(f"Device selection status: {response.message}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## microphone/audio_data

**Description**

```tex
用于实时发布唤醒后的麦克风音频数据流，供语音识别、音频存储等上层应用使用。音频流格式为PCM，采样率默认16000Hz，位深16bit，单声道。仅在麦克风被唤醒后开始发布数据，未唤醒时无消息输出。数据用于后续语音转文字（ASR）或音频分析处理
```

**ROS Type:** `Topic`

**Data Type:** `audio/AudioData.msg`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rostopic echo microphone/audio_data
```

**Example:**

```python
#!/usr/bin/env python3
"""
ROS Topic Subscriber - Audio Data
"""
import rospy
from audio.msg import AudioData

def audio_data_callback(msg):
    """
    处理音频数据流
    msg.channel: 声道数
    msg.vad_status: VAD状态（0=无语音，>0=有语音）
    msg.is_final: 是否是最终一帧
    msg.audio_data: 音频数据（uint8数组）
    """
    if msg.vad_status > 0:
        rospy.loginfo(f"Voice detected: channel={msg.channel}, is_final={msg.is_final}, data_size={len(msg.audio_data)}")
    
def main():
    # Initialize ROS node
    rospy.init_node('audio_data_subscriber', anonymous=True)
    
    # Subscribe to audio data topic
    rospy.Subscriber('microphone/audio_data', AudioData, audio_data_callback)
    
    rospy.loginfo("Listening to audio data stream...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## listen

**Description**

```tex
唤醒倾听服务为订阅型接口，接收外部唤醒指令，触发麦克风开始音频采集和倾听。订阅该话题后，接收"唤醒"指令（trigger: true）时，麦克风启动采集；接收"停止"指令（trigger: false）时，麦克风停止采集。指令需携带唤醒关键词标识，用于区分不同唤醒场景
```

**ROS Type:** `Topic`

**Data Type:** `std_msgs/Bool.msg`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rostopic pub listen std_msgs/Bool "data: true"
```

**Example:**

```python
#!/usr/bin/env python3
"""
ROS Topic Publisher - Wake Up Control
"""
import rospy
from std_msgs.msg import Bool

def main():
    # Initialize ROS node
    rospy.init_node('wake_up_control', anonymous=True)
    
    # Create publisher
    pub = rospy.Publisher('listen', Bool, queue_size=10)
    
    # Wait for subscribers
    rospy.sleep(1.0)
    
    # Send wake up command
    msg = Bool()
    msg.data = True  # True=唤醒，False=休眠
    
    rospy.loginfo(f"Sending wake up command: {msg.data}")
    pub.publish(msg)
    
    rospy.loginfo("Wake up command sent successfully!")
    rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## listen_state

**Description**

```tex
实时发布麦克风的唤醒倾听状态，告知上层应用当前是否处于音频采集状态。状态包括"未唤醒""唤醒中""倾听超时"三种，便于上层应用判断是否进行后续交互。倾听超时时间默认30秒，超时后自动切换为"未唤醒"状态
```

**ROS Type:** `Topic`

**Data Type:** `std_msgs/Bool.msg`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rostopic echo listen_state
```

**Example:**

```python
#!/usr/bin/env python3
"""
ROS Topic Subscriber - Listen State
"""
import rospy
from std_msgs.msg import Bool

def listen_state_callback(msg):
    """
    处理倾听状态
    msg.data: True=正在倾听，False=未倾听
    """
    if msg.data:
        rospy.loginfo("Robot is listening...")
    else:
        rospy.loginfo("Robot is not listening")

def main():
    # Initialize ROS node
    rospy.init_node('listen_state_subscriber', anonymous=True)
    
    # Subscribe to listen state topic
    rospy.Subscriber('listen_state', Bool, listen_state_callback)
    
    rospy.loginfo("Monitoring listen state...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## asr_text

**Description**

```tex
实时将麦克风采集的音频流转换为文本，通过话题发布转换结果，支持连续语音识别。识别语言默认中文，支持识别标点符号，准确率与语音清晰度、环境噪音相关。连续语音识别时，每100ms发布一次当前识别结果，句子结束后发布最终完整文本。未识别到有效语音时，发布空文本字符串
```

**ROS Type:** `Topic`

**Data Type:** `std_msgs/String.msg`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rostopic echo asr_text
```

**Example:**

```python
#!/usr/bin/env python3
"""
ROS Topic Subscriber - ASR Text
"""
import rospy
from std_msgs.msg import String

def asr_text_callback(msg):
    """
    处理语音识别结果
    msg.data: 识别到的文本
    """
    if msg.data:
        rospy.loginfo(f"Recognized text: {msg.data}")
    else:
        rospy.loginfo("No speech detected")

def main():
    # Initialize ROS node
    rospy.init_node('asr_text_subscriber', anonymous=True)
    
    # Subscribe to ASR text topic
    rospy.Subscriber('asr_text', String, asr_text_callback)
    
    rospy.loginfo("Listening to ASR text output...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

## version

**Description**

```tex
用于查询音频模块的当前软件版本号，包括基础功能版本和各子服务版本信息。返回结果包含主版本号、子版本号、修订号，以及TTS、ASR、LLM等子服务的版本信息。版本号格式遵循语义化版本规范（X.Y.Z）
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger.srv`

**Version:**

- 1.0.0 : added

**CLI:**

```shell
rosservice call version "{}"
```

**Example:**

```python
#!/usr/bin/env python3
"""
ROS Service Test Script - Audio Module Version
"""
import rospy
import sys
from std_srvs.srv import Trigger, TriggerRequest

def main():
    # Initialize ROS node
    rospy.init_node('audio_version_test', anonymous=True)
    
    # Wait for service
    rospy.loginfo("Waiting for service version...")
    rospy.wait_for_service('version')
    
    try:
        # Create service proxy
        service_proxy = rospy.ServiceProxy('version', Trigger)
        
        # Create empty request (Trigger service has no request parameters)
        req = TriggerRequest()
        
        rospy.loginfo("Calling service to get audio module version...")
        
        # Call service
        response = service_proxy(req)
        
        rospy.loginfo(f"Response: success={response.success}")
        print(f"\nService call successful!")
        print(f"Audio Module Version: {response.message}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```
