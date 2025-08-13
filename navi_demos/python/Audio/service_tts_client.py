#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from navi_types.srv import Audio_TTS, Audio_TTSRequest
import sys


def main():
    """
    测试 TTS 服务
    """
    # 初始化 ROS 节点
    rospy.init_node('tts_service_test_client', anonymous=True)
    rospy.loginfo("TTS 服务测试客户端已启动")
    
    # 等待 TTS 服务可用
    rospy.loginfo("等待 TTS 服务...")
    rospy.wait_for_service('Audio/tts_service')
    
    try:
        # 创建服务代理
        tts_proxy = rospy.ServiceProxy('Audio/tts_service',Audio_TTS)
        
        # 获取要转换的文本
        if len(sys.argv) > 1:
            text_to_speak = ' '.join(sys.argv[1:])
        else:
            text_to_speak = "你好，这是一个测试语音合成的消息。"
        
        # 创建请求
        is_play = True  # 设置为 True 表示合成后立即播放
        request = Audio_TTSRequest(text=text_to_speak, isPlay=is_play)
        
        # 调用服务
        rospy.loginfo(f"发送 TTS 请求: '{text_to_speak}', 播放: {is_play}")
        response = tts_proxy(request)
        
        # 处理响应
        if response.status==200:
            rospy.loginfo(f"TTS 请求成功")
            rospy.loginfo(f"生成的音频文件路径: {response.file_path}")
        else:
            rospy.logerr("TTS 请求失败")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"调用 TTS 服务失败: {e}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass 