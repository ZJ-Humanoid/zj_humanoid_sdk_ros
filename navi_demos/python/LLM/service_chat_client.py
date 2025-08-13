#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from navbrain_ros.srv import LLM, LLMRequest
import sys

def main():
    """
    测试 LLM 服务
    """
    # 初始化 ROS 节点
    rospy.init_node('llm_service_test_client', anonymous=True)
    rospy.loginfo("LLM 服务测试客户端已启动")
    
    # 等待 LLM 服务可用
    rospy.loginfo("等待 LLM 服务...")
    rospy.wait_for_service('llm_service')
    
    try:
        # 创建服务代理
        llm_proxy = rospy.ServiceProxy('llm_service', LLM)
        
        # 获取输入文本
        if len(sys.argv) > 1:
            input_text = ' '.join(sys.argv[1:])
        else:
            input_text = "你好，请问今天天气怎么样？"
        
        # 创建请求
        enable_context = True  # 是否启用上下文
        enable_save = True    # 是否保存对话
        context_id = "test_conversation"  # 对话上下文ID
        
        request = LLMRequest(
            raw_input=input_text,
            enable_context=enable_context,
            enable_save=enable_save,
            context_id=context_id
        )
        
        # 调用服务
        rospy.loginfo(f"发送 LLM 请求: '{input_text}'")
        response = llm_proxy(request)
        
        # 处理响应
        if response.status==200:
            rospy.loginfo("LLM 请求成功")
            rospy.loginfo(f"AI 回复: {response.response}")
        else:
            rospy.logerr("LLM 请求失败")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"调用 LLM 服务失败: {e}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass 