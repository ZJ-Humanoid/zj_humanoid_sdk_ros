#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from navbrain_ros.srv import NLU, NLURequest
import sys

def main():
    """
    测试 NLU 服务
    """
    # 初始化 ROS 节点
    rospy.init_node('nlu_service_test_client', anonymous=True)
    rospy.loginfo("NLU 服务测试客户端已启动")
    
    # 等待 NLU 服务可用 
    rospy.loginfo("等待 NLU 服务...")
    rospy.wait_for_service('nlu_service')
    
    try:
        # 创建服务代理
        nlu_proxy = rospy.ServiceProxy('nlu_service', NLU)
        
        # 获取输入文本
        if len(sys.argv) > 1:
            input_text = ' '.join(sys.argv[1:])
        else:
            input_text = "帮我打开客厅的灯"
        
        # 创建请求
        enable_context = True  # 是否启用上下文
        enable_save = True    # 是否保存对话
        context_id = "test_conversation"  # 对话上下文ID
        
        request = NLURequest(
            raw_input=input_text,
            enable_context=enable_context,
            enable_save=enable_save,
            context_id=context_id
        )
        
        # 调用服务
        rospy.loginfo(f"发送 NLU 请求: '{input_text}'")
        response = nlu_proxy(request)
        
        # 处理响应
        if response.status==200:
            rospy.loginfo("NLU 请求成功")
            rospy.loginfo(f"解析结果: {response.nlu_result}")
        else:
            rospy.logerr("NLU 请求失败")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"调用 NLU 服务失败: {e}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass 