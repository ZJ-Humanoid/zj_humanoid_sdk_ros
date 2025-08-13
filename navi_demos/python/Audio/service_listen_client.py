#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from navi_types.srv import Audio_Listen, Audio_ListenRequest

def init_ros_node():
    """
    初始化 ROS 节点。
    """
    rospy.init_node("test_client")


def listen(operation: int):
    """
    调用Listen服务
    
    Args:
        operation: 操作类型
            1: 启动唤醒聆听模式
            2: 开启手动控制聆听模式
            3: 结束手动聆听
            0: 关闭唤醒聆听模式
    
    Returns:
        Audio_ListenResponse: 服务响应
    """
    rospy.wait_for_service('navbrain_ai/listen')
    start_listen_client = rospy.ServiceProxy('navbrain_ai/listen', Audio_Listen)
    request = Audio_ListenRequest(
            operation=operation
        )
    response = start_listen_client(request)
    rospy.loginfo(f"Listen操作状态: {response.status}")
    return response


def main():
    """
    主函数，用于测试listen服务
    """
    try:
        init_ros_node()
        while not rospy.is_shutdown():
            try:
                operation = int(input("请输入operation值（1-启动唤醒聆听模式，2-开启手动控制聆听模式，3-结束手动聆听，0-关闭唤醒聆听模式）："))
                if operation in [0, 1, 2, 3]:
                    response = listen(operation)
                    if response.status == 200:
                        rospy.loginfo("Listen请求成功")
                    else:
                        rospy.logerr("Listen请求失败")
                else:
                    rospy.logwarn("无效的operation值，请输入0、1、2或3")
            except ValueError:
                rospy.logerr("请输入有效的数字")
            except KeyboardInterrupt:
                rospy.loginfo("程序已退出")
                break
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
