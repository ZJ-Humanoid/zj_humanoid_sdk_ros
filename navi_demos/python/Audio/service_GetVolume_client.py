#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

import rospy
from navi_types.srv import Audio_GetVolume

def main():
    """
    测试获取音量服务
    """
    # 初始化 ROS 节点
    rospy.init_node('get_volume_test_client')
    rospy.loginfo("Get volume test client已启动")
    rospy.wait_for_service('Audio/get_volume')
    
    try:
        # 创建服务代理
        get_volume_client = rospy.ServiceProxy('Audio/get_volume', Audio_GetVolume)
        response = get_volume_client()
        print(response.status,'音量:',response.volume)
        if response.status == 200:
            rospy.loginfo(f"Get volume请求成功")
        else:
            rospy.logerr(f"Get volume请求失败")

    except rospy.ServiceException as e:
        rospy.logerr(f"Get volume请求失败: {e}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Get volume test client已关闭")
