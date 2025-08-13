#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from navi_types.srv import Audio_GetDeviceList

def main():
    """
    测试获取扬声器信息服务
    """     
    # 初始化 ROS 节点
    rospy.init_node('get_speaker_info_test_client')
    rospy.loginfo("Get speaker info test client已启动")
    rospy.wait_for_service('Audio/get_speaker_info')

    try:
        # 创建服务代理
        get_speaker_client = rospy.ServiceProxy('Audio/get_speaker_info', Audio_GetDeviceList)
        response = get_speaker_client()
        print("扬声器信息:", response.devicelist)
        
        if response.status == 200:
            rospy.loginfo(f"Get speaker info 请求成功")
        else:
            rospy.logerr(f"Get speaker info 请求失败")

    except rospy.ServiceException as e:
        rospy.logerr(f"Get speaker info请求失败: {e}")
    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Get speaker info test client已关闭")
    