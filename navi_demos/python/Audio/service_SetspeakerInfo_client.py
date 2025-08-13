#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from navi_types.srv import Audio_SetDevice

def main(name: str):
    """
    测试设置扬声器信息服务
    """     
    # 初始化 ROS 节点
    rospy.init_node('set_speaker_info_test_client')
    rospy.loginfo("Set speaker info test client已启动")
    rospy.wait_for_service('set_speaker_info')

    try:
        # 创建服务代理
        set_speaker_client = rospy.ServiceProxy('set_speaker_info', Audio_SetDevice)
        response = set_speaker_client(name)
        print("设置扬声器",name)
        
        if response.status == 200:
            rospy.loginfo(f"Set speaker info 请求成功")
        else:
            rospy.logerr(f"Set speaker info 请求失败")


    except rospy.ServiceException as e:
        rospy.logerr(f"Set speaker info请求失败: {e}")  

if __name__ == "__main__":
    try:
        name = "KT USB Audio: - (hw:3,0)"
        main(name)
    except rospy.ROSInterruptException:
        rospy.loginfo("Set speaker info test client已关闭")
