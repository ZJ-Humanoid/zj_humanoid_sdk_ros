#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from navi_types.srv import Audio_MediaPlay
def main(audio_file:str):
    """
    测试媒体播放服务
    """ 
    # 初始化 ROS 节点
    rospy.init_node('mediaplay_test_client')
    rospy.loginfo("Media play client已启动")
    rospy.wait_for_service('media_play_service')
    
    try:
        # 创建服务代理
        media_play_client = rospy.ServiceProxy('media_play_service', Audio_MediaPlay)
        response = media_play_client(audio_file)
        rospy.loginfo(f"Media play status: {response.status}")

        if response.status == 200:
            rospy.loginfo(f"Media play请求成功")
        else:
            rospy.logerr(f"Media play请求失败") 

    except rospy.ServiceException as e:
        rospy.logerr(f"Media play service call failed: {e}")


if __name__ == "__main__":
    try:
        audio_file = "/tmp/8d8b30de-99b1-46f5-9e69-2ee4c895bf1c.wav"
        main(audio_file)
    except rospy.ROSInterruptException:
        rospy.loginfo("Media play test client已关闭")
   
   
