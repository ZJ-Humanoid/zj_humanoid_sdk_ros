#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from navi_types.srv import Audio_SetVolume

def main(volume: int):
    """
    测试音量设置服务
    """
    # 初始化 ROS 节点
    rospy.init_node('set_volume_test_client')
    rospy.loginfo("Set volume test client已启动")
    rospy.wait_for_service('set_volume')
    try:
        # 创建服务代理
        set_volume_client = rospy.ServiceProxy('set_volume', Audio_SetVolume)
        response = set_volume_client(volume)

        if response.status == 200:
            rospy.loginfo(f"Set volume请求成功")
        else:
            rospy.logerr(f"Set volume请求失败")

    except rospy.ServiceException as e:
        rospy.logerr(f"Set volume请求失败: {e}")

if __name__ == "__main__":
    try:
        volume = 50
        main(volume)
    except rospy.ROSInterruptException:
        rospy.loginfo("Set volume test client已关闭")
