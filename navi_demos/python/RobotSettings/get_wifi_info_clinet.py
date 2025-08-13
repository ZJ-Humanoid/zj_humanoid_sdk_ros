#!/usr/bin/env python
import rospy
from navi_types.srv import Robot_WifiInfo

def get_wifi_info_client():
    rospy.wait_for_service('get_wifi_info')
    try:
        # 创建服务代理
        get_robot_info = rospy.ServiceProxy('get_wifi_info', Robot_WifiInfo)

        # 调用服务并获得响应
        response = get_robot_info()

        # 打印响应内容
        if response.success:
            rospy.loginfo(f"Message: {response.message}")
            rospy.loginfo("Available WiFi Networks:")
            for wifi in response.wifi_name:
                rospy.loginfo(f"  - {wifi}")
        else:
            rospy.logwarn(f"Failed to retrieve robot info: {response.message}")
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('robot_info_client')
    get_wifi_info_client()
