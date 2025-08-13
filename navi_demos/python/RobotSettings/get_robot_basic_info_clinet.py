#!/usr/bin/env python
import rospy
from navi_types.srv import Robot_BasicInfo

def get_robot_basic_info_client():
    rospy.wait_for_service('get_robot_basic_info')
    try:
        # 创建服务代理
        get_robot_info = rospy.ServiceProxy('get_robot_basic_info', Robot_BasicInfo)

        # 调用服务并获得响应
        response = get_robot_info()

        # 打印响应内容
        if response.success:
            rospy.loginfo(f"Robot Version: {response.robot_version}")
            rospy.loginfo(f"Hardware Version: {response.hardware_version}")
            rospy.loginfo(f"Software Version: {response.software_version}")
            rospy.loginfo(f"IP Address: {response.ip_addr}")
        else:
            rospy.logwarn(f"Failed to retrieve robot info: {response.message}")
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('robot_info_client', anonymous=True)
    get_robot_basic_info_client()
