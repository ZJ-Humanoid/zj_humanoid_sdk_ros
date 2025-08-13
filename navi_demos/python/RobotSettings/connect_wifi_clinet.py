#!/usr/bin/env python
import rospy
from navi_types.srv import Robot_ConnectWifi, Robot_ConnectWifiRequest

def connect_wifi_client(wifi_name, wifi_password):
    # 等待服务可用
    rospy.wait_for_service('connect_wifi')
    try:
        # 创建服务代理
        connect_wifi = rospy.ServiceProxy('connect_wifi', Robot_ConnectWifi)
        
        req = Robot_ConnectWifiRequest()
        req.name = wifi_name
        req.password = wifi_password

        # 调用服务并获得响应
        response = connect_wifi(req)

        # 打印响应内容
        if response.success:
            rospy.loginfo(f"Successfully connected to WiFi: {wifi_name}")
        else:
            rospy.logwarn(f"Failed to connect to WiFi: {response.message}")
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node('wifi_client_node')

    # 输入WiFi名称和密码
    wifi_name = "WiFi-Example"  # WiFi名称
    wifi_password = "password123"  # WiFi密码

    # 调用连接WiFi的客户端函数
    connect_wifi_client(wifi_name, wifi_password)
