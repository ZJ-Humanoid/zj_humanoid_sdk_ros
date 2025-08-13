#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 无自启下，站立->GMP控制器模式->行走2米
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time

class AutoController:
    def __init__(self):
        rospy.init_node('auto_controller', anonymous=True)
        
        # 初始化发布者
        self.stand_pub = rospy.Publisher('/set_stand', Float32, queue_size=1)
        self.gmp_pub = rospy.Publisher('/load_gmp_controller', Float32, queue_size=1)
        self.cmd_web_vel_pub = rospy.Publisher('/cmd_web_vel', Twist, queue_size=1)
        
    def run(self):
        # 创建消息
        float_msg = Float32()
        float_msg.data = 2.0  # 与joy.yaml中的scale值保持一致
        
        # 等待用户输入进入站立模式
        input("按回车键进入站立模式...")    # Python 3.x
        self.stand_pub.publish(float_msg)
        rospy.sleep(2.0)  # 等待2秒让机器人完成站立
        
        # 等待用户输入进入GMP控制器模式
        input("按回车键进入GMP控制器模式...")    # Python 3.x
        self.gmp_pub.publish(float_msg)
        rospy.sleep(2.0)  # 等待2秒让机器人准备就绪
        
        # 控制机器人前进2米
        cmd_web_vel = Twist()
        cmd_web_vel.linear.x = 0.45  # 使用与joy.yaml中相同的速度比例
        cmd_web_vel.linear.y = 0.0
        cmd_web_vel.angular.z = 0.0
        
        start_time = rospy.get_time()
        distance = 0.0
        rate = rospy.Rate(10)  # 10Hz的控制频率
        
        print("开始前进2米...")
        
        # 假设以0.45m/s的速度，需要大约4.44秒走完2米
        while distance < 2.0 and not rospy.is_shutdown():
            self.cmd_web_vel_pub.publish(cmd_web_vel)
            distance = 0.45 * (rospy.get_time() - start_time)
            rate.sleep()
            
        # 停止机器人
        cmd_web_vel.linear.x = 0.0
        self.cmd_web_vel_pub.publish(cmd_web_vel)
        
        print("机器人已完成2米行走任务并停止")

if __name__ == '__main__':
    try:
        controller = AutoController()
        controller.run()
    except rospy.ROSInterruptException:
        pass