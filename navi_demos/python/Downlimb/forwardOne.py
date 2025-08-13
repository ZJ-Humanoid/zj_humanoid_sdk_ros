#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 无自启下，GMP控制器模式->行走指定距离和方向
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import sys

class AutoController:
    def __init__(self, direction, distance):
        rospy.init_node('auto_controller', anonymous=True)
        
        # 初始化发布者
        self.gmp_pub = rospy.Publisher('/load_gmp_controller', Float32, queue_size=1)
        self.cmd_web_vel_pub = rospy.Publisher('/cmd_web_vel', Twist, queue_size=1)
        
        self.distance = distance
        self.direction = direction
        
    def run(self):
        # 创建消息
        float_msg = Float32()
        float_msg.data = 2.0  # 与joy.yaml中的scale值保持一致
        
        # 自动进入GMP控制器模式
        self.gmp_pub.publish(float_msg)
        rospy.sleep(2.0)  # 等待2秒让机器人准备就绪
        
        # 控制机器人前进或后退指定距离
        cmd_web_vel = Twist()
        if self.direction == 'forward':
            cmd_web_vel.linear.x = 0.5  # 使用与joy.yaml中相同的速度比例
        elif self.direction == 'back':
            cmd_web_vel.linear.x = -0.5  # 负值表示后退
        else:
            print("Invalid direction. Please use 'forward' or 'back'.")
            return
        
        cmd_web_vel.linear.y = 0.0
        cmd_web_vel.angular.z = 0.0
        
        start_time = rospy.get_time()
        traveled_distance = 0.0
        rate = rospy.Rate(10)  # 10Hz的控制频率
        
        print(f"开始{self.direction} {self.distance}米...")
        
        while abs(traveled_distance) < self.distance and not rospy.is_shutdown():
            self.cmd_web_vel_pub.publish(cmd_web_vel)
            traveled_distance = cmd_web_vel.linear.x * (rospy.get_time() - start_time)
            rate.sleep()
            
        # 停止机器人
        cmd_web_vel.linear.x = 0.0
        self.cmd_web_vel_pub.publish(cmd_web_vel)
        
        print(f"机器人已完成{self.distance}米 {self.direction} 任务并停止")

if __name__ == '__main__':
    try:
        args = rospy.myargv(argv=sys.argv)
        if len(args) < 2 or len(args) > 3:
            print("Usage: rosrun navi_demos test.py <direction> [<distance>]")
            print("<direction> can be 'forward' or 'back'")
            print("If <distance> is not provided, default is 1.5 meters.")
            sys.exit(1)
        
        direction = args[1].lower()
        if len(args) == 3:
            distance = float(args[2])
        else:
            distance = 1.5  # 默认距离为1.5米
        
        controller = AutoController(direction, distance)
        controller.run()
    except rospy.ROSInterruptException:
        pass



