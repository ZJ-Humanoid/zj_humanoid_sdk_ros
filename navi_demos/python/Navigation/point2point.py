#!/usr/bin/env python3

"""
point2point.py - 该demo实现机器人的点到点导航
"""

import rospy
import time
from navi_types.msg import TaskInfo, MetaData, Waypoint
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header

def create_task_info_msg():
    """创建任务信息消息"""
    task_msg = TaskInfo()
    
    # 设置消息头部
    task_msg.header = Header()
    task_msg.header.seq = 0
    task_msg.header.stamp = rospy.Time.now()
    task_msg.header.frame_id = ""  # 参考坐标系
    
    # 设置元数据
    task_msg.meta_data = MetaData()
    task_msg.meta_data.start_stamp_ms = int(time.time() * 1000)  # 当前时间(毫秒)
    task_msg.meta_data.end_stamp_ms = 0  # 结束时间将在任务完成后设置
    task_msg.meta_data.duration_ms = 0   # 持续时间将在任务完成后计算
    
    # 创建从当前点到点(5,5)的简单路径
    x = 5.0
    y = 5.0
        
    # 创建路径点
    waypoint = Waypoint()
    waypoint.id = 1
    waypoint.action = 0
    waypoint.audio = 0 
    
    # 设置位姿
    waypoint.pose = Pose()
    waypoint.pose.position = Point(x=x, y=y, z=0.0)
    
    # 设置朝向 (简单地朝向目标点)
    waypoint.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    
    # 添加到路径
    task_msg.waypoints.append(waypoint)
    
    return task_msg

def publish_task_info():
    """发布任务信息话题"""
    # 初始化ROS节点
    rospy.init_node('task_info_publisher', anonymous=True)
    
    # 创建发布者
    pub = rospy.Publisher('/task_info', TaskInfo, queue_size=10)
    
    # 设置发布频率
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        # 创建任务信息消息
        task_msg = create_task_info_msg()
        
        # 更新时间戳
        current_time_ms = int(time.time() * 1000)
        task_msg.header.stamp = rospy.Time.now()
        task_msg.meta_data.start_stamp_ms = current_time_ms
        
        # 发布消息
        pub.publish(task_msg)
        rospy.loginfo(f"Published task with {len(task_msg.waypoints)} waypoints")
        
        # 按照设定频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_task_info()
    except rospy.ROSInterruptException:
        pass