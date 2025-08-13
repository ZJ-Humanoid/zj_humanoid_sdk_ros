#!/usr/bin/env python3

import rospy
from navi_types.msg import Uplimb_ServoJ
from sensor_msgs.msg import JointState
import math

# 定义脖子初始状态
neck_z_initial = 0.0
neck_y_initial = 0.0
initialized = False  # 添加一个标志，标记是否已初始化
time_ctrl = 0.02 #控制周期，ms
pub = rospy.Publisher('/uplimb_servoJ_cmd', Uplimb_ServoJ, queue_size=10)

# 回调函数，用来处理接收到的 JointState 消息
def joint_state_callback(msg):
    global neck_z_initial, neck_y_initial, initialized

    # 获取消息的接收时间戳
    timestamp = msg.header.stamp
    # 获取当前系统时间（ROS时间）
    current_time = rospy.get_time()

    joint_names = msg.name
    joint_positions = msg.position

    if 'Neck_Z' in joint_names and 'Neck_Y' in joint_names:
        neck_z_index = joint_names.index('Neck_Z')
        neck_y_index = joint_names.index('Neck_Y')
        # rospy.loginfo("Neck_Z: %f,Neck_Y: %f, >at time: %.3f (ROS time:%.3f)", 
        #               joint_positions[neck_z_index], joint_positions[neck_y_index], current_time, timestamp.to_sec())
        # 只有在尚未初始化时才获取初始值
        if not initialized:
            neck_z_initial = joint_positions[neck_z_index]
            neck_y_initial = joint_positions[neck_y_index]
            
            initialized = True  # 初始化完成，设置标志为 True

# 定时器回调函数，每20ms更新脖子的角度
def control_neck_position(event):
    global neck_z_initial, neck_y_initial

    if not initialized:
        return
    
    # 获取当前时间（秒）
    t = rospy.get_time()
    
    # 计算 Neck_Y 和 Neck_Z 的新角度
    neck_z_new = neck_z_initial + 0.4 * math.cos(t)
    neck_y_new = neck_y_initial + -0.15 * math.sin(t)  # 画圈的幅度和速度可以调整

    # 创建并发布 Uplimb_ServoJ 消息
    cmd = Uplimb_ServoJ()
    cmd.neck = [neck_z_new, neck_y_new]  # 假设这两个轴是顺序的，可能需要根据实际情况调整
    cmd.neckValid = True
    cmd.achieve_time = time_ctrl

    cmd.waist = [neck_z_new,neck_y_new,0]  # 假设这两个轴是顺序的，可能需要根据实际情况调整
    # cmd.waistValid = True

    # 发布消息
    pub.publish(cmd)
    
        # 获取当前系统时间（ROS时间）
    current_time = rospy.get_time()

    # rospy.loginfo("new neck position:Neck_Z: %.3f, Neck_Y: %.3f, time: %.3f", neck_z_new,neck_y_new, current_time)

def joint_status_listener():
    # 初始化 ROS 节点
    rospy.init_node('joint_status_listener', anonymous=True)
    
    # 订阅 joint_states 主题
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    
    # 创建定时器，20ms执行一次控制函数
    rospy.Timer(rospy.Duration(time_ctrl), control_neck_position)  # 20ms = 0.02s
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    try:
        joint_status_listener()
    except rospy.ROSInterruptException:
        pass
