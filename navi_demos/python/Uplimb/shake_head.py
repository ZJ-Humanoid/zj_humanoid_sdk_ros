#!/usr/bin/env python3

import rospy
from navi_types.msg import Uplimb_ServoJ
from sensor_msgs.msg import JointState
import math

# 定义脖子初始状态
neck_y_initial = 0.0
initialized = False

# --- 参数定义区: 定义平滑摇头动作 ---
# 1. 定义摇头的最大幅度 (从中心到一侧)
SHAKE_AMPLITUDE = 0.50

# 2. 定义完成整个摇头动作所需的时间 (秒)
SHAKE_DURATION = 1.0  # 动作越快，这个值越小

# 3. 控制循环频率
time_ctrl = 0.02 # 20ms
# ---------------------------------------------

# --- 状态追踪变量 ---
motion_started = False
start_time = 0.0
# --------------------------------

pub = rospy.Publisher('/uplimb_servoJ_cmd', Uplimb_ServoJ, queue_size=10)

def joint_state_callback(msg):
    global neck_y_initial, initialized, motion_started, start_time

    joint_names = msg.name
    joint_positions = msg.position

    if 'Neck_Y' in joint_names:
        if not initialized:
            # 捕获Y轴初始位置，以保持头部不会上下晃动
            neck_y_index = joint_names.index('Neck_Y')
            neck_y_initial = joint_positions[neck_y_index]
            rospy.loginfo("Neck initial Y position captured: %.3f", neck_y_initial)
            
            # 标记初始化完成，并准备开始动作
            initialized = True
            motion_started = True
            start_time = rospy.get_time()
            rospy.loginfo(f"Smooth shake sequence started. Duration: {SHAKE_DURATION}s, Amplitude: {SHAKE_AMPLITUDE}rad.")


def control_neck_position(event):
    global neck_y_initial, motion_started, start_time

    # 如果节点正在关闭或动作尚未开始，则不执行任何操作
    if rospy.is_shutdown() or not motion_started:
        return
    
    current_time = rospy.get_time()
    time_elapsed = current_time - start_time

    # --- 核心逻辑: 平滑的正弦运动 ---
    if time_elapsed <= SHAKE_DURATION:
        # 计算角频率 (omega)，确保在SHAKE_DURATION内完成一个2*pi的周期
        omega = (2 * math.pi) / SHAKE_DURATION
        
        # 使用sin函数计算当前Z轴位置
        neck_z_new = SHAKE_AMPLITUDE * math.sin(omega * time_elapsed)
        neck_y_new = neck_y_initial # Y轴保持稳定
    else:
        # 动作时间到，准备退出
        rospy.loginfo("Smooth shake finished. Setting final position to 0.0 and shutting down.")
        
        # 明确地将最终位置设置为0.0，并发布一次
        neck_z_new = 0.0
        neck_y_new = neck_y_initial
        
        # 发布最终位置指令
        cmd = Uplimb_ServoJ()
        cmd.neck = [neck_z_new, neck_y_new]
        cmd.neckValid = True
        cmd.achieve_time = 0.1
        pub.publish(cmd)
        
        # 等待一小段时间确保指令发出
        rospy.sleep(0.1) 
        
        # 发出关闭信号
        motion_started = False # 停止循环
        rospy.signal_shutdown("Sequence complete")
        return

    # --- 发布指令 ---
    cmd = Uplimb_ServoJ()
    cmd.neck = [neck_z_new, neck_y_new]
    cmd.neckValid = True
    cmd.achieve_time = time_ctrl # 持续发送，平滑过渡

    # 如果需要，也可以控制腰部
    # cmd.waist = [neck_z_new, neck_y_new, 0]
    # cmd.waistValid = True

    pub.publish(cmd)

def joint_status_listener():
    rospy.init_node('neck_smooth_shake_controller', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, joint_state_callback, queue_size=1)
    rospy.Timer(rospy.Duration(time_ctrl), control_neck_position)
    
    rospy.loginfo("Neck smooth shake controller started.")
    rospy.spin()
    rospy.loginfo("Node has been shut down. Exiting program.")

if __name__ == '__main__':
    try:
        joint_status_listener()
    except rospy.ROSInterruptException:
        pass