#!/usr/bin/env python3

import rospy
from navi_types.msg import Uplimb_ServoJ
from sensor_msgs.msg import JointState
import math

# 定义脖子初始状态
neck_z_initial = 0.0
neck_y_initial = 0.0
initialized = False

# --- 参数定义区 ---
# 1. 定义点头两次的序列: 上 -> 下 -> 上 -> 下
#    这代表了两个完整的点头动作。
TARGET_POSITIONS = [0.28, 0.0, 0.28, 0.0] 

# 2. 定义在每个位置停留的时间（秒）
STEP_DELAY = 0.5  # 在每个位置停留1.5秒

# 3. 控制循环频率
time_ctrl = 0.02 # 20ms
# ---------------------------------------------

# --- 状态追踪变量 ---
current_target_index = 0
last_move_time = 0.0
# --------------------------------

pub = rospy.Publisher('/uplimb_servoJ_cmd', Uplimb_ServoJ, queue_size=10)

def joint_state_callback(msg):
    global neck_z_initial, neck_y_initial, initialized, last_move_time

    joint_names = msg.name
    joint_positions = msg.position

    if 'Neck_Z' in joint_names and 'Neck_Y' in joint_names:
        if not initialized:
            neck_z_index = joint_names.index('Neck_Z')
            neck_z_initial = joint_positions[neck_z_index]
            rospy.loginfo("Neck initial Z position captured: %.3f", neck_z_initial)
            
            # 记录序列开始的时间，并立即发布第一个目标位置
            last_move_time = rospy.get_time()
            initialized = True
            rospy.loginfo(f"Sequence started. Moving to first position: {TARGET_POSITIONS[0]}")


def control_neck_position(event):
    global neck_z_initial, initialized
    global current_target_index, last_move_time

    # 如果节点正在关闭或未初始化，则不执行任何操作
    if rospy.is_shutdown() or not initialized:
        return
    
    current_time = rospy.get_time()

    # --- 核心逻辑: 检查是否应该进入序列的下一步 ---
    if current_time - last_move_time > STEP_DELAY:
        # 移动到下一个目标索引
        current_target_index += 1
        
        # --- 关键修改: 检查序列是否已完成 ---
        if current_target_index >= len(TARGET_POSITIONS):
            rospy.loginfo("Sequence finished. Robot at final position. Shutting down node.")
            rospy.signal_shutdown("Sequence complete") # 发出关闭信号
            return # 停止进一步操作
        # -----------------------------------------
        
        # 如果序列未完成，则重置计时器并记录下一个动作
        last_move_time = current_time
        rospy.loginfo(f"Moving to next position: {TARGET_POSITIONS[current_target_index]}")

    # --- 发布指令 ---
    # 只要节点在运行，就持续发布当前的目标位置指令
    # 这确保机器人控制器会努力到达并保持该位置
    neck_z_new = neck_z_initial  # Z轴保持稳定
    neck_y_new = TARGET_POSITIONS[current_target_index] # 从列表中获取Y轴目标

    cmd = Uplimb_ServoJ()
    cmd.neck = [neck_z_new, neck_y_new]
    cmd.neckValid = True
    cmd.achieve_time = 0.2 # 给予一个较短的时间来到达目标

    cmd.waist = [neck_z_new, neck_y_new, 0]
    # cmd.waistValid = True

    pub.publish(cmd)

def joint_status_listener():
    rospy.init_node('neck_finite_task_controller', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.Timer(rospy.Duration(time_ctrl), control_neck_position)
    
    rospy.loginfo("Neck finite task controller started. Will nod twice then exit.")
    
    # rospy.spin() 会在这里阻塞，直到节点被关闭（例如通过rospy.signal_shutdown）
    rospy.spin()
    
    rospy.loginfo("Node has been shut down. Exiting program.")

if __name__ == '__main__':
    try:
        joint_status_listener()
    except rospy.ROSInterruptException:
        # 这个异常会在rospy.spin()结束后被捕获，是正常行为
        pass