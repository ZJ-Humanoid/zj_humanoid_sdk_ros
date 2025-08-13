#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def left_arm_control_client():
    rospy.init_node('left_arm_control_client')

    # 创建动作客户端
    client = actionlib.SimpleActionClient('/left_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    rospy.loginfo("等待左臂控制器服务器启动...")
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("左臂控制器服务器未响应！")
        return

    rospy.loginfo("左臂控制器服务器已启动，发送目标轨迹...")

    # 创建目标消息
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        "Shoulder_Y_L", "Shoulder_X_L", "Shoulder_Z_L",
        "Elbow_L", "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L"
    ]

    # 定义轨迹点（符合关节限位）
    points = []

    # 初始位置（根据用户提供的初始值）
    initial_positions = [0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0]  # Shoulder_X_L = 0.2
    point1 = JointTrajectoryPoint()
    point1.positions = initial_positions
    point1.velocities = [0.0] * 7
    point1.accelerations = [0.0] * 7
    point1.time_from_start = rospy.Duration(0.0)
    points.append(point1)

    # 轨迹点2: 移动到中间位置（符合限位）
    point2 = JointTrajectoryPoint()
    point2.positions = [
        0.5,   # Shoulder_Y_L (限位: -3.0~1.1) ✅
        1.0,   # Shoulder_X_L (限位: 0.0~3.0) ✅
        1.0,   # Shoulder_Z_L (限位: -2.8~2.8) ✅
        -1.0,  # Elbow_L (限位: -2.0~0.0) ✅
        1.5,   # Wrist_Z_L (限位: -2.8~2.8) ✅
        0.5,   # Wrist_Y_L (限位: -1.4~1.4) ✅
        0.5    # Wrist_X_L (限位: -1.4~1.4) ✅
    ]
    point2.velocities = [0.0] * 7
    point2.accelerations = [0.0] * 7
    point2.time_from_start = rospy.Duration(1.5)
    points.append(point2)

    # 轨迹点3: 返回初始位置
    point3 = JointTrajectoryPoint()
    point3.positions = initial_positions
    point3.velocities = [0.0] * 7
    point3.accelerations = [0.0] * 7
    point3.time_from_start = rospy.Duration(3.0)
    points.append(point3)

    # 将轨迹点添加到目标中
    goal.trajectory.points = points

    # 发送目标并等待结果
    client.send_goal(goal)
    finished_before_timeout = client.wait_for_result(rospy.Duration(4.0))

    if finished_before_timeout:
        state = client.get_state()
        rospy.loginfo(f"左臂动作完成，状态: {state}")
    else:
        rospy.loginfo("左臂动作超时未完成！")

if __name__ == '__main__':
    try:
        left_arm_control_client()
    except rospy.ROSInterruptException:
        pass