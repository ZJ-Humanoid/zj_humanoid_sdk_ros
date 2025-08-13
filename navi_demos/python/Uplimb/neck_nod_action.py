#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def perform_neck_nod():
    """
    使用 FollowJointTrajectoryAction 控制颈部完成一次点头动作。
    新定义：向下为正，向上为负。
    """
    # 1. 定义Action服务端和关节名称
    ACTION_SERVER_NAME = '/neck_controller/follow_joint_trajectory'
    NECK_JOINT_NAMES = ["Neck_Z", "Neck_Y"] # 顺序必须与服务端一致

    # 2. 创建一个Action客户端
    client = actionlib.SimpleActionClient(
        ACTION_SERVER_NAME,
        FollowJointTrajectoryAction
    )

    # 3. 等待Action服务端启动
    rospy.loginfo("正在等待Action服务端 '{}'...".format(ACTION_SERVER_NAME))
    if not client.wait_for_server(timeout=rospy.Duration(5.0)):
        rospy.logerr("连接Action服务端超时！请确保服务端节点已启动。")
        return
    rospy.loginfo("已连接到Action服务端。")

    # 4. 定义点头动作的轨迹序列 (已按新要求修改)
    # 每个元组代表一个关键帧: ([Neck_Z, Neck_Y] 角度, 到达该点的时间)
    # Neck_Z (摇头) 保持为0，Neck_Y (点头) 变化
    nod_sequence = [
        # ([0.0, 0.0], 1.0),    # 可选：先回到初始姿态，用时1秒
        ([0.0, 0.28], 1.5),   # 1. 向下点头 (正方向), 用时1.5秒
        ([0.0, -0.1], 3.5),   # 2. 向上抬头 (负方向), 用时2秒 (3.5 - 1.5)
        ([0.0, 0.0], 5.0)     # 3. 回到中心位置, 用时1.5秒 (5.0 - 3.5)
    ]

    # 5. 创建一个Action Goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = NECK_JOINT_NAMES

    # 6. 填充轨迹点
    rospy.loginfo("正在构建点头动作轨迹...")
    for i, point_data in enumerate(nod_sequence):
        positions, time_sec = point_data
        
        point = JointTrajectoryPoint()
        point.positions = positions
        # 速度和加速度设为0，因为我们只关心关键帧的位置
        point.velocities = [0.0] * len(NECK_JOINT_NAMES)
        point.accelerations = [0.0] * len(NECK_JOINT_NAMES)
        point.time_from_start = rospy.Duration(time_sec)
        
        goal.trajectory.points.append(point)
        rospy.loginfo("添加路径点 {}: 位置 {}, 时间 {}s".format(i+1, positions, time_sec))

    # 为轨迹添加一个时间戳
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)

    # 7. 发送Goal到服务端，并等待结果
    rospy.loginfo("正在发送点头动作目标至服务端...")
    client.send_goal(goal)

    rospy.loginfo("等待运动完成...")
    # 等待动作完成，超时时间应比总运动时间长
    finished_within_time = client.wait_for_result(timeout=rospy.Duration(10.0))

    if finished_within_time:
        state = client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("点头动作成功完成!")
        else:
            rospy.logwarn("动作完成，但状态为: {}".format(client.get_goal_status_text()))
    else:
        rospy.logerr("动作未能在指定时间内完成。")
        client.cancel_goal()

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('neck_nod_client_revised_py', anonymous=True)
        
        # 调用函数执行点头动作
        perform_neck_nod()

    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断。")