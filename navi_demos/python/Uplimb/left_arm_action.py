#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def move_left_arm_to_joint_positions(positions):
    """
    使用 FollowJointTrajectoryAction 将左臂移动到指定关节角度。

    Args:
        positions (list of float): 7个关节的目标角度值 (单位：弧度)。
    """
    rospy.loginfo("正在初始化ROS节点...")
    # 1. 初始化ROS节点 (如果尚未初始化)
    # 在实际使用中，一个脚本通常只初始化一次节点。
    # 这里为了示例的独立性，我们把它放在函数里。
    # rospy.init_node('left_arm_trajectory_client_py', anonymous=True)

    # 2. 定义左臂的关节名称，顺序必须与服务端完全一致
    left_arm_joint_names = [
        "Shoulder_Y_L", "Shoulder_X_L", "Shoulder_Z_L", "Elbow_L",
        "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L"
    ]

    # 检查输入的位置数量是否正确
    if len(positions) != len(left_arm_joint_names):
        rospy.logerr("提供的目标位置数量 ({}) 与关节数量 ({}) 不匹配!".format(
            len(positions), len(left_arm_joint_names)
        ))
        return

    # 3. 创建一个Action客户端，连接到服务端
    #    服务端名称: /left_arm_controller/follow_joint_trajectory
    #    Action类型: FollowJointTrajectoryAction
    client = actionlib.SimpleActionClient(
        '/left_arm_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )

    # 4. 等待Action服务端启动
    rospy.loginfo("正在等待Action服务端 '/left_arm_controller/follow_joint_trajectory'...")
    if not client.wait_for_server(timeout=rospy.Duration(5.0)):
        rospy.logerr("连接Action服务端超时！请确保服务端节点已启动。")
        return
    rospy.loginfo("已连接到Action服务端。")

    # 5. 创建一个Action Goal (目标)
    goal = FollowJointTrajectoryGoal()

    # 6. 填充Goal中的轨迹信息
    # 填充关节名
    goal.trajectory.joint_names = left_arm_joint_names

    # 创建一个轨迹点
    point = JointTrajectoryPoint()
    # 填充目标位置
    point.positions = positions
    # 填充速度和加速度 (即使不用，也需要提供正确尺寸的列表，内容为0)
    point.velocities = [0.0] * len(left_arm_joint_names)
    point.accelerations = [0.0] * len(left_arm_joint_names)
    # 设定到达此目标点的时间
    point.time_from_start = rospy.Duration(3.0)  # 移动过程花费3秒

    # 将这个轨迹点添加到Goal的轨迹中
    goal.trajectory.points.append(point)
    
    # 为轨迹添加一个时间戳
    goal.trajectory.header.stamp = rospy.Time.now()

    # 7. 发送Goal到服务端，并等待结果
    rospy.loginfo("正在发送目标至服务端...")
    client.send_goal(goal)

    rospy.loginfo("等待运动完成...")
    # 等待动作完成，可以设置一个超时时间
    finished_within_time = client.wait_for_result(timeout=rospy.Duration(10.0))

    if finished_within_time:
        state = client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("动作成功完成!")
        else:
            rospy.logwarn("动作完成，但状态为: {}".format(client.get_goal_status_text()))
    else:
        rospy.logerr("动作未能在指定时间内完成。")
        client.cancel_goal() # 如果超时，最好取消目标

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('left_arm_trajectory_client_py', anonymous=True)
        
        # 定义要移动到的目标关节角度
        target_joint_positions = [-0.0, 0.5, -0.0, -0.0, -0.0, -0.0, 0.0]
        
        # 调用函数执行移动
        move_left_arm_to_joint_positions(target_joint_positions)

    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断。")