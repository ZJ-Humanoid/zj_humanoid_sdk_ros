#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def waist_control_client():
    # 初始化ROS节点
    rospy.init_node('waist_control_client')

    # 创建动作客户端
    client = actionlib.SimpleActionClient('/waist_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    rospy.loginfo("Waiting for waist action server to start...")
    client.wait_for_server()  # 等待服务器启动

    rospy.loginfo("Waist action server started, sending goal.")

    # 创建目标消息
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["A_Waist"]  # 仅一个关节

    # 添加轨迹点
    # 轨迹点1: 初始位置
    point1 = JointTrajectoryPoint()
    point1.positions = [0.0]  # A_Waist: 0.0
    point1.velocities = [0.0]
    point1.accelerations = [0.0]
    point1.time_from_start = rospy.Duration(0.0)

    # 轨迹点2: 向右摆动 (A_Waist = 0.5)
    point2 = JointTrajectoryPoint()
    point2.positions = [0.30]
    point2.velocities = [0.0]
    point2.accelerations = [0.0]
    point2.time_from_start = rospy.Duration(1.0)

    # 轨迹点3: 回到初始位置
    point3 = JointTrajectoryPoint()
    point3.positions = [0.0]
    point3.velocities = [0.0]
    point3.accelerations = [0.0]
    point3.time_from_start = rospy.Duration(2.0)

    point4 = JointTrajectoryPoint()
    point4.positions = [-0.30]  # 回到初始位置
    point4.velocities = [0.0]
    point4.accelerations = [0.0]
    point4.time_from_start = rospy.Duration(3.0)

    # 轨迹点5: 向左摆动 (A_Waist = -0.5)
    point5 = JointTrajectoryPoint()
    point5.positions = [0.0]  # 回到初始位置
    point5.velocities = [0.0]
    point5.accelerations = [0.0]
    point5.time_from_start = rospy.Duration(5.0)    

    # 添加轨迹点到目标中
    goal.trajectory.points.append(point1)
    goal.trajectory.points.append(point2)
    goal.trajectory.points.append(point3)
    goal.trajectory.points.append(point4)
    goal.trajectory.points.append(point5)  

    # 发送目标
    client.send_goal(goal)

    # 等待结果
    finished_before_timeout = client.wait_for_result(rospy.Duration(6.0))  # 超时时间大于轨迹总时间

    if finished_before_timeout:
        state = client.get_state()
        rospy.loginfo(f"Waist action finished: {state}")
    else:
        rospy.loginfo("Waist action did not finish before timeout.")

if __name__ == '__main__':
    try:
        waist_control_client()
    except rospy.ROSInterruptException:
        pass