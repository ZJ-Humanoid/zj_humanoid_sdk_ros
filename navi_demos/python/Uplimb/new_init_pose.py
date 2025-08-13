#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from navi_types.srv import Uplimb_MoveJRequest, Uplimb_MoveJ

def send_neck_goal():
    # 创建一个简单的动作客户端
    ac = actionlib.SimpleActionClient('/neck_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    
    rospy.loginfo("Waiting for neck action server to start.")
    ac.wait_for_server()  # 等待服务器启动
    
    rospy.loginfo("Neck action server started, sending goal.")
    
    # 创建目标消息
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["Neck_Y", "Neck_Z"]
    
    # 添加轨迹点
    point1 = JointTrajectoryPoint()
    point1.positions = [0.0, 0.0]  # 目标位置
    point1.velocities = [0.0, 0.0]  # 目标速度
    point1.accelerations = [0.0, 0.0]  # 目标加速度
    point1.time_from_start = rospy.Duration(0.0)  # 到达时间
    
    point2 = JointTrajectoryPoint()
    point2.positions = [0.0, 0.0]  # 目标位置
    point2.velocities = [0.0, 0.0]  # 目标速度
    point2.accelerations = [0.0, 0.0]  # 目标加速度
    point2.time_from_start = rospy.Duration(0.5)  # 到达时间
    
    goal.trajectory.points.append(point1)
    goal.trajectory.points.append(point2)
    
    # 发送目标
    ac.send_goal(goal)
    
    # 等待结果
    finished_before_timeout = ac.wait_for_result(rospy.Duration(1.0))
    
    if finished_before_timeout:
        state = ac.get_state()
        rospy.loginfo(f"Neck action finished: {state}")
    else:
        rospy.loginfo("Neck action did not finish before the time out.")

def send_waist_goal():
    # 创建一个简单的动作客户端
    ac = actionlib.SimpleActionClient('/waist_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    
    rospy.loginfo("Waiting for waist action server to start.")
    ac.wait_for_server()  # 等待服务器启动
    
    rospy.loginfo("Waist action server started, sending goal.")
    
    # 创建目标消息
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["A_Waist"]
    
    # 添加轨迹点
    point1 = JointTrajectoryPoint()
    point1.positions = [0.0]  # 目标位置
    point1.velocities = [0.0]  # 目标速度
    point1.accelerations = [0.0]  # 目标加速度
    point1.time_from_start = rospy.Duration(0.0)  # 到达时间
    
    point2 = JointTrajectoryPoint()
    point2.positions = [0.0]  # 目标位置
    point2.velocities = [0.0]  # 目标速度
    point2.accelerations = [0.0]  # 目标加速度
    point2.time_from_start = rospy.Duration(0.5)  # 到达时间
    
    goal.trajectory.points.append(point1)
    goal.trajectory.points.append(point2)
    
    # 发送目标
    ac.send_goal(goal)
    
    # 等待结果
    finished_before_timeout = ac.wait_for_result(rospy.Duration(1.0))
    
    if finished_before_timeout:
        state = ac.get_state()
        rospy.loginfo(f"Waist action finished: {state}")
    else:
        rospy.loginfo("Waist action did not finish before the time out.")

# 新的初始姿势
if __name__ == "__main__":
    rospy.init_node("new_init_pose_node")

    # 先发送腰部和颈部的目标
    send_waist_goal()
    send_neck_goal()

    client_left = rospy.ServiceProxy("/left_arm_movej_service", Uplimb_MoveJ)
    client_left.wait_for_service()

    client_right = rospy.ServiceProxy("/right_arm_movej_service", Uplimb_MoveJ)
    client_right.wait_for_service()

    # 左臂
    req_left = Uplimb_MoveJRequest()
    req_left.jnt_angle = [-0.0, 0.3, -0.0, -0.0, 
                          -0.0, -0.0, 0.0]
    req_left.not_wait = False

    # 右臂
    req_right = Uplimb_MoveJRequest()
    req_right.jnt_angle = [-0.0, -0.3, 0.0, -0.0, 
                           0.0, -0.0, -0.0]
    req_right.not_wait = False

    res_left = client_left.call(req_left)
    res_right = client_right.call(req_right)

    rospy.loginfo(f"left result:{res_left.finish}")
    rospy.loginfo(f"right result:{res_right.finish}")



