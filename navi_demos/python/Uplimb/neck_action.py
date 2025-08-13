#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def neck_control_client():
    rospy.init_node('neck_control_client')

    client = actionlib.SimpleActionClient('/neck_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    rospy.loginfo("Waiting for action server to start...")
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("Action server not available!")
        return

    rospy.loginfo("Action server started, sending goal.")

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ["Neck_Y", "Neck_Z"]

    # 轨迹点定义（严格递增时间）
    points = []
    # 第一次点头 + 左倾
    points.append(create_point([0.0, 0.0], 0.0))
    points.append(create_point([0.2, 0.3], 0.5))
    points.append(create_point([0.0, 0.0], 1.0))
    # 第二次点头 + 右倾
    points.append(create_point([0.2, -0.3], 1.5))  # 直接从1.0s跳到1.5s
    points.append(create_point([0.0, 0.0], 2.0))

    goal.trajectory.points = points

    client.send_goal(goal)
    finished_before_timeout = client.wait_for_result(rospy.Duration(3.0))

    if finished_before_timeout:
        state = client.get_state()
        rospy.loginfo(f"Action finished with state: {state}")
    else:
        rospy.loginfo("Action did not finish before the timeout.")

def create_point(positions, time):
    point = JointTrajectoryPoint()
    point.positions = positions
    point.velocities = [0.0, 0.0]
    point.accelerations = [0.0, 0.0]
    point.time_from_start = rospy.Duration(time)
    return point

if __name__ == '__main__':
    try:
        neck_control_client()
    except rospy.ROSInterruptException:
        pass



