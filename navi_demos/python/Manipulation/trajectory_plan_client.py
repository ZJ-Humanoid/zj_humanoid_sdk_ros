'''
@Description: trajectory plan client test
@Version: 1.0
'''

import sys
import argparse
import numpy as np
import rospy, rospkg
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from navi_types.srv import Manip_GetTrajectory, Manip_GetTrajectoryRequest

def done_cb_right(state, result):
    rospy.loginfo(f"运行结束. 右臂状态：{state}, 右臂结果：{result}")

def done_cb_left(state, result):
    rospy.loginfo(f"运行结束. 左臂状态：{state}, 左臂结果：{result}")

class GetTrajectoryClient():
    def __init__(self, args):
        self.joint_space_trajecotry_planner_client = rospy.ServiceProxy("/joint_space_trajectory_planner", Manip_GetTrajectory)
        self.which_arm = args.which_arm

        self.left_arm_waist_client = actionlib.SimpleActionClient('/left_arm_waist_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.right_arm_waist_client = actionlib.SimpleActionClient('/right_arm_waist_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for action server...")
        self.left_arm_waist_client.wait_for_server()
        self.right_arm_waist_client.wait_for_server()
        rospy.loginfo("Connected to action server.")

        self.joint_data_right = np.array([[0.00888430564373266, -0.33204692869276187, -0.15845542610804841, 0.01711347364107496, -0.01467480364722178, -0.01724530011870229, 0.016430728769546928, 0.016346720537179556],
                                          [0.0, 0.017, -0.158, -0.017, -0.105, 0.017, 0.017, 0.017]])
        
        self.joint_data_left = np.array([[0.008964200478658313, -0.3318312126384626, 0.15856328413519805, 0.016909741812014545, -0.01691657282040069, 0.01700561561392533, 0.016607881722830096, -0.017414799779629956],
                                         [-0.0037270940492817317,-0.6561243476016898,0.15826367850422685,-0.017089505190597265,-0.10499146039362586,-0.017029584064403025,-0.016957738040817655,0.015593850149649946]])
    
    def call_service(self):
        request = Manip_GetTrajectoryRequest()
        request.which_arm = self.which_arm
        joint_data = self.joint_data_right if self.which_arm == 'right' else self.joint_data_left
        request.joint_data = Float32MultiArray()
        request.joint_data.data = joint_data.flatten().tolist()

        response = self.joint_space_trajecotry_planner_client(request)
        rospy.loginfo(f"{self.which_arm} arm trajectory plan response: {response.ros_trajectory}")

        if response.success:
            rospy.loginfo("Trajectory planning successful!")
            return response.ros_trajectory
        else:
            rospy.logwarn("Trajectory planning failed!")
            return None

    def run_trajectory(self, ros_trajectory):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = ros_trajectory
        goal.goal_time_tolerance = rospy.Duration(10.0)

        rospy.loginfo(f"Sending trajectory to /follow_joint_trajectory for {self.which_arm} arm...")

        if self.which_arm == 'right':
            self.right_arm_waist_client.send_goal(goal, done_cb=done_cb_right)
            self.right_arm_waist_client.wait_for_result()
        else:
            self.left_arm_waist_client.send_goal(goal, done_cb=done_cb_left)
            self.left_arm_waist_client.wait_for_result()


if __name__ == "__main__":
    rospy.init_node('Get_Trajectory_Client', anonymous=True)

    parser = argparse.ArgumentParser(description="ROS node for joint_space_trajectory_planner service test")
    parser.add_argument('--which_arm', choices=['right', 'left'], help='Choose which arm to test')
    args = parser.parse_args()

    get_trajectory_client = GetTrajectoryClient(args)
    ros_trajectory = get_trajectory_client.call_service()
    get_trajectory_client.run_trajectory(ros_trajectory)


