#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

/*
颈部关节轨迹跟随demo
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "neck_control_client");
  ros::NodeHandle nh;

  // 创建一个简单的动作客户端
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/neck_controller/follow_joint_trajectory", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();  // 等待服务器启动

  ROS_INFO("Action server started, sending goal.");

  // 创建目标消息
  control_msgs::FollowJointTrajectoryGoal goal;
  // Neck_Y和Neck_Z的范围都是-10.0°~20.0°（-0.17rad~0.34rad）
  goal.trajectory.joint_names = {"Neck_Y", "Neck_Z"};

  // 添加轨迹点
  // 摇头
  trajectory_msgs::JointTrajectoryPoint point1;
  point1.positions = {0.0, 0.0};  // 目标位置
  point1.velocities = {0.0, 0.0}; // 目标速度
  point1.accelerations = {0.0, 0.0}; // 目标加速度
  point1.time_from_start = ros::Duration(0.0); // 到达时间

  trajectory_msgs::JointTrajectoryPoint point2;
  point2.positions = {0.0, 0.32};  // 目标位置
  point2.velocities = {0.0, 0.0}; // 目标速度
  point2.accelerations = {0.0, 0.0}; // 目标加速度
  point2.time_from_start = ros::Duration(2.0); // 到达时间

  trajectory_msgs::JointTrajectoryPoint point3;
  point3.positions = {0.0, -0.15};  // 目标位置
  point3.velocities = {0.0, 0.0}; // 目标速度
  point3.accelerations = {0.0, 0.0}; // 目标加速度
  point3.time_from_start = ros::Duration(4.0); // 到达时间

  trajectory_msgs::JointTrajectoryPoint point4;
  point4.positions = {0.0, 0.0};  // 目标位置
  point4.velocities = {0.0, 0.0}; // 目标速度
  point4.accelerations = {0.0, 0.0}; // 目标加速度
  point4.time_from_start = ros::Duration(6.0); // 到达时间

  // 点头
  trajectory_msgs::JointTrajectoryPoint point5;
  point5.positions = {0.2, 0.0};  // 目标位置
  point5.velocities = {0.0, 0.0}; // 目标速度
  point5.accelerations = {0.0, 0.0}; // 目标加速度
  point5.time_from_start = ros::Duration(8.0); // 到达时间

  trajectory_msgs::JointTrajectoryPoint point6;
  point6.positions = {0.0, 0.0};  // 目标位置
  point6.velocities = {0.0, 0.0}; // 目标速度
  point6.accelerations = {0.0, 0.0}; // 目标加速度
  point6.time_from_start = ros::Duration(10.0); // 到达时间

  goal.trajectory.points.push_back(point1);
  goal.trajectory.points.push_back(point2);
  goal.trajectory.points.push_back(point3);
  goal.trajectory.points.push_back(point4);
  goal.trajectory.points.push_back(point5);
  goal.trajectory.points.push_back(point6);

  // 发送目标
  ac.sendGoal(goal);

  // 等待结果
  bool finished_before_timeout = ac.waitForResult(ros::Duration(5.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
  {
    ROS_INFO("Action did not finish before the time out.");
  }

  return 0;
}



