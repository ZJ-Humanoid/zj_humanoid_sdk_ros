#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

void sendLeftArmGoal(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_left_arm)
{
  // 创建目标消息
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = {
    "Shoulder_Y_L",
    "Shoulder_X_L",
    "Shoulder_Z_L",
    "Elbow_L",
    "Wrist_Z_L",
    "Wrist_Y_L",
    "Wrist_X_L"
  };

  // 添加轨迹点
  trajectory_msgs::JointTrajectoryPoint point1;
  point1.positions = {0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0};  // 目标位置
  point1.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 目标速度
  point1.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 目标加速度
  point1.time_from_start = ros::Duration(0.0); // 到达时间

  trajectory_msgs::JointTrajectoryPoint point2;
  point2.positions = {0.5, 0.5, 0.5, -1.0, 0.0, 0.0, 0.0};  // 目标位置
  point2.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 目标速度
  point2.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 目标加速度
  point2.time_from_start = ros::Duration(3.0); // 到达时间

  trajectory_msgs::JointTrajectoryPoint point3;
  point3.positions = {0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0};  // 目标位置
  point3.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 目标速度
  point3.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 目标加速度
  point3.time_from_start = ros::Duration(6.0); // 到达时间

  trajectory_msgs::JointTrajectoryPoint point4;
  point4.positions = {0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0};  // 目标位置
  point4.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 目标速度
  point4.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 目标加速度
  point4.time_from_start = ros::Duration(9.0); // 到达时间

  goal.trajectory.points.push_back(point1);
  goal.trajectory.points.push_back(point2);
  goal.trajectory.points.push_back(point3);
  goal.trajectory.points.push_back(point4);


  // 发送目标
  ac_left_arm.sendGoal(goal);
}

void sendWaistGoal(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac_waist)
{
  // 创建目标消息
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = {"A_Waist"};

  // 添加轨迹点
  trajectory_msgs::JointTrajectoryPoint point1;
  point1.positions = {0.0};  // 目标位置
  point1.velocities = {0.0}; // 目标速度
  point1.accelerations = {0.0}; // 目标加速度
  point1.time_from_start = ros::Duration(0.0); // 到达时间

  trajectory_msgs::JointTrajectoryPoint point2;
  point2.positions = {-0.5};  // 目标位置
  point2.velocities = {0.0}; // 目标速度
  point2.accelerations = {0.0}; // 目标加速度
  point2.time_from_start = ros::Duration(3.0); // 到达时间

  trajectory_msgs::JointTrajectoryPoint point3;
  point3.positions = {0.0};  // 目标位置
  point3.velocities = {0.0}; // 目标速度
  point3.accelerations = {0.0}; // 目标加速度
  point3.time_from_start = ros::Duration(6.0); // 到达时间

  goal.trajectory.points.push_back(point1);
  goal.trajectory.points.push_back(point2);
  goal.trajectory.points.push_back(point3);

  // 发送目标
  ac_waist.sendGoal(goal);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "combined_control_client");
  ros::NodeHandle nh;

  // 创建一个简单的动作客户端
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_waist("/waist_controller/follow_joint_trajectory", true);
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_left_arm("/left_arm_controller/follow_joint_trajectory", true);

  ROS_INFO("Waiting for action servers to start.");
  ac_waist.waitForServer();  // 等待服务器启动
  ac_left_arm.waitForServer();  // 等待服务器启动

  ROS_INFO("Action servers started, sending goals.");

  // 发送腰部关节的目标
  sendWaistGoal(ac_waist);

  // 发送左臂关节的目标
  sendLeftArmGoal(ac_left_arm);

  // 等待结果
  bool finished_before_timeout_waist = ac_waist.waitForResult(ros::Duration(7.0));
  bool finished_before_timeout_left_arm = ac_left_arm.waitForResult(ros::Duration(15.0));

  if (finished_before_timeout_waist)
  {
    actionlib::SimpleClientGoalState state = ac_waist.getState();
    ROS_INFO("Waist Action finished: %s", state.toString().c_str());
  }
  else
  {
    ROS_INFO("Waist Action did not finish before the time out.");
  }

  if (finished_before_timeout_left_arm)
  {
    actionlib::SimpleClientGoalState state = ac_left_arm.getState();
    ROS_INFO("Left Arm Action finished: %s", state.toString().c_str());
  }
  else
  {
    ROS_INFO("Left Arm Action did not finish before the time out.");
  }

  return 0;
}



