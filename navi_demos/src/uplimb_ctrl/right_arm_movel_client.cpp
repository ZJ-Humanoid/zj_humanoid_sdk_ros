#include <ros/ros.h>
#include "navi_types/Uplimb_MoveL.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "navi_types/Uplimb_MoveJ.h"

/*
末端关节空间（moveL）控制demo
*/

bool received = false;

geometry_msgs::Pose curr_pose;

void cb(const geometry_msgs::Pose::ConstPtr& msg)
{
    curr_pose = *msg;
    received = true;
    ROS_INFO("Received current pose: x=%f, y=%f, z=%f", curr_pose.position.x, curr_pose.position.y, curr_pose.position.z);
}

void init_pose(ros::NodeHandle nh)
{
    // Wait for the service to become available
    ros::service::waitForService("/right_arm_movej_service");
    
    // Create a client for the service
    ros::ServiceClient client = nh.serviceClient<navi_types::Uplimb_MoveJ>("/right_arm_movej_service");

    // Create a request object
    navi_types::Uplimb_MoveJ srv1;
    srv1.request.jnt_angle = {0, -0.3, 0, -0.7, 0, 0, 0};
    srv1.request.not_wait = false;

    if (client.call(srv1))
    {
        ROS_INFO("Service called successfully. Finish: %d", srv1.response.finish.data);
    }
    else
    {
        ROS_ERROR("Failed to call service right_arm_movej_service");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "right_arm_movel_client_node");
    ros::NodeHandle nh;

    // 1. 获取关节状态
    ros::Subscriber joint_sub = nh.subscribe("/right_arm_tcp_pose", 1, cb);
    while (ros::ok() && !received) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    // Wait for the service to become available
    ros::service::waitForService("/right_arm_movel_service");

    // Create a client for the service
    ros::ServiceClient client = nh.serviceClient<navi_types::Uplimb_MoveL>("/right_arm_movel_service");

    // 先把机械臂移动到一个位置，避免目标不可达或者出现奇异点
    init_pose(nh);

    // Create a request object
    navi_types::Uplimb_MoveL srv1;
    geometry_msgs::Pose pose1;
    pose1.position.x = curr_pose.position.x;
    pose1.position.y = curr_pose.position.y;
    pose1.position.z = curr_pose.position.z + 0.1;  // 手臂向上移动10cm
    pose1.orientation = curr_pose.orientation;
    srv1.request.target_pose = pose1;
    srv1.request.not_wait = false;

    navi_types::Uplimb_MoveL srv2;
    geometry_msgs::Pose pose2;
    pose2.position.x = curr_pose.position.x;
    pose2.position.y = curr_pose.position.y;
    pose2.position.z = curr_pose.position.z;  // 手臂回到原来的位置
    pose2.orientation = curr_pose.orientation;
    srv2.request.target_pose = pose2;
    srv2.request.not_wait = false;

    // Call the service and check if it was successful
    if (client.call(srv1))
    {
        ROS_INFO("Service called successfully. Finish: %d", srv1.response.finish.data);
    }
    else
    {
        ROS_ERROR("Failed to call service right_arm_movej_service");
        return 1;
    }

    if (client.call(srv2))
    {
        ROS_INFO("Service called successfully. Finish: %d", srv2.response.finish.data);
    }
    else
    {
        ROS_ERROR("Failed to call service right_arm_movej_service");
        return 1;
    }
    return 0;
}