#include <ros/ros.h>
#include "navi_types/Uplimb_MoveJ.h"

/*
关节空间（moveJ）控制demo
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "right_arm_movej_client_node");
    ros::NodeHandle nh;

    // Wait for the service to become available
    ros::service::waitForService("/right_arm_movej_service");

    // Create a client for the service
    ros::ServiceClient client = nh.serviceClient<navi_types::Uplimb_MoveJ>("/right_arm_movej_service");

    // Create a request object
    navi_types::Uplimb_MoveJ srv1;
    srv1.request.jnt_angle = {0, -0.3, 0, -0.7, 0, 0, 0};
    srv1.request.not_wait = false;

    navi_types::Uplimb_MoveJ srv2;
    srv2.request.jnt_angle = {0, -0.3, 0, 0.6, 0, 0, 0};
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