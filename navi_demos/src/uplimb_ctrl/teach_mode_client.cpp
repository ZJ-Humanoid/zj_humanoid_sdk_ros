#include <ros/ros.h>
#include "navi_types/Uplimb_TeachMode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teach_mode_client_node");
    ros::NodeHandle nh;

    // Wait for the service to become available
    ros::service::waitForService("/teach_mode_service");

    // Create a client for the service
    ros::ServiceClient client = nh.serviceClient<navi_types::Uplimb_TeachMode>("/teach_mode_service");

    // Create a request object
    navi_types::Uplimb_TeachMode srv;
    srv.request.mode = 2; // You can change this value as needed (0: right_left_waist, 1: right_left, 2: end_mode)

    // Call the service and check if it was successful
    if (client.call(srv))
    {
        ROS_INFO("Service called successfully. Success: %d", srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed to call service teach_mode_service");
        return 1;
    }

    return 0;
}