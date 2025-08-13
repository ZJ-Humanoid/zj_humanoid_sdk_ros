#include <ros/ros.h>
#include "navi_types/Uplimb_MoveJ.h"
#include <vector>

// 挥手动作
int main(int argc, char **argv)
{
    ros::init(argc, argv, "right_arm_trajectory_client_node");
    ros::NodeHandle nh;

    // Wait for the service to become available
    ros::service::waitForService("/right_arm_movej_service");

    // Create a client for the service
    ros::ServiceClient client = nh.serviceClient<navi_types::Uplimb_MoveJ>("/right_arm_movej_service");

    // Define a trajectory as a series of joint positions for waving
    std::vector<std::vector<float>> trajectory = {
        {-0.9, -1.6, -0.5, -1.5, 0.0, 0.0, 0.0},  // Start position (down)
        {-0.9, -1.6, -0.5, -0.5, 0.0, 0.0, 0.0},  // Start position (down)
        {-0.9, -1.6, -0.5, -1.5, 0.0, 0.0, 0.0},  // Start position (down)
        {-0.9, -1.6, -0.5, -0.5, 0.0, 0.0, 0.0},  // Start position (down)
        
        {0.0, -0.25, 0.0, 0.0, 0.0, 0.0, 0.0}
    };

    // Iterate over each point in the trajectory and send a MoveJ request
    for (const auto& point : trajectory)
    {
        navi_types::Uplimb_MoveJ srv;
        srv.request.jnt_angle = point;
        srv.request.not_wait = false;

        // Call the service and check if it was successful
        if (client.call(srv))
        {
            ROS_INFO("Moved to position: %f, %f, %f, %f, %f, %f, %f. Finish: %d",
                     point[0], point[1], point[2], point[3], point[4], point[5], point[6],
                     srv.response.finish.data);
        }
        else
        {
            ROS_ERROR("Failed to call service right_arm_movej_service");
            return 1;
        }

        // Sleep for a short duration before sending the next request
        ros::Duration(0.01).sleep();
    }

    return 0;
}



