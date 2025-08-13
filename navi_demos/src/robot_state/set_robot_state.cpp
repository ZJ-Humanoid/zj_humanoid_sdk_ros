#include <ros/ros.h>
#include <navi_types/Robot_SetState.h>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_state_demo");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<navi_types::Robot_SetState>("set_robot_state");

    // 跳转序列及等待时间（秒），第一个1为当前状态，不发送
    std::vector<int> states   = {1, 2, 3, 4, 5, 9, 1, 2, 3, 4, 5};
    std::vector<int> wait_sec = {0, 30, 8, 7, 4, 7, 50, 30, 8, 7, 0}; // 1->2:30s, 9->1:6s, 其余不变

    navi_types::Robot_SetState srv;

    // 从第二个状态开始跳转，跳过第一个1
    for (size_t i = 1; i < states.size(); ++i) {
        srv.request.target_state = states[i];
        if (client.call(srv)) {
            ROS_INFO("Set to %d: %s", states[i], srv.response.message.c_str());
        } else {
            ROS_ERROR("Failed to call service set_robot_state for state %d", states[i]);
            return 1;
        }
        if (wait_sec[i] > 0) {
            ros::Duration(wait_sec[i]).sleep();
        }
    }

    return 0;
}