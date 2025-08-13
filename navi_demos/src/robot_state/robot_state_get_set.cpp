#include <ros/ros.h>
#include "navi_types/Robot_SetState.h"
#include "navi_types/Robot_StateMsg.h"

bool state_received = false;
uint8_t current_state_numeric;
std::string current_state_info;
ros::ServiceClient client;

void robotStateCallback(const navi_types::Robot_StateMsg::ConstPtr& msg) {
    current_state_numeric = msg->state;
    current_state_info = msg->state_info;
    state_received = true;

    // 检查当前状态是否大于5（RUN）
    if (current_state_numeric > 5) {
        navi_types::Robot_SetState srv;
        srv.request.target_state = 4; // 设置为目标状态4（INIT）

        // 调用服务
        if (client.call(srv)) {
            if (srv.response.success) {
                ROS_INFO_STREAM("Successfully reinitialized robot state to INIT: " << srv.response.message);
            } else {
                ROS_ERROR_STREAM("Failed to reinitialize robot state: " << srv.response.message);
            }
        } else {
            ROS_ERROR("Failed to call service robot_set_state for reinitialization");
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_state_client");
    ros::NodeHandle n;
    client = n.serviceClient<navi_types::Robot_SetState>("robot_set_state");

    // 等待服务可用
    while (!client.exists()) {
        ROS_INFO("Waiting for the robot_set_state service...");
        sleep(1);
    }
    ROS_INFO("Connected to robot_set_state service");

    // 订阅robot_state话题
    ros::Subscriber sub = n.subscribe("robot_state", 1000, robotStateCallback);

    ros::Rate loop_rate(1); // 1 Hz

    while (ros::ok()) {
        std::cout << "Enter target state (0-9): ";
        int target_state;
        std::cin >> target_state;

        if (target_state >= 0 && target_state <= 9) {
            navi_types::Robot_SetState srv;
            srv.request.target_state = static_cast<uint8_t>(target_state);

            // 调用服务
            if (client.call(srv)) {
                if (srv.response.success) {
                    ROS_INFO_STREAM("Successfully set robot state to: " << srv.response.message);

                    // 等待并获取最新的机器人状态
                    state_received = false;
                    while (!state_received) {
                        ros::spinOnce();
                        loop_rate.sleep();
                    }

                    ROS_INFO_STREAM("Current robot state: " << current_state_info << " (" << static_cast<int>(current_state_numeric) << ")");
                } else {
                    ROS_ERROR_STREAM("Failed to set robot state: " << srv.response.message);
                }
            } else {
                ROS_ERROR("Failed to call service robot_set_state");
            }
        } else {
            ROS_ERROR("Invalid target state. Please enter a value between 0 and 9.");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



