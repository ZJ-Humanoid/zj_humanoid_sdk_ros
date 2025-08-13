#include <ros/ros.h>
#include "navi_types/Hand_Gesture.h"

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "service_gesture_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<navi_types::Hand_Gesture>("/robotGestureSwitch");
    navi_types::Hand_Gesture srv;

    //可选动作Paper,   Wave,   One, Two, Three, Rock
    srv.request.leftHand_action = "One";
    srv.request.rightHand_action = "Two";

    
    // 等待服务启动
    // ros::service::waitForService("/robotSwitch");
    // client.waitForExistence();
    if (client.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("操作成功, %s", srv.response.message.c_str());
        }
        else
        {
            ROS_ERROR("操作失败, %s", srv.response.message.c_str());
        }
    }
    else
    {
        ROS_ERROR("操作失败, 未知错误!");
    }

    return 0;
}
