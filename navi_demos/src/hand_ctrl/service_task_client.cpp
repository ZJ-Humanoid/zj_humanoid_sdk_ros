#include <ros/ros.h>
#include "navi_types/Hand_Task.h"

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "service_hello_world_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<navi_types::Hand_Task>("/robotTaskSwitch");
    navi_types::Hand_Task srv;

    //可选mode：    Grasp,  Drill,  Pinch,PinchEgg
    //PinchEgg是带力控模式的, 再close状态去捏手指上的力传感器，手指会缩回
    //可选action：  Home,   Open,   Close
    srv.request.leftHand_mode = "PinchEgg";
    srv.request.leftHand_action = "Open";

    srv.request.rightHand_mode = "Drill";
    srv.request.rightHand_action = "Open";
    
    
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
