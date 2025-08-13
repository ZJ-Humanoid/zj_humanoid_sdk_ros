#include <ros/ros.h>
#include "navi_types/Hand_Joint.h"

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "service_joint_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<navi_types::Hand_Joint>("/robotHandJointSwitch");
    navi_types::Hand_Joint srv;
    
    // 定义6关节姿态
    std::vector<float> q_(6);
    q_[0] = 0;
    q_[1] = 0;
    q_[2] = 1.1;
    q_[3] = 1.1;
    q_[4] = 1.1;
    q_[5] = 0;
    srv.request.q = q_;

    //选择左右手： 0是左手；1是右手
    srv.request.id = 1;

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
