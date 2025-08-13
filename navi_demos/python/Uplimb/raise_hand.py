#!/usr/bin/env python3
import rospy
from navi_types.srv import Uplimb_MoveJ, Uplimb_MoveJRequest
import time

def move_left_arm(trajectory):
    """
    控制左臂按照给定的轨迹点运动。
    """
    # 等待左臂运动规划服务可用
    rospy.wait_for_service('/left_arm_movej_service')
    
    try:
        # 创建服务的客户端
        client = rospy.ServiceProxy('/left_arm_movej_service', Uplimb_MoveJ)
        
        # 遍历轨迹中的每一个点，并发送MoveJ请求
        for point in trajectory:
            srv_request = Uplimb_MoveJRequest()
            srv_request.jnt_angle = point  # 设置目标关节角度
            srv_request.not_wait = False     # 设置为False，表示等待本次运动完成后再继续
            
            # 调用服务并检查是否成功
            response = client(srv_request)
            
            rospy.loginfo(f"Moved left arm to position: {point}. Finish: {response.finish.data}")
            
            # 在发送下一个请求前短暂休眠
            time.sleep(0.5) # 可以适当增加延时，以便观察每个动作
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to call service left_arm_movej_service: {e}")

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('left_arm_trajectory_client_node')
    
    # 定义左臂抬手动作的轨迹（一系列关节位置）
    # 这些点来自于您提供的 JointTrajectoryPoint.positions
    trajectory = [
        # point1: 初始抬手位置
        [-0.352815, 0.262106, -1.127823, -0.564352, -0.464496, -0.044641, 0.047971],
        
        # point3: 另一个姿态
        [0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0],
        
        # point4: 最终位置
        [-1.457245, 0.775834, -0.995553, -0.757325, -0.201275, -0.376784, -0.184971]
    ]
    
    # 移动左臂执行抬手动作
    move_left_arm(trajectory)
    
    rospy.loginfo("Left arm trajectory execution complete.")