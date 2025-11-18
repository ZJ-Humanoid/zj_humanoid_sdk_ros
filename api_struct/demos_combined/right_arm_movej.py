#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Service Test Script

Service: /zj_humanoid/upperlimb/movej/right_arm
Type: upperlimb/MoveJ
Description: 右臂movej运动

Usage:
    python3 right_arm_movej.py right_arm_movej.yaml

This script calls the service with request data from a YAML file.
"""

import rospy
import yaml
import sys
from std_srvs.srv import Trigger, TriggerRequest
from upperlimb.srv import MoveJ, MoveJRequest


def load_yaml_data(yaml_file):
    """Load service request data from YAML file."""
    try:
        with open(yaml_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception as e:
        rospy.logerr(f"Failed to load YAML file: {e}")
        sys.exit(1)


def move_right_arm(yaml_file):
    """
    控制机器人右臂的ROS1 Python函数。
    1. 调用 go_down 服务复位。
    2. 调用 movej 服务执行关节运动。
    """
    rospy.init_node('right_arm_controller_client', anonymous=True)
    rospy.loginfo("右臂运动控制器客户端已启动...")
    
    # 加载 YAML 配置文件
    data = load_yaml_data(yaml_file)
    if 'joints' not in data or 'v' not in data or 'acc' not in data or 't' not in data or 'arm_type' not in data or 'is_async' not in data:
        rospy.logerr("YAML data is missing required keys: 'joints', 'v', 'acc', 't', 'arm_type', or 'is_async'.")
        sys.exit(1)

    go_down_service_name = '/zj_humanoid/upperlimb/go_down/right_arm'
    movej_service_name = '/zj_humanoid/upperlimb/movej/right_arm'

    try:
        rospy.loginfo(f"等待服务: {go_down_service_name} 就绪...")
        rospy.wait_for_service(go_down_service_name, timeout=10.0)
        go_down_client = rospy.ServiceProxy(go_down_service_name, Trigger)
        rospy.loginfo(f"服务 {go_down_service_name} 已就绪.")
        
        go_down_req = TriggerRequest()
        rospy.loginfo("正在发送 Go Down (回初始点) 请求...")
        go_down_resp = go_down_client(go_down_req)

        if go_down_resp.success:
            rospy.loginfo(f"Go Down 成功: {go_down_resp.message}")
            rospy.sleep(3.0) # 暂停一段时间，等待机械臂运动完成
        else:
            rospy.logerr(f"Go Down 失败: {go_down_resp.message}")
            return # 如果复位失败，则中止后续运动
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Go Down 服务调用失败: {e}")
        return
    except rospy.ROSException as e:
        rospy.logerr(f"等待服务 {go_down_service_name} 超时或出现其他ROS异常: {e}")
        return

    try:
        rospy.loginfo(f"等待服务: {movej_service_name} 就绪...")
        rospy.wait_for_service(movej_service_name, timeout=10.0)
        movej_client = rospy.ServiceProxy(movej_service_name, MoveJ)
        rospy.loginfo(f"服务 {movej_service_name} 已就绪.")
        
        movej_req = MoveJRequest()
        
        movej_req.joints = [float(j) for j in data['joints']]
        movej_req.v = float(data['v'])
        movej_req.acc = float(data['acc'])
        movej_req.t = float(data['t'])
        movej_req.is_async = bool(data['is_async'])
        movej_req.arm_type = int(data['arm_type'])

        rospy.loginfo(f"正在发送 MoveJ 运动请求，目标关节值: {movej_req.joints}")
        rospy.loginfo(f"参数 - v: {movej_req.v}, acc: {movej_req.acc}, t: {movej_req.t}, arm_type: {movej_req.arm_type}")
        movej_resp = movej_client(movej_req)

        if movej_resp.success:
            rospy.loginfo(f"MoveJ 运动成功完成: {movej_resp.message}")
        else:
            rospy.logerr(f"MoveJ 运动执行失败: {movej_resp.message}")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"MoveJ 服务调用失败: {e}")
    except rospy.ROSException as e:
        rospy.logerr(f"等待服务 {movej_service_name} 超时或出现其他ROS异常: {e}")
        
    rospy.loginfo("程序执行完毕。")


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <request_data.yaml>")
        sys.exit(1)
    
    yaml_file = sys.argv[1]
    
    try:
        move_right_arm(yaml_file)
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS 程序中断.")
    except Exception as e:
        rospy.logerr(f"发生未预期的错误: {e}")
        