#!/usr/bin/env python3

import sys
import rospy
from navi_types.srv import Hand_Joint, Hand_JointRequest

def hand_joint_client(gesture_id, hand_id):
    # 初始化ROS节点
    rospy.init_node('service_joint_client')
    
    # 等待服务可用
    rospy.wait_for_service('/robotHandJointSwitch')
    
    try:
        # 创建服务客户端
        client = rospy.ServiceProxy('/robotHandJointSwitch', Hand_Joint)
        
        # 定义不同手势的6关节姿态
        gestures = {
            1: [0.0, 1.0, 0.0, 1.0, 1.0, 1.0],          # 手势1
            2: [0.0, 1.25, 0.0, 0.0, 1.0, 1.0],         # 剪刀
            3: [-0.1, 0.9, 1.0, 0.0, 0.0, 0.0],         # OK
            4: [1.0, 1.0, 0.0, 0.0, 0.0, 0.0],          # 四指张开
            5: [0.0, -0.5, 0.0, 0.0, 0.0, 0.0],         # 布
            6: [[0.0, 0.0, 1.5, 1.5, 1.5, 1.5], [1.0, 0.0, 1.5, 1.5, 1.5, 1.5]]  # 石头（两次动作）
        }
        
        if gesture_id not in gestures:
            rospy.logerr("Invalid gesture_id. It should be one of the following: %s", list(gestures.keys()))
            sys.exit(1)
        
        q_list = gestures[gesture_id]
        
        if isinstance(q_list[0], list):  # 如果是石头手势，需要执行两次动作
            for q_ in q_list:
                request = Hand_JointRequest()
                request.q = q_
                request.id = hand_id  # 选择左右手： 0是左手；1是右手
                
                # 调用服务
                response = client(request)
                
                if response.success:
                    rospy.loginfo("操作成功, %s", response.message)
                else:
                    rospy.logerr("操作失败, %s", response.message)
                    
                # 阻塞等待0.25秒
                rospy.sleep(0.25)
        else:
            q_ = q_list
            
            # 创建请求对象
            request = Hand_JointRequest()
            request.q = q_
            request.id = hand_id  # 选择左右手： 0是左手；1是右手
            
            # 调用服务
            response = client(request)
            
            if response.success:
                rospy.loginfo("操作成功, %s", response.message)
            else:
                rospy.logerr("操作失败, %s", response.message)
            
    except rospy.ServiceException as e:
        rospy.logerr("操作失败, 未知错误: %s", e)

if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)
    if len(args) != 3:
        rospy.logerr("Usage: rosrun navi_demos hand_control.py <gesture_id> <hand_id>")
        rospy.logerr("<gesture_id> should be one of the following: 1, 2, 3, 4, 5, 6")
        rospy.logerr("<hand_id> should be 0 (left hand) or 1 (right hand)")
        sys.exit(1)
    
    try:
        gesture_id = int(args[1])
        hand_id = int(args[2])
        
        if hand_id not in [0, 1]:
            raise ValueError("Invalid hand_id. It should be 0 (left hand) or 1 (right hand)")
        
    except ValueError as e:
        rospy.logerr(e)
        sys.exit(1)
    
    hand_joint_client(gesture_id, hand_id)



