## camera_calibration

**Description**

```tex
用于执行或加载机器视觉系统中的相机标定任务，包括内参（Intrinsic）和外参（Extrinsic）标定。
在机器人工作前，对视觉传感器进行精确校准，确保图像像素坐标与世界坐标系之间的转换关系准确无误，是精确抓取和位姿估计的基础。from_folder模式通常指加载已有的标定文件，run_trajectory指机器人驱动末端工具进行特定运动，配合视觉传感器完成基于运动的标定。
```

**ROS Type:** `Service`

**Data Type:** `manipulation/CameraCalibration.srv`

**Version:**

- 1.0.0 : added

**CLI：**

```shell
rosservice call /zj_humanoid/manipulate/camera_calibration "{camera_name: 'sacrum_to_hand', purpose: 'intrinsic', mode: 'from_folder'}"
```

## execute_pick_task

**Description**

```tex
接收目标物体的标签（target_label），执行一个完整的抓取流程。
流程涵盖： 物体检测、位姿估计、运动规划、手臂控制和夹持。
```

**ROS Type:** `Service`

**Data Type:** `manipulation/ExecutePickTask.srv`

**Version:**

- 1.0.0 : added

**CLI：**

```shell
rosservice call /zj_humanoid/manipulate/execute_pick_task "target_label: 'melon_seeds'"
```

## grasp_teach_service

**Description**

```tex
用于执行抓取位姿的示教操作。机器人可能进入示教模式，记录指定手臂（which_arm）针对特定物体（object_label）的抓取姿态。这些示教数据随后可用于训练模型或作为预设抓取配置。
```

**ROS Type:** `Service`

**Data Type:** `manipulation/GraspTeach.srv`

**Version:**

- 1.0.0 : added

**CLI：**

```shell
rosservice call /zj_humanoid/manipulation/grasp_teach_service "which_arm: 'right'
object_label: 'melon_seeds'"
```

## instance_segmentation_action

**Description**

```tex
执行基于深度学习模型的实例分割任务。Goal：接收需要检测的物体标签列表（labels）。Feedback：在处理过程中周期性地返回当前已经检测到的物体信息（包括彩色图、深度图和部分 DetItem），以便客户端实时了解进度。Result：返回最终完整的分割结果列表（items），每个 DetItem 包含标签、掩码（Mask）等关键信息。
```

**ROS Type:** `Action`

**Data Type:** `manipulation/InstSeg.action`

**Version:**

- 1.0.0 : added

**Example：**

```Python
#!/usr/bin/env python3
# coding=utf-8

import rospy
import actionlib
import sys
from manipulation.msg import InstSegAction, InstSegGoal, InstSegFeedback, InstSegResult

# 全局变量，用于在回调中存储最终结果
# 注意：在更复杂的应用中，您可能会使用类来管理状态
g_final_items = []

def feedback_callback(feedback):
    """
    处理Action服务器在执行期间发送的反馈信息。
    """
    rospy.loginfo("--- 收到实例分割反馈 (Feedback) ---")
    if len(feedback.items) > 0:
        rospy.loginfo(f"当前检测到的物体数量: {len(feedback.items)}")
        # 打印第一个检测到的物体的标签和掩码信息
        first_item = feedback.items[0]
        rospy.loginfo(f"  - 物体[0] 标签: {first_item.label}")
        rospy.loginfo(f"  - 物体[0] 掩码 (Mask) 高度: {first_item.mask.height}, 宽度: {first_item.mask.width}")
    else:
        rospy.loginfo("反馈为空，尚未检测到物体。")

def done_callback(state, result):
    """
    处理Action执行完成后的最终状态和结果。
    """
    global g_final_items
    rospy.loginfo("--- 实例分割Action已完成 ---")
    rospy.loginfo(f"最终状态: {actionlib.GoalStatus.to_string(state)}")
    
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Action 成功执行。")
        rospy.loginfo(f"共找到 {len(result.items)} 个物体。")
        g_final_items = result.items
        for i, item in enumerate(result.items):
            rospy.loginfo(f"  - 最终物体[{i}] 标签: {item.label}")
            rospy.loginfo(f"  - 最终物体[{i}] 掩码 (Mask) 高度: {item.mask.height}, 宽度: {item.mask.width}")
    else:
        rospy.logwarn(f"Action 未能成功执行 (状态码: {state})")

def main_inst_seg_client():
    """
    主函数：初始化客户端、发送目标并等待结果。
    """
    rospy.init_node('test_instance_segmentation_client')

    action_name = '/zj_humanoid/manipulation/instance_segmentation_action'
    client = actionlib.SimpleActionClient(action_name, InstSegAction)

    rospy.loginfo(f"等待 {action_name} Action 服务器连接...")
    if not client.wait_for_server(timeout=rospy.Duration(10.0)):
        rospy.logerr(f"连接超时，请确保 {action_name} 服务器正在运行。")
        return

    rospy.loginfo("Action 服务器已连接。")

    # --- 1. 创建 Action 目标 (Goal) ---
    target_labels = ['melon_seeds'] 
    
    goal = InstSegGoal()
    goal.labels = target_labels
    rospy.loginfo(f"发送实例分割目标 (Goal)，搜索标签: {goal.labels}")

    # --- 2. 发送目标 ---
    # 我们同时注册 'done' 和 'feedback' 回调函数
    client.send_goal(goal, 
                    done_cb=done_callback, 
                    feedback_cb=feedback_callback)

    # --- 3. 等待 Action 完成 ---
    rospy.loginfo("等待Action执行结果...")
    
    # wait_for_result() 会阻塞，直到Action完成
    # 您可以设置一个超时时间
    if client.wait_for_result(rospy.Duration(30.0)): # 等待30秒
        rospy.loginfo("Action 客户端执行完毕。")
    else:
        rospy.logwarn("Action 执行超时，可能仍在运行。")
        client.cancel_goal() # 30秒后超时，取消目标

    # rospy.spin() # 如果您希望节点在回调后继续运行，可以使用spin()
                  # 但对于简单的'wait_for_result'，脚本会在此处结束

if __name__ == '__main__':
    try:
        main_inst_seg_client()
    except rospy.ROSInterruptException:
        print("程序被中断")
    except Exception as e:
        rospy.logerr(f"发生未捕获的异常: {e}")
```

## joint_space_trajectory_planner

**Description**

```tex
基于用户提供的目标关节状态序列（joint_states，即一系列关节空间的航点），在关节空间进行运动规划，生成一条平滑、无碰撞的关节轨迹（ros_trajectory）。适用于已知精确关节目标点、需要保证轨迹平滑性的场合，常用于回到安全位姿或在配置空间内进行精确移动。
```

**ROS Type:** `Service`

**Data Type:** `manipulation/GetTrajectory.srv`

**Version:**

- 1.0.0 : added

**CLI：**

```shell
rosservice call /zj_humanoid/manipulation/joint_space_trajectory_planner "which_arm: 'right'
joint_states:
- header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}
  name: ['A_Waist', 'Shoulder_Y_R', 'Shoulder_X_R', 'Shoulder_Z_R', 'Elbow_R', 'Wrist_Z_R', 'Wrist_Y_R', 'Wrist_X_R']
  position: [0.00019973708731413353, 0.017077520965358417, -0.1580000255489722, -0.01700561561392533, -0.10404177046457336, -0.015986956468623248, 0.017098819662688846, -0.01699351273299105]
  velocity: [0.00019973708731413353, 0.017077520965358417, -0.1580000255489722, -0.01700561561392533, -0.10404177046457336, -0.015986956468623248, 0.017098819662688846, -0.01699351273299105]
  effort: [3.4911185033263457e-16, -0.35685272343248964, -2.392117249126224, -0.0995120903377424, -0.55195909629302, -0.0024506632966402, -0.18711383751280253, -0.17090844531224972]
- header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}
  name: ['A_Waist', 'Shoulder_Y_R', 'Shoulder_X_R', 'Shoulder_Z_R', 'Elbow_R', 'Wrist_Z_R', 'Wrist_Y_R', 'Wrist_X_R']
  position: [0.0002476739882695256, -0.1762759690382154, -0.1620746621301805, -0.017029584064403025, -0.10403128426748937, -0.015771240414323984, 0.016620577882241548, -0.016993987355772785]
  velocity: [0.0002476739882695256, -0.1762759690382154, -0.1620746621301805, -0.017029584064403025, -0.10403128426748937, -0.015771240414323984, 0.016620577882241548, -0.016993987355772785]
  effort: [4.863165425916804e-16, -2.98639232844424, -2.4907215217201006, -0.09342238031822873, -1.5831045908170647, 0.005822229939730975, -0.4670283495714845, -0.1707499096604585]
"
```

## pose_estimation_service

**Description**

```tex
接收彩色图、深度图和分割出的物体列表（items），计算出目标物体在机器人坐标系中的精确三维位姿（obj_poses，位置和姿态）。 
```

**ROS Type:** `Service`

**Data Type:** `manipulation/PoseEst.srv`

**Version:**

- 1.0.0 : added

**Example：**

```Python
#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from manipulation.srv import PoseEst, PoseEstRequest, PoseEstResponse
from manipulation.msg import DetItem # 假设 'Item' 消息在 'manipulation' 包的 'msg' 目录下
import sys

def create_mock_data(bridge):
    """
    创建用于测试的模拟 Color 图像、Depth 图像和 Item 列表。
    
    在真实测试中，您应该替换这里，使用从 /camera/.. topic 订阅到的
    真实图像，或者从磁盘加载包含真实物体的 .png / .npy 文件。
    """
    rospy.loginfo("正在创建模拟 (Mock) 数据...")
    
    # --- 1. 模拟 Color 图像 (480x640, BGR8) ---
    # 创建一个黑色背景
    mock_color_cv = np.zeros((480, 640, 3), dtype=np.uint8)
    # 在图像中心绘制一个 100x100 的绿色方块，模拟一个物体
    cv2.rectangle(mock_color_cv, (270, 190), (370, 290), (0, 255, 0), -1)
    
    # --- 2. 模拟 Depth 图像 (480x640, 16UC1 - mono16) ---
    # 您的服务器代码使用 "passthrough"，这通常意味着 16UC1 (uint16)
    # 深度值通常以毫米(mm)为单位
    mock_depth_cv = np.zeros((480, 640), dtype=np.uint16)
    # 将绿色方块区域的深度设置为 500mm (0.5米)
    mock_depth_cv[190:291, 270:291] = 500 

    # --- 3. 模拟 Item (带 Mask) ---
    mock_item = DetItem()
    mock_item.label = "Eastroc_water" # melon_seeds
    
    # 创建掩码 (Mask)。根据您的服务器代码，掩码是 uint16 类型
    mask_array = np.zeros((480, 640), dtype=np.uint16)
    # 掩码区域应与彩色图中的物体精确对应
    mask_array[190:291, 270:291] = 1 # 非零值表示掩码
    
    # --- 4. 使用 CvBridge 转换 ---
    try:
        color_msg = bridge.cv2_to_imgmsg(mock_color_cv, encoding="bgr8")
        # 16-bit 单通道深度图使用 "mono16" 编码
        depth_msg = bridge.cv2_to_imgmsg(mock_depth_cv, encoding="mono16")
        mask_msg = bridge.cv2_to_imgmsg(mask_array, encoding="mono16")
        
        mock_item.mask = mask_msg
        
        return color_msg, depth_msg, [mock_item]
        
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge 转换失败: {e}")
        return None, None, None

def main_pose_est_client():
    """
    主函数：初始化客户端、准备请求、调用服务并打印响应。
    """
    rospy.init_node('test_pose_estimation_client')

    service_name = '/zj_humanoid/manipulation/pose_estimation_service'
    
    rospy.loginfo(f"等待 {service_name} Service 服务器连接...")
    try:
        # 等待服务可用，设置10秒超时
        rospy.wait_for_service(service_name, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr(f"服务 {service_name} 未在10秒内变为可用。请确保服务正在运行。")
        return

    rospy.loginfo("Service 服务器已连接。")

    try:
        pose_estimator = rospy.ServiceProxy(service_name, PoseEst)
        
        bridge = CvBridge()
        color_img, depth_img, items_list = create_mock_data(bridge)
        
        if color_img is None:
            rospy.logerr("创建模拟数据失败。")
            return

        # 填充 PoseEstRequest
        request = PoseEstRequest()
        request.color_image = color_img
        request.depth_image = depth_img
        request.items = items_list

        rospy.loginfo("准备调用位姿估计服务...")
        rospy.loginfo(f"  - 目标物体数量: {len(request.items)}")
        rospy.loginfo(f"  - 物体[0] 标签: {request.items[0].label}")

        # --- 3. 调用 Service ---
        # 这是一次阻塞调用，程序会在此处等待，直到服务返回响应
        response = pose_estimator(request)
        
        # --- 4. 处理响应数据 (Response) ---
        rospy.loginfo("--- 收到位姿估计响应 (Response) ---")
        if response.success:
            rospy.loginfo("服务执行成功！")
            rospy.loginfo(f"检测到的物体姿态 (obj_poses):")
            # 假设 response.obj_poses 是一个 geometry_msgs/PoseArray 或类似结构
            # (在您的代码中它被直接打印，所以我在此也直接打印)
            rospy.loginfo(str(response.obj_poses))
        else:
            rospy.logwarn("服务执行失败 (success=False)。")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service 调用失败: {e}")
    except Exception as e:
        rospy.logerr(f"发生未捕获的异常: {e}")

if __name__ == '__main__':
    try:
        main_pose_est_client()
    except rospy.ROSInterruptException:
        print("程序被中断")
```

## pose_space_trajectory_planner

**Description**

```tex
基于用户提供的笛卡尔空间位姿序列（waypoints），规划出末端执行器（End-Effector）在工作空间中的运动轨迹，并转换为关节轨迹（ros_trajectory）。适用于要求末端执行器沿直线或特定路径移动的场景，如插孔、拧螺丝或精确地放置物体。
```

**ROS Type:** `Service`

**Data Type:** `manipulation/MotionPlan.srv`

**Version:**

- 1.0.0 : added

**CLI：**

```shell
rosservice call /zj_humanoid/manipulation/pose_space_trajectory_planner "which_arm: 'right'
waypoints:
- position:
    x: 0.1801810664964644
    y: -0.1320977449761974
    z: 0.01364840767672533
  orientation:
    x: -0.3160102350523244
    y: -0.2673918536851587
    z: 0.6511814324582815
    w: 0.6360832256447027"
```

## scene_update

**Description**

```tex
用于动态更新机器人环境模型（Planning Scene）。加载新的障碍物（obstacle_names，如桌子、容器、夹具模型）到运动规划器的碰撞检测环境中，确保后续的运动规划是安全、无碰撞的。
```

**ROS Type:** `Service`

**Data Type:** `manipulation/SceneUpdate.srv`

**Version:**

- 1.0.0 : added

**CLI：**

```shell
rosservice call /zj_humanoid/manipulation/scene_update "obstacle_names:
- 'table'"
```

## version

**Description**

```tex
一个简单的标准服务，用于查询当前操作模块的版本号。 方便集成工作快速核实正在运行的功能模块是否与文档匹配，便于故障排查和版本管理。
```

**ROS Type:** `Service`

**Data Type:** `std_srvs/Trigger.srv`

**Version:**

- 1.0.0 : added

**CLI：**

```shell
rosservice call /zj_humanoid/manipulation/version "{}" 
```