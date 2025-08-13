#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
import math
import numpy as np

# 导入控制颈部运动的服务类型
from navi_types.srv import Uplimb_MoveJ, Uplimb_MoveJRequest

# ------------------ 参数设置 ------------------
# 已知 RGB 摄像机水平 FOV，单位：度
HORIZONTAL_FOV_DEG = 68.7938
FRAME_SKIP = 8
frame_counter = 0
# ------------------ 辅助函数 ------------------
def get_box(landmarks, image_width, image_height):
    """
    根据 mediapipe 检测到的面部 landmarks 计算人脸包围框
    返回 (x_min, y_min, x_max, y_max)
    """
    pts = []
    for lm in landmarks.landmark:
        x_px = min(math.floor(lm.x * image_width), image_width - 1)
        y_px = min(math.floor(lm.y * image_height), image_height - 1)
        pts.append((x_px, y_px))
    pts = np.array(pts)
    x_min = int(np.min(pts[:, 0]))
    y_min = int(np.min(pts[:, 1]))
    x_max = int(np.max(pts[:, 0]))
    y_max = int(np.max(pts[:, 1]))
    return x_min, y_min, x_max, y_max

# ------------------ 全局初始化 ------------------
# cv_bridge 实例，用于 ROS 图像消息与 OpenCV 图像之间转换
bridge = CvBridge()

# 初始化 mediapipe 面部检测（face mesh），只检测一张脸
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(
    max_num_faces=1,
    refine_landmarks=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# 声明全局变量保存颈部运动服务的代理句柄
neck_service_proxy = None

# ------------------ 图像回调函数 ------------------
def image_callback(msg):
    global neck_service_proxy,frame_counter
    try:
        # 将 ROS Image 消息转换为 OpenCV 格式（BGR）
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image = cv_image.copy()
    except CvBridgeError as e:
        rospy.logerr("CvBridge 转换错误: %s", e)
        return

    frame_counter+= 1
    if frame_counter % FRAME_SKIP != 0:
        cv2.imshow("Face Detection",cv_image)
        cv2.waitKey(1)
        return


    # 获取图像尺寸（ROS 消息内包含分辨率信息）
    image_height = msg.height
    image_width  = msg.width

    # 将 BGR 转换为 RGB（mediapipe 要求 RGB 图像）
    rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    # 使用 mediapipe 进行面部检测
    results = face_mesh.process(rgb_image)

    if results.multi_face_landmarks:
        # 取第一张检测到的人脸
        landmarks = results.multi_face_landmarks[0]
        # 获取人脸包围框
        startX, startY, endX, endY = get_box(landmarks, image_width, image_height)
        # 计算人脸框中心点
        face_center_x = (startX + endX) / 2.0
        face_center_y = (startY + endY) / 2.0

        # 图像中心点（初始点）
        image_center_x = image_width / 2.0
        image_center_y = image_height / 2.0

        # 计算人脸中心与图像中心的像素偏差
        offset_x = face_center_x - image_center_x  # 水平方向偏差（正：向右）
        offset_y = face_center_y - image_center_y  # 垂直方向偏差（正：向下）

        # 将 FOV 从度转换为弧度（水平 FOV）
        horizontal_fov_rad = math.radians(HORIZONTAL_FOV_DEG)
        # 根据图像长宽比计算垂直 FOV：公式 vertical_fov = 2 * arctan(tan(horizontal_fov/2) * (height/width))
        vertical_fov_rad = 2 * math.atan(math.tan(horizontal_fov_rad / 2) * (float(image_height) / image_width))

        # 计算每个像素对应的弧度值
        pixel_angle_x = horizontal_fov_rad / image_width
        pixel_angle_y = vertical_fov_rad / image_height

        # 计算机器人需要转动的角度
        # 假设：x 方向偏差（左右）对应颈部 yaw；y 方向偏差（上下）对应颈部 pitch
        # 注意：由于图像 y 坐标向下增加，当人脸位于图像下方时，偏差为正，
        # 但通常机器人需要向下转（pitch 负值），因此这里取负值处理
        robot_yaw = offset_x * pixel_angle_x
        robot_pitch = - offset_y * pixel_angle_y

        rospy.loginfo("人脸中心：(%.2f, %.2f)  图像中心：(%.2f, %.2f)", face_center_x, face_center_y, image_center_x, image_center_y)
        rospy.loginfo("像素偏差：(%.2f, %.2f)", offset_x, offset_y)
        rospy.loginfo("每像素弧度：(%.6f, %.6f)", pixel_angle_x, pixel_angle_y)
        rospy.loginfo("计算出的颈部角度调整：pitch = %.4f rad, yaw = %.4f rad", robot_pitch, robot_yaw)

        # 绘制人脸包围框
        cv2.rectangle(cv_image, (startX, startY), (endX, endY), (0, 255, 0), 2)
        # 绘制人脸中心点和图像中心点
        cv2.circle(cv_image, (int(face_center_x), int(face_center_y)), 5, (255, 0, 0), -1)
        cv2.circle(cv_image, (int(image_center_x), int(image_center_y)), 5, (0, 0, 255), -1)

        # 在图像上显示人脸中心与图像中心的偏差信息
        text_offset = "Offset: (%.2f, %.2f)" % (offset_x, offset_y)
        cv2.putText(cv_image, text_offset, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # 在图像上显示机器人颈部运动的坐标值
        text_neck = "Neck: pitch=%.4f, yaw=%.4f" % (robot_pitch, robot_yaw)
        cv2.putText(cv_image, text_neck, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # 构造并调用控制机器人颈部运动的服务请求
        req = Uplimb_MoveJRequest()
        # 按服务定义，这里传入两个角度数值；具体含义（pitch/yaw）需参考实际机器人控制要求
        req.jnt_angle = [0.0, -0.0]
        # req.jnt_angle = [0 ,robot_pitch]
        print("------------------",robot_pitch)
        req.not_wait = False
        try:
            res = neck_service_proxy.call(req)
            rospy.loginfo("颈部运动服务返回：%s", res.finish)
        except Exception as e:
            rospy.logerr("调用颈部运动服务失败：%s", e)
    else:
        rospy.loginfo("当前未检测到人脸")
        # 当未检测到人脸时，在图像上添加提示文字
        cv2.putText(cv_image, "No face detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        #显示处理后的图像
        cv2.imshow("Face Detection", cv_image)
        cv2.waitKey(1)

# ------------------ 主函数 ------------------
def main():
    global neck_service_proxy
    rospy.init_node('face_neck_tracker_node', anonymous=True)

    # 等待并连接颈部运动服务（服务名与类型根据实际配置）
    rospy.loginfo("等待服务 neck_movej_service ...")
    rospy.wait_for_service("neck_movej_service")
    try:
        neck_service_proxy = rospy.ServiceProxy("neck_movej_service", Uplimb_MoveJ)
        rospy.loginfo("成功连接到 neck_movej_service")
    except Exception as e:
        rospy.logerr("连接颈部运动服务失败：%s", e)
        return

    # 订阅图像数据 topic
    rospy.Subscriber("/img/CAM_B/image_raw", Image, image_callback, queue_size=1)
    rospy.loginfo("节点 face_neck_tracker_node 已启动，订阅 /img/CAM_B/image_raw")


    rospy.spin()
    # 程序退出时关闭所有 OpenCV 窗口
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
