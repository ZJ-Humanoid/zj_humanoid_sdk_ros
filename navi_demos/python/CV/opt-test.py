#!/usr/bin/env python
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
FRAME_SKIP = 10
frame_counter = 0

# 允许的像素容忍值（用于判断人脸中心与图像中心偏差）
PIXEL_TOLERANCE = 5

# 控制系数，用于平滑调整（可根据实际情况调节）
Kp = 0.5

# ------------------ 全局变量初始化 ------------------
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

# 用于记录当前控制的颈部角度（假设初始状态为 0）
current_yaw = 0.0
current_pitch = 0.0

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

# ------------------ 图像回调函数 ------------------
def image_callback(msg):
    global neck_service_proxy, frame_counter, current_yaw, current_pitch
    try:
        # 将 ROS Image 消息转换为 OpenCV 格式（BGR）
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image = cv_image.copy()
    except CvBridgeError as e:
        rospy.logerr("CvBridge 转换错误: %s", e)
        return

    frame_counter += 1
    if frame_counter % FRAME_SKIP != 0:
        cv2.imshow("Face Detection", cv_image)
        cv2.waitKey(1)
        return

    image_height = msg.height
    image_width  = msg.width

    # 转换为 RGB 格式供 mediapipe 使用
    rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb_image)

    if results.multi_face_landmarks:
        # 取第一张检测到的人脸
        landmarks = results.multi_face_landmarks[0]
        startX, startY, endX, endY = get_box(landmarks, image_width, image_height)
        # 计算人脸包围框中心点
        face_center_x = (startX + endX) / 2.0
        face_center_y = (startY + endY) / 2.0
        # 图像中心点
        image_center_x = image_width / 2.0
        image_center_y = image_height / 2.0

        # 计算像素偏差
        offset_x = face_center_x - image_center_x  # 正值：人脸在右侧
        offset_y = face_center_y - image_center_y  # 正值：人脸在下方

        # 如果 x、y 两个方向的偏差均在容忍范围内，则认为达到理想状态，不再调整
        if abs(offset_x) < PIXEL_TOLERANCE and abs(offset_y) < PIXEL_TOLERANCE:
            rospy.loginfo("x 和 y 方向均在容忍范围内，当前状态理想，无需调整")
        else:
            # 将水平 FOV 从度转换为弧度，并根据图像比例计算垂直 FOV
            horizontal_fov_rad = math.radians(HORIZONTAL_FOV_DEG)
            vertical_fov_rad = 2 * math.atan(math.tan(horizontal_fov_rad / 2) * (float(image_height) / image_width))
            # 计算每个像素对应的弧度值
            pixel_angle_x = horizontal_fov_rad / image_width
            pixel_angle_y = vertical_fov_rad / image_height

            # 独立处理 x（yaw）方向
            if abs(offset_x) >= PIXEL_TOLERANCE:
                target_yaw = offset_x * pixel_angle_x
                if current_yaw * target_yaw < 0:
                    rospy.loginfo("yaw 方向检测到偏差，重新调整 yaw 角度")
                    current_yaw = target_yaw
                else:
                    current_yaw += Kp * (target_yaw - current_yaw)
            else:
                rospy.loginfo("x 方向偏差在容忍范围内，保持当前 yaw 值")

            # 独立处理 y（pitch）方向
            if abs(offset_y) >= PIXEL_TOLERANCE:
                target_pitch = -offset_y * pixel_angle_y
                if current_pitch * target_pitch < 0:
                    rospy.loginfo("pitch 方向检测到偏差，重新调整 pitch 角度")
                    current_pitch = target_pitch
                else:
                    current_pitch += Kp * (target_pitch - current_pitch)
            else:
                rospy.loginfo("y 方向偏差在容忍范围内，保持当前 pitch 值")

            rospy.loginfo("更新颈部角度：pitch=%.4f rad, yaw=%.4f rad", current_pitch, current_yaw)

            # 构造并调用颈部运动服务请求
            req = Uplimb_MoveJRequest()
            req.jnt_angle = [-current_yaw, 0]
            req.not_wait = False
            try:
                res = neck_service_proxy.call(req)
                rospy.loginfo("颈部运动服务返回：%s", res.finish)
            except Exception as e:
                rospy.logerr("调用颈部运动服务失败：%s", e)

        # 绘制人脸包围框、中心点和偏差信息
        cv2.rectangle(cv_image, (startX, startY), (endX, endY), (0, 255, 0), 2)
        cv2.circle(cv_image, (int(face_center_x), int(face_center_y)), 5, (255, 0, 0), -1)
        cv2.circle(cv_image, (int(image_center_x), int(image_center_y)), 5, (0, 0, 255), -1)
        cv2.putText(cv_image, "Offset: (%.2f, %.2f)" % (offset_x, offset_y), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(cv_image, "Neck: pitch=%.4f, yaw=%.4f" % (current_pitch, current_yaw), (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    else:
        rospy.loginfo("当前未检测到人脸")
        cv2.putText(cv_image, "No face detected", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow("Face Detection", cv_image)
    cv2.waitKey(1)

# ------------------ 主函数 ------------------
def main():
    global neck_service_proxy
    rospy.init_node('face_neck_tracker_node', anonymous=True)

    # 等待并连接颈部运动服务
    rospy.loginfo("等待服务 neck_movej_service ...")
    rospy.wait_for_service("neck_movej_service")
    try:
        neck_service_proxy = rospy.ServiceProxy("neck_movej_service", Uplimb_MoveJ)
        rospy.loginfo("成功连接到 neck_movej_service")
    except Exception as e:
        rospy.logerr("连接颈部运动服务失败：%s", e)
        return

    # 订阅图像数据 topic
    rospy.Subscriber("/img/CAM_A/image_raw", Image, image_callback, queue_size=1)
    rospy.loginfo("节点 face_neck_tracker_node 已启动，订阅 /img/CAM_A/image_raw")

    # 订阅 joint_states 话题（此处仅作为示例，如需使用需补充回调函数）
    rospy.Subscriber("/joint_states", rospy.AnyMsg)
    rospy.loginfo("节点 joint_states 已启动，订阅 /joint_states")

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
