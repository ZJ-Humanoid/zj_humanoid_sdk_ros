#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import torch
import math

from blazebase import resize_pad, denormalize_detections
from blazeface import BlazeFace
from blazeface_landmark import BlazeFaceLandmark
from visualization import draw_detections, draw_landmarks, draw_roi, FACE_CONNECTIONS

# 导入控制颈部运动的服务类型
from navi_types.srv import Uplimb_MoveJ, Uplimb_MoveJRequest

# ------------------ 全局初始化 ------------------
bridge = CvBridge()
gpu = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
torch.set_grad_enabled(False)

# 摄像头水平 FOV（单位：度）
HORIZONTAL_FOV_DEG = 68.7938

# 加载 BlazeFace 检测模型（根据需要选择后端模型）
back_detector = True
face_detector = BlazeFace(back_model=back_detector).to(gpu)
if back_detector:
    face_detector.load_weights("blazefaceback.pth")
    face_detector.load_anchors("anchors_face_back.npy")
else:
    face_detector.load_weights("blazeface.pth")
    face_detector.load_anchors("anchors_face.npy")

# 加载 BlazeFaceLandmark 关键点回归模型
face_regressor = BlazeFaceLandmark().to(gpu)
face_regressor.load_weights("blazeface_landmark.pth")

# 全局变量：颈部运动服务代理
neck_service_proxy = None

# ------------------ 图像回调函数 ------------------
def image_callback(msg):
    global neck_service_proxy
    try:
        # 将 ROS Image 消息转换为 OpenCV 格式（BGR）
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image = cv_image.copy()
    except CvBridgeError as e:
        rospy.logerr("CvBridge 转换错误: %s", e)
        return

    height, width = cv_image.shape[:2]

    # 将 BGR 转换为 RGB，模型输入要求为 RGB
    rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    # 图像预处理：获得适合模型输入的图像（img1、img2）以及缩放因子与填充信息
    img1, img2, scale, pad = resize_pad(rgb_image)

    # 使用 BlazeFace 进行人脸检测（选择适合的输入图像）
    if back_detector:
        normalized_face_detections = face_detector.predict_on_image(img1)
    else:
        normalized_face_detections = face_detector.predict_on_image(img2)
    face_detections = denormalize_detections(normalized_face_detections, scale, pad)

    if face_detections is None or face_detections.shape[0] == 0:
        rospy.loginfo("未检测到人脸")
        cv2.imshow("Face Detection", cv_image)
        cv2.waitKey(1)
        return

    # 提取人脸 ROI，并利用 BlazeFaceLandmark 回归关键点
    xc, yc, scale_val, theta = face_detector.detection2roi(face_detections.cpu())
    # 注意：此处使用原始 RGB 图像，以确保 ROI 与后续绘制一致
    roi_img, affine, box = face_regressor.extract_roi(rgb_image, xc, yc, theta, scale_val)
    flags, normalized_landmarks = face_regressor(roi_img.to(gpu))
    landmarks = face_regressor.denormalize_landmarks(normalized_landmarks.cpu(), affine)

    # 绘制人脸关键点（只绘制置信度大于 0.5 的关键点）
    for i in range(len(flags)):
        if flags[i] > 0.5:
            draw_landmarks(cv_image, landmarks[i][:, :2], FACE_CONNECTIONS, size=1)
    
    # 绘制 ROI 框和检测框
    draw_roi(cv_image, box)
    draw_detections(cv_image, face_detections)

    # 计算人脸中心（假定 box 格式为 (x_min, y_min, x_max, y_max)）
    face_center_x = (box[0] + box[2]) / 2.0
    face_center_y = (box[1] + box[3]) / 2.0

    # 图像中心
    image_center_x = width / 2.0
    image_center_y = height / 2.0

    # 计算偏差（单位：像素）
    offset_x = face_center_x - image_center_x
    offset_y = face_center_y - image_center_y

    # 根据摄像头 FOV 计算每像素对应的角度（弧度）
    horizontal_fov_rad = math.radians(HORIZONTAL_FOV_DEG)
    vertical_fov_rad = 2 * math.atan(math.tan(horizontal_fov_rad / 2) * (height / width))
    pixel_angle_x = horizontal_fov_rad / width
    pixel_angle_y = vertical_fov_rad / height

    # 计算机器人颈部调整角度（水平偏差对应 yaw，垂直偏差对应 pitch，注意 pitch 取负值）
    robot_yaw = offset_x * pixel_angle_x
    robot_pitch = -offset_y * pixel_angle_y

    rospy.loginfo("人脸中心：(%.2f, %.2f)  图像中心：(%.2f, %.2f)", face_center_x, face_center_y, image_center_x, image_center_y)
    rospy.loginfo("偏差：(%.2f, %.2f)   颈部调整：pitch=%.4f rad, yaw=%.4f rad", offset_x, offset_y, robot_pitch, robot_yaw)

    # 在图像上绘制信息
    cv2.rectangle(cv_image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
    cv2.circle(cv_image, (int(face_center_x), int(face_center_y)), 5, (255, 0, 0), -1)
    cv2.circle(cv_image, (int(image_center_x), int(image_center_y)), 5, (0, 0, 255), -1)
    cv2.putText(cv_image, "Offset: (%.2f, %.2f)" % (offset_x, offset_y), (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(cv_image, "Neck: pitch=%.4f, yaw=%.4f" % (robot_pitch, robot_yaw), (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    # 构造并调用颈部运动服务请求
    req = Uplimb_MoveJRequest()
    req.jnt_angle = [robot_pitch, robot_yaw]
    req.not_wait = False
    try:
        res = neck_service_proxy.call(req)
        rospy.loginfo("颈部运动服务返回：%s", res.finish)
    except Exception as e:
        rospy.logerr("调用颈部运动服务失败：%s", e)

    cv2.imshow("Face Detection", cv_image)
    cv2.waitKey(1)

# ------------------ 主函数 ------------------
def main():
    global neck_service_proxy
    rospy.init_node('face_neck_tracker_node', anonymous=True)

    rospy.loginfo("等待服务 neck_movej_service ...")
    rospy.wait_for_service("neck_movej_service")
    try:
        neck_service_proxy = rospy.ServiceProxy("neck_movej_service", Uplimb_MoveJ)
        rospy.loginfo("成功连接到 neck_movej_service")
    except Exception as e:
        rospy.logerr("连接颈部运动服务失败：%s", e)
        return

    rospy.Subscriber("/img/CAM_A/image_raw", Image, image_callback, queue_size=1)
    rospy.loginfo("节点 face_neck_tracker_node 已启动，订阅 /img/CAM_A/image_raw")
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

