#!/usr/bin/env python3
import math
import numpy as np
import cv2
import torch
#import depthai as dai
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# 导入控制颈部运动的服务类型
from navi_types.srv import Uplimb_MoveJ, Uplimb_MoveJRequest

# ----------------------- 全局参数 -----------------------
FRAME_SKIP = 1
PIXEL_TOLERANCE = 10
Kp = 0.5

# 全局变量
bridge = CvBridge()
frame_counter = 0
current_yaw = 0.0
current_pitch = 0.0
neck_service_proxy = None

# ----------------------- 环境初始化与模型加载 -----------------------
def init_torch():
    gpu = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    torch.set_grad_enabled(False)
    if torch.cuda.is_available():
        print("Using GPU for model loading")
    else:
        print("GPU not available, using CPU")
    print("Torch version:", torch.__version__)
    return gpu

def init_mediapipe():
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=False,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    return face_mesh

def load_torch_models(gpu, back_detector=True):
    models = {}
    # BlazeFace 人脸检测模型
    from blazeface import BlazeFace
    face_detector = BlazeFace(back_model=back_detector).to(gpu)
    if back_detector:
        face_detector.load_weights("blazefaceback.pth")
        face_detector.load_anchors("anchors_face_back.npy")
    else:
        face_detector.load_weights("blazeface.pth")
        face_detector.load_anchors("anchors_face.npy")
    models['face_detector'] = face_detector

    # BlazePalm 手掌检测模型
    from blazepalm import BlazePalm
    palm_detector = BlazePalm().to(gpu)
    palm_detector.load_weights("blazepalm.pth")
    palm_detector.load_anchors("anchors_palm.npy")
    palm_detector.min_score_thresh = 0.75
    models['palm_detector'] = palm_detector

    # BlazeFaceLandmark 人脸关键点回归模型
    from blazeface_landmark import BlazeFaceLandmark
    face_regressor = BlazeFaceLandmark().to(gpu)
    face_regressor.load_weights("blazeface_landmark.pth")
    models['face_regressor'] = face_regressor

    # BlazeHandLandmark 手部关键点回归模型
    from blazehand_landmark import BlazeHandLandmark
    hand_regressor = BlazeHandLandmark().to(gpu)
    hand_regressor.load_weights("blazehand_landmark.pth")
    models['hand_regressor'] = hand_regressor

    return models

# ----------------------- 辅助函数 -----------------------
def get_box(landmarks, image_width, image_height):
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

# ----------------------- 角度计算函数 -----------------------
def compute_angles(startX, startY, endX, endY, image_width, image_height):
    """
    根据人脸包围框计算人脸中心与图像中心的偏差，
    并根据偏差计算需要调整的 yaw 和 pitch（角度变化量）。
    返回：(offset_x, offset_y, new_yaw, new_pitch)
    """
    global current_yaw, current_pitch

    face_center_x = (startX + endX) / 2.0
    face_center_y = (startY + endY) / 2.0
    image_center_x = image_width / 2.0
    image_center_y = image_height / 2.0

    offset_x = face_center_x - image_center_x
    offset_y = face_center_y - image_center_y

    # 若偏差均在容忍范围内，则不做调整
    if abs(offset_x) < PIXEL_TOLERANCE and abs(offset_y) < PIXEL_TOLERANCE:
        rospy.loginfo("x 和 y 方向均在容忍范围内，无需调整")
        return offset_x, offset_y, current_yaw, current_pitch

    pixel_angle_x = 0.001
    pixel_angle_y = 0.001

    # 处理 yaw（x 方向）
    if abs(offset_x) >= PIXEL_TOLERANCE:
        target_yaw = offset_x * pixel_angle_x
        if current_yaw * target_yaw < 0:
            rospy.loginfo("yaw 方向偏差，重新调整")
            current_yaw = target_yaw
        else:
            current_yaw += Kp * (target_yaw - current_yaw)
    else:
        rospy.loginfo("x 方向偏差在容忍范围内，保持 yaw")
    # 处理 pitch（y 方向）
    if abs(offset_y) >= PIXEL_TOLERANCE:
        target_pitch = -offset_y * pixel_angle_y
        if current_pitch * target_pitch < 0:
            rospy.loginfo("pitch 方向偏差，重新调整")
            current_pitch = target_pitch
        else:
            current_pitch += Kp * (target_pitch - current_pitch)
    else:
        rospy.loginfo("y 方向偏差在容忍范围内，保持 pitch")

    rospy.loginfo("计算得到偏差：offset_x=%.2f, offset_y=%.2f; 更新后角度：yaw=%.4f, pitch=%.4f",
                  offset_x, offset_y, current_yaw, current_pitch)
    return offset_x, offset_y, current_yaw, current_pitch

# ----------------------- 运动请求函数 -----------------------
def send_motion_request(neck_service_proxy, yaw, pitch):
    """
    根据传入的 yaw 和 pitch 发送颈部运动服务请求。
    """
    req = Uplimb_MoveJRequest()
    req.jnt_angle = [-pitch, -yaw]
    req.not_wait = False
    try:
        res = neck_service_proxy.call(req)
        rospy.loginfo("颈部运动服务返回：%s", res.finish)
    except Exception as e:
        rospy.logerr("调用颈部运动服务失败：%s", e)

# ----------------------- 图像展示函数 -----------------------
def display_image(cv_image, startX, startY, endX, endY, face_center, image_center, offset, yaw, pitch):
    """
    在图像上绘制人脸包围框、中心点、偏差信息以及当前颈部角度。
    """
    cv2.rectangle(cv_image, (startX, startY), (endX, endY), (0, 255, 0), 2)
    cv2.circle(cv_image, (int(face_center[0]), int(face_center[1])), 5, (255, 0, 0), -1)
    cv2.circle(cv_image, (int(image_center[0]), int(image_center[1])), 5, (0, 0, 255), -1)
    cv2.putText(cv_image, "Offset: (%.2f, %.2f)" % (offset[0], offset[1]), (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(cv_image, "Neck: pitch=%.4f, yaw=%.4f" % (pitch, yaw), (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imshow("Face Detection", cv_image)
    cv2.waitKey(1)

# ----------------------- 图像回调函数 -----------------------
def image_callback(msg, face_mesh, models, gpu):
    global frame_counter, neck_service_proxy

    try:
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
    rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb_image)

    if results.multi_face_landmarks:
        landmarks = results.multi_face_landmarks[0]
        startX, startY, endX, endY = get_box(landmarks, image_width, image_height)
        face_center = ((startX + endX) / 2.0, (startY + endY) / 2.0)
        image_center = (image_width / 2.0, image_height / 2.0)

        # 计算角度并更新全局 yaw、pitch
        offset_x, offset_y, new_yaw, new_pitch = compute_angles(startX, startY, endX, endY, image_width, image_height)
        # 发送运动请求
        send_motion_request(neck_service_proxy, new_yaw, new_pitch)
        # 在图像上绘制信息
        display_image(cv_image, startX, startY, endX, endY, face_center, image_center, (offset_x, offset_y), new_yaw, new_pitch)
    else:
        rospy.loginfo("当前未检测到人脸")
        cv2.putText(cv_image, "No face detected", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow("Face Detection", cv_image)
        cv2.waitKey(1)

# ----------------------- 主函数 -----------------------
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

    # 初始化环境和模型
    gpu = init_torch()
    face_mesh = init_mediapipe()
    models = load_torch_models(gpu, back_detector=True)

    rospy.Subscriber("/img/CAM_A/image_raw", Image,
                     lambda msg: image_callback(msg, face_mesh, models, gpu), queue_size=1)
    rospy.loginfo("订阅 /img/CAM_A/image_raw")
    rospy.Subscriber("/joint_states", rospy.AnyMsg)
    rospy.loginfo("订阅 /joint_states")
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

