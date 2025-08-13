#! /usr/bin/env python3
# encoding: utf-8

from datetime import datetime
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, PoseStamped
import rospy
import tf
import copy
import random
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseArray

class traj_point:
    def __init__(self,type = None, value = None, loc_info = None, 
                 desc = None, wait_time = 0., is_record = True, check_joint_limit = 0.):
        self.type = type
        self.value = value
        self.loc_info = loc_info
        self.desc = desc
        self.wait_time = wait_time
        self.is_record = is_record
        self.check_joint_limit = check_joint_limit

def get_middle_pose(strat_pose, end_pose, k=1.5):
    middle_pose = np.eye(4)
    middle_pose[:3, :3] = end_pose[:3,:3]
    middle_pose[0,3] = (strat_pose[0,3] + end_pose[0,3])/k
    middle_pose[1,3] = (strat_pose[1,3] + end_pose[1,3])/k
    middle_pose[2,3] = (strat_pose[2,3] + end_pose[2,3])/k
    return middle_pose

def timestamp(marker: bool = False):
    template = '%Y-%m-%d %H:%M:%S' if marker else '%Y%m%d%H%M%S'
    return datetime.now().strftime(template)

def euler_to_matrix(xyz_euler):
    m = np.eye(4)
    m[:3, :3] = R.from_euler('xyz', xyz_euler[3:]).as_dcm()
    m[0, 3] = xyz_euler[0]
    m[1, 3] = xyz_euler[1]
    m[2, 3] = xyz_euler[2]

    return m

# 沿当前(TCP)位姿平移
def matrix_translation(T, x=0, y=0, z=0):
    Tt = np.eye(4)
    Tt[:3, 3] = [x, y, z]
    T_new = T.dot(Tt)
    return T_new

# 沿基坐标（SACRUM）平移
def matrix_translation_along_base(T, x=0, y=0, z=0):
    T_new = copy.deepcopy(T)
    T_new[0,3] += x
    T_new[1,3] += y
    T_new[2,3] += z
    return T_new

def ros_pose_to_matrix(ros_pose):
    position = ros_pose.position
    quat = ros_pose.orientation
    t = np.eye(4)
    t[:3,:3] = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_dcm()
    t[:3, 3] = [position.x, position.y, position.z]
    return t

def matrix_to_ros_pose(matrix):
    rotation = R.from_dcm(matrix[:3, :3]).as_quat()
    # rotation = R.from_dcm(matrix[:3, :3]).as_quat()
    position = matrix[:3, 3]
    ros_pose = Pose()
    ros_pose.position.x = position[0]
    ros_pose.position.y = position[1]
    ros_pose.position.z = position[2]
    ros_pose.orientation.x = rotation[0]
    ros_pose.orientation.y = rotation[1]
    ros_pose.orientation.z = rotation[2]
    ros_pose.orientation.w = rotation[3]
    return ros_pose

def matrix_to_ros_poseStamped(matrix):
    rotation = R.from_dcm(matrix[:3, :3]).as_quat()
    # rotation = R.from_dcm(matrix[:3, :3]).as_quat()
    position = matrix[:3, 3]
    ros_pose = PoseStamped()
    ros_pose.header.stamp = rospy.Time.now()
    ros_pose.header.frame_id = 'SACRUM' 
    ros_pose.pose.position.x = position[0]
    ros_pose.pose.position.y = position[1]
    ros_pose.pose.position.z = position[2]
    ros_pose.pose.orientation.x = rotation[0]
    ros_pose.pose.orientation.y = rotation[1]
    ros_pose.pose.orientation.z = rotation[2]
    ros_pose.pose.orientation.w = rotation[3]
    return ros_pose

def ros_pose_to_ros_poseStamped(pose):

    ros_pose = PoseStamped()
    ros_pose.header.stamp = rospy.Time.now()
    ros_pose.header.frame_id = 'SACRUM' 
    ros_pose.pose = pose

    return ros_pose

def transform_matrix_to_quaternion(transform_matrix):

    rotation_matrix = transform_matrix[:3, :3]

    rotation = R.from_dcm(rotation_matrix)

    quaternion = rotation.as_quat()
    
    return quaternion

def pose_to_transform_matrix(pose):
    # 提取位置和四元数
    x, y, z = pose[0:3]
    qx, qy, qz, qw = pose[3:7]
    
    # 将四元数转换为旋转矩阵
    rotation_matrix = np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
    ])
    
    # 构造4x4变换矩阵
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = [x, y, z]
    
    return transform_matrix

# TODO 位置更新函数
def update_grasp_pose(old_pose_A, new_pose_A, old_pose_B):

    shelf_in_SAC_old = quatertion_to_matrix(old_pose_A[:3], old_pose_A[3:])
    shelf_in_SAC_new = quatertion_to_matrix(new_pose_A[:3], new_pose_A[3:])

    bag_in_SAC_old = quatertion_to_matrix(old_pose_B[:3], old_pose_B[3:])

    shelf_in_SAC_old_inv = np.linalg.inv(np.array(shelf_in_SAC_old))

    T_AB = np.dot(shelf_in_SAC_old_inv, np.array(bag_in_SAC_old))
    # print('--------------------')
    # print(T_AB)
    new_mat_B = np.dot(shelf_in_SAC_new, T_AB)

    quat = transform_matrix_to_quaternion(new_mat_B)
    quat_all = [new_mat_B[0,3], new_mat_B[1,3], new_mat_B[2,3]] + quat.tolist()
    # print(quat_all)

    return quat_all


def quat_to_ros_poseStamped(xyz, quat):
    matrix = quatertion_to_matrix(xyz, quat)
    ros_pose = matrix_to_ros_poseStamped(matrix)
    return ros_pose

def quatertion_to_matrix(xyz, quat):
    matrix = np.eye(4)
    matrix[0, 3] = xyz[0]
    matrix[1, 3] = xyz[1]
    matrix[2, 3] = xyz[2]

    # qx, qy, qz, qw = quat
    # quat_ = [qw, qx, qy, qz]

    rotation_matrix = tf.transformations.quaternion_matrix(quat)
    matrix[:3, :3] = rotation_matrix[:3, :3] 
    return matrix

def transform_matrix_to_xyzQuat(transform_matrix):

    xyz_quat = [0, 0, 0, 0, 0, 0, 0]
    rotation_matrix = transform_matrix[:3, :3]

    rotation = R.from_dcm(rotation_matrix)

    quaternion = rotation.as_quat()
    xyz_quat[0] = transform_matrix[0, 3]
    xyz_quat[1] = transform_matrix[1, 3]
    xyz_quat[2] = transform_matrix[2, 3]

    xyz_quat[3] = quaternion[0]
    xyz_quat[4] = quaternion[1]
    xyz_quat[5] = quaternion[2]
    xyz_quat[6] = quaternion[3]

    return xyz_quat


# print(quatertion_to_matrix([0,0,0], [0,0,0.707,1]))

def get_symmetric_pose(original_pose):
    "get symmetric pose along x-z plane"
    x, y ,z, qx, qy, qz, qw = original_pose
    symmetric_pose = [x, -y, z, -qx, qy, -qz, qw]
    return symmetric_pose

def get_symmetric_joint(original_joint):
    "get symmetric joint along x-z plane"
    j0, j1, j2, j3, j4, j5, j6, j7 = original_joint
    symmetric_joint = [-j0, j1, -j2, -j3, j4, -j5, j6, -j7]
    return symmetric_joint

def transform_pose(pose_in_quat, transform_matrix):

    x, y, z = pose_in_quat[:3]
    qx, qy, qz, qw = pose_in_quat[3:]

    rotation_matrix = R.from_quat([qx, qy, qz, qw]).as_dcm()

    pose_matrix = np.eye(4)
    pose_matrix[:3, :3] = rotation_matrix
    pose_matrix[:3, 3] = [x, y, z]

    new_pose_matrix = transform_matrix @ pose_matrix

    new_position = new_pose_matrix[:3, 3]
    new_rotation = R.from_dcm(new_pose_matrix[:3, :3])
    new_quaternion = new_rotation.as_quat()

    return np.concatenate([new_position, new_quaternion])

def generate_random_pose(pose, position_error=[0.05,0.05,0.01], orientation_error=5, seed=None, scene_name=None):
    if seed is not None:
        random.seed(seed)
    
    # x_error = random.uniform(-position_error[0], position_error[0])
    # y_error = random.uniform(-position_error[1], position_error[1])
    # z_error = random.uniform(-position_error[2], position_error[2])

    # back
    # if scene_name == 'right':
    #     x_error = random.uniform(-position_error[0], 0)
    #     y_error = random.uniform(0, position_error[1])
    #     z_error = random.uniform(-position_error[2], position_error[2])
    # elif scene_name == 'left':
    #     x_error = random.uniform(-position_error[0], 0)
    #     y_error = random.uniform(-position_error[1], 0)
    #     z_error = random.uniform(-position_error[2], position_error[2])
    # elif scene_name == 'front':
    #     x_error = random.uniform(0, position_error[0])
    #     y_error = random.uniform(-0.5* position_error[1], 0.5* position_error[1])
    #     z_error = random.uniform(-position_error[2], position_error[2])
    # else:
    #     x_error = random.uniform(-position_error[0], position_error[0])
    #     y_error = random.uniform(-position_error[1], position_error[1])
    #     z_error = random.uniform(-position_error[2], position_error[2])

    # front
    if scene_name == 'right':
        x_error = random.uniform(0,position_error[0])
        y_error = random.uniform(0, position_error[1])
        z_error = random.uniform(-position_error[2], position_error[2])
    elif scene_name == 'left':
        x_error = random.uniform(0,position_error[0])
        y_error = random.uniform(-position_error[1], 0)
        z_error = random.uniform(-position_error[2], position_error[2])
    elif scene_name == 'front':
        x_error = random.uniform(0, position_error[0])
        y_error = random.uniform(-0.5* position_error[1], 0.5* position_error[1])
        z_error = random.uniform(-position_error[2], position_error[2])
    else:
        x_error = random.uniform(-position_error[0], position_error[0])
        y_error = random.uniform(-position_error[1], position_error[1])
        z_error = random.uniform(-position_error[2], position_error[2])

    
    qx_error = np.deg2rad(random.uniform(-orientation_error, orientation_error))
    qy_error = np.deg2rad(random.uniform(-orientation_error, orientation_error))
    qz_error = np.deg2rad(random.uniform(-orientation_error, orientation_error))
    
    q = np.array([pose[3], pose[4], pose[5], pose[6]])
    q_error = np.array([qx_error, qy_error, qz_error, 0]) 
    q_new = np.array([pose[3], pose[4], pose[5], pose[6]])
    q_total = np.add(q, q_error) 
    
    q_total = q_total / np.linalg.norm(q_total)
    
    pose_random = [
        pose[0] + x_error,
        pose[1] + y_error,
        pose[2] + z_error,
        q_total[0],
        q_total[1],
        q_total[2],
        q_total[3]
    ]
    
    return pose_random

def generate_scene_random_pose(ref_scene_pose, position_error=[0.04,0.04,0.01], orientation_error=0.1, seed=0):
    new_scene_pose = {'right':[], 'left':[], 'front':[]}
    new_scene_pose['right'].append(generate_random_pose(ref_scene_pose['right'][0], position_error, orientation_error, seed, 'right'))
    new_scene_pose['left'].append(generate_random_pose(ref_scene_pose['left'][0], position_error, orientation_error, seed+1, 'left'))
    new_scene_pose['front'].append(generate_random_pose(ref_scene_pose['front'][0], position_error, orientation_error, seed+2, 'front'))

    # new_scene_pose['right'].append(generate_random_pose(ref_scene_pose['right'][0], position_error, orientation_error, seed))
    # new_scene_pose['left'].append(generate_random_pose(ref_scene_pose['left'][0], position_error, orientation_error, seed+1))
    # new_scene_pose['front'].append(generate_random_pose(ref_scene_pose['front'][0], position_error, orientation_error, seed+2))
    return new_scene_pose

def euler_to_matrix(xyz_euler, mode = 'xyz'):
    m = np.eye(4)
    m[:3, :3] = R.from_euler(mode, xyz_euler[3:]).as_dcm()
    m[0, 3] = xyz_euler[0]
    m[1, 3] = xyz_euler[1]
    m[2, 3] = xyz_euler[2]

    return m

def create_dashed_cube_marker(x_lim, y_lim, z_lim):

    # x_lim = [-0.26, 0.02]
    # y_lim = [0.44, 0.54]
    # z_lim = [0.43, 0.53]

    marker = Marker()
    marker.header.frame_id = "SACRUM"  # 设置参考坐标系
    marker.header.stamp = rospy.Time.now()  # 设置时间戳
    marker.ns = "solid_cube" # solid_cube dashed_cube
    marker.id = 0  # Marker ID
    marker.type = Marker.LINE_LIST  # 使用 LINE_LIST 来画线
    marker.action = Marker.ADD  # 添加 Marker
    marker.scale.x = 0.003  # 设置线的宽度

    # 设置虚线的样式
    marker.color.r = 1.0  
    marker.color.g = 0.0  
    marker.color.b = 0.0  
    marker.color.a = 1.0  

    # 设置虚线的点
    vertices = [
        Point(x_lim[0], y_lim[0], z_lim[0]),  # 0
        Point(x_lim[1], y_lim[0], z_lim[0]),  # 1
        Point(x_lim[1], y_lim[1], z_lim[0]),  # 2
        Point(x_lim[0], y_lim[1], z_lim[0]),  # 3
        Point(x_lim[0], y_lim[0], z_lim[1]),  # 4
        Point(x_lim[1], y_lim[0], z_lim[1]),  # 5
        Point(x_lim[1], y_lim[1], z_lim[1]),  # 6
        Point(x_lim[0], y_lim[1], z_lim[1]),  # 7
    ]

    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  
        (4, 5), (5, 6), (6, 7), (7, 4),  
        (0, 4), (1, 5), (2, 6), (3, 7),  
    ]

    for start_idx, end_idx in edges:
        marker.points.append(vertices[start_idx])
        marker.points.append(vertices[end_idx])

    return marker

def rotation_matrix_z(theta):
    """
    get pose: torso_in_sacrum, from A_Waist rotation
    """
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    # Construct the rotation matrix
    # R_z = np.array([
    #     [cos_theta, -sin_theta, 0, 0],
    #     [sin_theta, cos_theta, 0, 0],
    #     [0, 0, 1, 0.0112],
    #     [0, 0, 0, 1]
    # ]) # little_green

    R_z = np.array([
        [cos_theta, -sin_theta, 0, 0],
        [sin_theta, cos_theta, 0, 0],
        [0, 0, 1, 0.0694],
        [0, 0, 0, 1]
    ]) # little_red

    return R_z



### zero pose
zero_joint_r = [0.0, 0.017, -0.158, -0.017, -0.105, 0.017, 0.017, 0.017]
zero_tcp_r = [0.024289, -0.31704, -0.16159,-0.0014782, 0.99682, -0.071297, 0.035676]

zero_joint_l = [0.0, 0.017, 0.158, -0.017, -0.105, -0.017, -0.017, 0.017]
zero_tcp_l = [0.026466, 0.31787, -0.16127, 0.01908, 0.99469, 0.087067, 0.05142]

ref_zero_tcp = {'right':[zero_tcp_r], 'left':[zero_tcp_l]}
ref_zero_joint = {'right':[zero_joint_r], 'left':[zero_joint_l]}

### ref_home_tcp
home_tcp_r = []
# home_joint_r = [0.017, -0.147, -0.939, 0.236, -1.402, -0.04, -0.087, -0.0]
home_joint_r = [0.0, -0.217, -0.587, 0.107, -1.408, 0.834, -0.01, -0.010]

home_tcp_l = []
home_joint_l = [0.017, -0.147, 0.939, -0.236, -1.402, -0.04, -0.087, -0.0]

ref_home_tcp = {'right':[home_tcp_r], 'left':[home_tcp_l]}
ref_home_joint = {'right':[home_joint_r], 'left':[home_joint_l]}

### ref_grasp_pose
right_joint_1_1 = [-0.6830626093344763, -1.5485138142760682, -0.7072358751202654, 0.09775176706336722, -0.651210339619156, 1.326721156373057, 0.45535196575718623, 0.6580847715915921]
right_tcp_pose_1_1 = [0.1541, -0.5665, 0.58849, 0.090611, -0.67235, 0.72165, -0.13769]

right_joint_1_2 = [-0.9776717989872674, -1.4232077708017892, -0.709020639491159, 0.0412601101918838, -0.9141016607126542, 1.2968754751682874, 0.3909015921524983, 0.7689391401121449]
right_tcp_pose_1_2 = [-0.014276, -0.55892, 0.58846,0.038042, -0.67422, 0.73315, -0.080474]

right_joint_1_3 = [-1.222787880580686, -1.3619363949262409, -0.7645335892438141, -0.005096310228614593, -0.9585686441918934, 1.2484831248522756, 0.3209515489025772, 0.7647431135206006]
right_tcp_pose_1_3 = [-0.17833, -0.53942, 0.58477, -0.035755, -0.67387, 0.73798, 0.0010494]

right_joint_2_1 = [-0.7589078553733415, -0.8514443870450095, -0.4652547600534182, 0.18723559377279578, -0.8663725814342494, 0.9214921083902063, 0.5561491712928527, 0.10347418385168328]
right_tcp_pose_2_1 = [0.14997, -0.56022, 0.2817,-0.092888, 0.7226, -0.66398, 0.16836]

right_joint_2_2 = [-0.9775639793716371, -0.5798090917786102, -0.5543337999298888, 0.1173619282602053, -1.225327027741331, 0.9475039981777114, 0.5474480250069247, 0.2093743644798537]
right_tcp_pose_2_2 = [-0.028622, -0.55317, 0.27786,-0.020354, 0.72735, -0.67835, 0.10196]

right_joint_2_3 = [-1.2474593449431006, -0.4788920030186134, -0.7076054999681557, 0.006622149390072968, -1.1358687998457444, 0.952974822398933, -0.035394454373690314, 0.12511456286299685]
right_tcp_pose_2_3 =[-0.24903, -0.53397, 0.26814, -0.1045, 0.70496, -0.69843, 0.065636]

# left
left_joint_1_1 = [0.6416535477872474, -1.4995190469725281, 0.7304867666404514, -0.12550040793849784, -0.6970055742418958, -1.349269453218503, 0.42635786517692326, -0.7058731111497593]
left_tcp_pose_1_1 = [0.17319, 0.56374, 0.57694, 0.12914, 0.67467, 0.71662, 0.12085]

left_joint_1_2 = [0.8957420727395453, -1.3445263024710778, 0.7963163136355884, -0.03409821476461538, -0.9496497207853628, -1.3100005464149802, 0.33012524995227976, -0.8080414568226236]
left_tcp_pose_1_2 = [-0.0041721, 0.57542, 0.57616,0.075111, 0.68387, 0.7233, 0.059372]
# hand_pose = [-0.013902, 0.51607, 0.57221,0.72409, -0.060923, -0.077329, 0.68264]

left_joint_1_3 = [1.1330769773976876, -1.279850835898979, 0.8537883909674944, 0.025738790206137816, -1.0112250367949247, -1.2528069830597062, 0.30968790778951305, -0.8178786270717694]
left_tcp_pose_1_3 = [-0.17033, 0.55456, 0.57667,-0.021995, 0.68736, 0.72591, -0.010133]

left_joint_2_1 = [0.6990863061225043, -0.7190936672432436, 0.5180436318690333, -0.17203170094954984, -1.0056372283747184, -1.076698724628445, 0.500704803144202, -0.20911013349196256]
left_tcp_pose_2_1 = [0.15351, 0.55144, 0.26883,0.17756, 0.70886, 0.66753, 0.14285]

left_joint_2_2 = [0.9778502795900216, -0.46486283912291637, 0.5797234260509854, -0.10574325876792279, -1.3677778231582127, -1.0734650189104509, 0.4055894829097487, -0.3109414446748908]
left_tcp_pose_2_2 = [-0.03757, 0.5325, 0.27264,0.12124, 0.72203, 0.67448, 0.095121]

left_joint_2_3 = [1.1906055630852002, -0.42759742341892415, 0.7135271907420478, 0.001999759466345873, -1.2488679835583834, -0.9459987637334895, 0.19688186216356104, -0.23853172723000313]
left_tcp_pose_2_3 = [-0.2247, 0.5357, 0.27341,0.049381, 0.7307, 0.68033, 0.028193]

# joint value of grasp to generate trajectory
ref_grasp_joint = {'right':[right_joint_1_1, right_joint_1_2, right_joint_1_3,
                            right_joint_2_1, right_joint_2_2, right_joint_2_3],
                    'left':[left_joint_1_1, left_joint_1_2, left_joint_1_3,
                            left_joint_2_1, left_joint_2_2, left_joint_2_3]}
# tcp pose value of grasp
ref_grasp_tcp = {'right':[right_tcp_pose_1_1, right_tcp_pose_1_2, right_tcp_pose_1_3,
                          right_tcp_pose_2_1, right_tcp_pose_2_2, right_tcp_pose_2_3],
                'left': [left_tcp_pose_1_1, left_tcp_pose_1_2, left_tcp_pose_1_3,
                         left_tcp_pose_2_1, left_tcp_pose_2_2, left_tcp_pose_2_3]}

### ref_place_tcp
ref_place_joint_r = [-0.19956690978072586, -0.7950135811302401, -0.18146285525559963, 0.39237200457304927, -1.0016610999551865, 0.9603445286113171, 0.0840858598611391, 0.021550100815058263]
ref_place_pose_r =[0.42838, -0.18899, 0.24962,-0.50727, 0.47371, -0.41968, 0.58492]

ref_place_joint_l = [0.20042105827690102, -0.8661775333195911, 0.07983246632825079, -0.44285402680808306, -0.9596965889412946, -1.0242095966452458, 0.06723021192906561, -0.03663881385923748]
ref_place_pose_l = [0.42854, 0.12941, 0.26002,0.54662, 0.42523, 0.39971, 0.60052]

ref_place_tcp = {'right':[ref_place_pose_r], 'left':[ref_place_pose_l]}
ref_place_joint = {'right':[ref_place_joint_r], 'left':[ref_place_joint_l]}

### ref_scene_pose: shelf and table
shelf_pose_right = [0, -0.83, -0.96, 0., 0., 0.707, 0.707]
shelf_pose_left = [0, 0.83, -0.96, 0., 0., 0.707, 0.707]
table_pose = [0.7, 0., -0.96, 0., 0., 0., 1.]
ref_scene_pose = {'right':[shelf_pose_right], 'left':[shelf_pose_left], 'front':[table_pose]}

### ref_QR_pose: left, right, front
QR_pose_right = [0, -0.83, -0.96, 0., 0., 0.707, 0.707]
QR_pose_left = [0, 0.83, -0.96, 0., 0., 0.707, 0.707]
QR_pose_front = [0.7, 0., -0.96, 0., 0., 0., 1.]
ref_scene_pose = {'right':[QR_pose_right], 'left':[QR_pose_left], 'front':[QR_pose_front]}

spacing_z = 0.32
spacing_x = 0.21
ref_bag_pose_right11 = [0.22, -0.8, 0.43, 0., 0., 0., 1.]
ref_bag_pose_right12 = [0.22 - spacing_x, -0.8, 0.43, 0., 0., 0., 1.]
ref_bag_pose_right13 = [0.22 - 2*spacing_x, -0.8, 0.43, 0., 0., 0., 1.]
ref_bag_pose_right21 = [0.22, -0.8, 0.43 - spacing_z, 0., 0., 0., 1.]
ref_bag_pose_right22 = [0.22 - spacing_x, -0.8 , 0.43- spacing_z, 0., 0., 0., 1.]
ref_bag_pose_right23 = [0.22 - 2*spacing_x, -0.8, 0.43- spacing_z, 0., 0., 0., 1.]


ref_bag_pose_left11 = [0.22, 0.8, 0.43, 0., 0., 0., 1.]
ref_bag_pose_left12 = [0.22- spacing_x, 0.8, 0.43, 0., 0., 0., 1.]
ref_bag_pose_left13 = [0.22- 2*spacing_x, 0.8, 0.43, 0., 0., 0., 1.]
ref_bag_pose_left21 = [0.22, 0.8, 0.43- spacing_z, 0., 0., 0., 1.]
ref_bag_pose_left22 = [0.22- spacing_x, 0.8, 0.43- spacing_z, 0., 0., 0., 1.]
ref_bag_pose_left23 = [0.22- 2*spacing_x, 0.8, 0.43- spacing_z, 0., 0., 0., 1.]

ref_bag_pose = {'right':[ref_bag_pose_right11, ref_bag_pose_right12, ref_bag_pose_right13,
                         ref_bag_pose_right21, ref_bag_pose_right22, ref_bag_pose_right23],
                'left':[ref_bag_pose_left11, ref_bag_pose_left12, ref_bag_pose_left13,
                        ref_bag_pose_left21, ref_bag_pose_left22, ref_bag_pose_left23]}

def visualize_pose(matrix_pose_list):
    visualize_poses = []
    # shelf_in_tag_r = [1.065, -0.005, -0.147, 0, 1.57, 0]
    # pose = euler_to_matrix(shelf_in_tag_r)
    # pose_ = matrix_to_ros_pose(pose)
    # visualize_poses.append(pose_)

    # new_euler = [0., 0., 0., 0., 0., 0.]
    # zero_pose= euler_to_matrix(new_euler)
    # new_pose = np.dot(pose, zero_pose)
    for matrix_pose in matrix_pose_list:
        visualize_poses.append(matrix_to_ros_pose(matrix_pose))

    # torso_in_sacrum = rotation_matrix_z(-0.74)
    # visualize_poses.append(matrix_to_ros_pose(torso_in_sacrum))

    # print(pose)
    # print(new_pose)

    pose2rviz = PoseArray()
    pose2rviz.header.frame_id = 'SACRUM'
    pose2rviz.poses = visualize_poses

    return pose2rviz

# file_name = '/home/fengyifei/catkin_ws/src/naviai_manipulation/naviai_manip_beverage_robot_execution/pose_info/sacrum_to_hand_intrinsic.npy'
# file_name = '/home/fengyifei/catkin_ws/src/naviai_manipulation/naviai_manip_beverage_robot_execution/pose_info/sacrum_to_hand_extrinsic.npy'

# camera_info = np.load(file_name, allow_pickle=True)
# print(camera_info)

# torso_in_sacrum_quat = [-0.00037576, 0, 0.0112, 0, 0, 0, 1]
# torso_in_sacrum = quatertion_to_matrix(torso_in_sacrum_quat[:3], torso_in_sacrum_quat[3:])
# print(torso_in_sacrum)
# file_name = '/home/fengyifei/catkin_ws/src/naviai_manipulation/naviai_manip_trajectory_calibration/pose_info/torso_in_sacrum.npy'
# np.save(file_name, torso_in_sacrum)

# file_name = '/home/fengyifei/catkin_ws/src/naviai_manipulation/naviai_manip_trajectory_calibration/pose_info/camera_info.npy'
# camera_info = np.load(file_name, allow_pickle=True)
# print(camera_info)

shelf_in_tag_r = [1.065, -0.005, -0.147, 0, 0, 1.57] # x y z roll pitch yaw

shelf_in_tag_l = []

table_in_tag = []

# visualize_pose()


