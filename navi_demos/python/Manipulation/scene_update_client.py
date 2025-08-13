'''
@Description: scene updata client test
@Version: 1.0
'''

import sys
import copy
import rospy
import rospkg
import moveit_commander
import tool_funcs_bag as toolfuncs

from copy import deepcopy
from navi_types.srv import Manip_GetScenePose, Manip_GetScenePoseRequest

class GetScenePoseClient():

    def __init__(self):
        rospack = rospkg.RosPack()
        # self.package_path = rospack.get_path('naviai_manip_retail_service')
        self.package_path = "/navi_ws/src/navi_sdk_ros1/navi_demos/python/Manipulation"
        
        # Moviet API
        moveit_commander.roscpp_initialize(sys.argv)
        self.planning_scene = moveit_commander.PlanningSceneInterface()
        self.get_scene_pose_client = rospy.ServiceProxy('/get_scene_pose', Manip_GetScenePose)

    def call_service(self):
        request = Manip_GetScenePoseRequest()
        request.scene_id = 1 

        response = self.get_scene_pose_client(request)

        print(response.which_scene)
        print(response.success)
        print(response.scene_poses)

        self.create_scene(response.scene_poses, response.success)

    def create_scene(self, scene_poses, success_all):
        self.planning_scene.remove_world_object()
        scene_pose = copy.deepcopy(scene_poses)

        file_path_shelf_l = self.package_path + '/mesh/processed/' + 'shelf_l_lower.stl'
        file_path_shelf_r = self.package_path + '/mesh/processed/' + 'shelf_r_lower.stl'
        file_path_cupboard = self.package_path + '/mesh/processed/' + 'cupboard1.stl'

        table_size = [1, 1, 1]
        table_pose_mat = scene_pose[0]
        table_pose = toolfuncs.ros_pose_to_ros_poseStamped(table_pose_mat)

        shelf_size = [1, 1, 1]
        shelf_pose_mat_r = scene_pose[1]
        shelf_pose_r = toolfuncs.ros_pose_to_ros_poseStamped(shelf_pose_mat_r)

        shelf_pose_mat_l = scene_pose[2]
        shelf_pose_l = toolfuncs.ros_pose_to_ros_poseStamped(shelf_pose_mat_l)

        rospy.loginfo('Loading scene ... ')
        if success_all[0]:
            self.planning_scene.add_mesh('table', table_pose, file_path_cupboard, table_size) # table
        if success_all[1]:
            self.planning_scene.add_mesh('shelf_right', shelf_pose_r, file_path_shelf_r, shelf_size) # right shelf
        if success_all[2]:
            self.planning_scene.add_mesh('shelf_left', shelf_pose_l, file_path_shelf_l, shelf_size) # left shelf

        rospy.loginfo('Loading success ... ')

if __name__ == "__main__":
    rospy.init_node('Get_Scene_Pose_Client', anonymous=True)
    get_scene_pose_client = GetScenePoseClient()
    get_scene_pose_client.call_service()

    rospy.spin()
