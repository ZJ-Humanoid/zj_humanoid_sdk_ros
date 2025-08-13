import time
import rospy
import subprocess
from navi_types.srv import RunCmd, RunCmdRequest

class RunCmdClinet():

    def __init__(self):

        self.run_cmd_client = rospy.ServiceProxy("/run_cmd", RunCmd)

    def call_service(self):

        request = RunCmdRequest()
        request.session_name = 'ros_demos'
        # request.cmd_list = ["docker exec -it naviai_v3 bash && sleep 3",
        #                     "conda activate seg && sleep 3",
        #                     "python /home/catkin_ws/src/naviai_manip_instance_segmentation/scripts/seg_action/seg_action_server.py && sleep 10"]
        
        request.cmd_list = ["rosrun navi_demos nod.py"]
        request.success_flag = "Action finished"
        
        response = self.run_cmd_client(request)

        if response.success:
            rospy.loginfo(f"{request.session_name} start success ... ")
            return True
        else:
            rospy.loginfo(f"{request.session_name} start failed ... ")
            return False

if __name__ == "__main__":
    rospy.init_node('Run_Cmd_Client', anonymous=True)
    run_cmd_client = RunCmdClinet()
    success = run_cmd_client.call_service()

