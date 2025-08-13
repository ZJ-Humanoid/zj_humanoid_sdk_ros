import time
import rospy
import subprocess
from navi_types.srv import RunCmd, RunCmdResponse


class RunCmdService():

    def __init__(self):

        self.run_cmd_service = rospy.Service('/run_cmd', RunCmd, self.run_cmd_callback)

    def run_cmd_callback(self, req):
        session_name = req.session_name
        cmd_list = req.cmd_list
        success_flag = req.success_flag

        response = RunCmdResponse()
        if cmd_list != []:
            subprocess.run(["tmux", "new-session", "-d", "-s", session_name])
            rospy.loginfo(f'Success to create {session_name}')
            for cmd in cmd_list:
                subprocess.run(["tmux", "send-keys", "-t", session_name, cmd, "C-m"])
                time.sleep(10)
            if self.check_phrase_in_output(session_name, success_flag):
                response.success = True
                rospy.loginfo("success to run cmd ... ")
            else:
                response.success = False
                rospy.logwarn("fail to run cmd ! ")
        else:
            response.success = False

        return response

    def get_tmux_output(self, session):
        result = subprocess.run(["tmux", "capture-pane", "-p", "-t", session], capture_output=True, text=True)
        return result.stdout
    
    def check_phrase_in_output(self, session, phrase):
        output = self.get_tmux_output(session)
        return phrase in output

if __name__ == "__main__":
    rospy.init_node('Run_Cmd_Service', anonymous=True)
    run_cmd_service = RunCmdService()
    rospy.loginfo("run cmd service start ... ")

    rospy.spin()


# session_name = "seg_action_session"
# # tmux kill-session -t seg_action_session && sudo kill -9 $(pgrep -f seg_action_server)  # kill server

# # 启动 tmux 会话（如果会话已存在，则不创建新的）
# subprocess.run(["tmux", "new-session", "-d", "-s", session_name])

# subprocess.run(["tmux", "send-keys", "-t", session_name, "docker exec -it naviai_v3 bash", "C-m"])
# time.sleep(2)

# subprocess.run(["tmux", "send-keys", "-t", session_name, "conda activate seg", "C-m"])
# time.sleep(2)

# subprocess.run(["tmux", "send-keys", "-t", session_name, "python /home/catkin_ws/src/naviai_manip_instance_segmentation/scripts/seg_action/seg_action_server.py", "C-m"])
# time.sleep(10)

# def get_tmux_output(session):
#     """ 获取 tmux 会话的最新输出 """
#     result = subprocess.run(["tmux", "capture-pane", "-p", "-t", session], capture_output=True, text=True)
#     return result.stdout

# def check_phrase_in_output(session, phrase):
#     """ 检查 tmux 输出是否包含指定的句子 """
#     output = get_tmux_output(session)
#     return phrase in output

# request_success_flag = "instance service start"
# if check_phrase_in_output(session_name, request_success_flag):
#     success = True
# else:
#     success = False

# show terminal
# subprocess.run(["tmux", "attach-session", "-t", session_name])

