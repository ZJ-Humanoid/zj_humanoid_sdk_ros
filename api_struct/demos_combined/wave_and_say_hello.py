#!/usr/bin/env python3
"""
ROS Service Test Script

Service: /zj_humanoid/upperlimb/movej_by_path/right_arm
Type: upperlimb/MoveJByPath
Description: 右臂轨迹movej

Usage:
    python3 wave_and_say_hello.py wave_and_say_hello.yaml

This script calls the service with request data from a YAML file.
"""

import rospy
import yaml
import sys
import threading
from upperlimb.srv import MoveJByPath, MoveJByPathRequest
from upperlimb.msg import Joints
from audio.srv import TTS, TTSRequest
from std_srvs.srv import Trigger, TriggerRequest


def load_yaml_data(yaml_file):
    """Load service request data from YAML file."""
    try:
        with open(yaml_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception as e:
        rospy.logerr(f"Failed to load YAML file: {e}")
        sys.exit(1)

def call_tts_service(tts_text_list, is_play=True, delay=3.0):
    """
    Calls the /zj_humanoid/audio/tts_service.
    
    Args:
        tts_text_list (list): List of strings to be synthesized and played.
        is_play (bool): Whether to play the synthesized audio.
    """
    SERVICE_NAME = '/zj_humanoid/audio/tts_service'
    
    rospy.loginfo(f"Waiting for TTS service: {SERVICE_NAME}...")
    try:
        rospy.wait_for_service(SERVICE_NAME, timeout=10) 
    except rospy.ROSException as e:
        rospy.logerr(f"TTS service {SERVICE_NAME} not available after timeout: {e}")
        return False, "Service not available"

    try:
        tts_proxy = rospy.ServiceProxy(SERVICE_NAME, TTS)
        
        req = TTSRequest()
        req.text = tts_text_list
        req.isPlay = is_play
        
        rospy.loginfo(f"Calling TTS service with text: {tts_text_list}")
        
        rospy.sleep(delay)
        response = tts_proxy(req)
        
        if response.success:
            rospy.loginfo(f"TTS service call successful. Message: {response.message}")
        else:
            rospy.logwarn(f"TTS service failed (Status: {response.status}). Message: {response.message}")

        return response.success, response.message
        
    except rospy.ServiceException as e:
        rospy.logerr(f"TTS Service call failed: {e}")
        return False, str(e)
    

def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <request_data.yaml>")
        sys.exit(1)
    
    yaml_file = sys.argv[1]
    
    rospy.init_node('wave_and_say_hello_demo', anonymous=True)
    
    data = load_yaml_data(yaml_file)
    if 'path' not in data or 'time' not in data or 'arm_type' not in data:
            rospy.logerr("YAML data is missing 'path', 'time', or 'arm_type' keys.")
            sys.exit(1)

    # Wait for service
    ARM_SERVICE_NAME = '/zj_humanoid/upperlimb/movej_by_path/right_arm'
    GO_DOWN_SERVICE_NAME = '/zj_humanoid/upperlimb/go_down/right_arm'
    rospy.loginfo(f"Waiting for service ...")
    rospy.wait_for_service(ARM_SERVICE_NAME)
    rospy.wait_for_service(GO_DOWN_SERVICE_NAME)
    
    try:
        service_proxy = rospy.ServiceProxy(ARM_SERVICE_NAME, MoveJByPath)
        go_down_proxy = rospy.ServiceProxy(GO_DOWN_SERVICE_NAME, Trigger)

        req = MoveJByPathRequest()
        
        joint_msgs = []
        for joint_data in data['path']:
            if 'joint' in joint_data and isinstance(joint_data['joint'], list):
                joint_msg = Joints()
                joint_msg.joint = [float(j) for j in joint_data['joint']] 
                joint_msgs.append(joint_msg)
            else:
                rospy.logwarn(f"Skipping invalid joint data: {joint_data}")
        
        req.path = joint_msgs
        req.time = float(data['time'])
        req.is_async = bool(data['is_async'])
        req.arm_type = int(data['arm_type'])

        rospy.loginfo(f"Calling service {ARM_SERVICE_NAME}...")
        rospy.loginfo(f"Request (time: {req.time}, arm_type: {req.arm_type}, path_points: {len(req.path)})")

        tts_text = ["你好", "我可以为你做些什么？"]
        tts_thread = threading.Thread(target=call_tts_service, args=(tts_text,), daemon=True)
        tts_thread.start()

        response = service_proxy(req)

        rospy.loginfo(f"Response: {response}")
        print(f"\nService call successful!")
        print(f"Response: {response}")

        tts_thread.join(timeout=3.0)

        rospy.loginfo(f"Preparing to call {GO_DOWN_SERVICE_NAME} service...")
        
        go_down_req = TriggerRequest()
        go_down_response = go_down_proxy(go_down_req)
        
        if go_down_response.success:
            rospy.loginfo(f"{GO_DOWN_SERVICE_NAME} service successful. Message: {go_down_response.message}")
        else:
            rospy.logerr(f"{GO_DOWN_SERVICE_NAME} service failed. Message: {go_down_response.message}")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass