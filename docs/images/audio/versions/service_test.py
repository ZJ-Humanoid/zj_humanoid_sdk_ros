#!/usr/bin/env python3
"""
ROS Service Test Script

Service: /zj_humanoid/audio/version
Type: std_srvs/Trigger
Description: 语音模块的版本号

Usage:
    python3 service_test.py

This script calls the service (no parameters needed for Trigger service).
"""

import rospy
import sys
from std_srvs.srv import Trigger, TriggerRequest


def main():
    # Initialize ROS node
    rospy.init_node('service_test_zj_humanoid_audio_version', anonymous=True)
    
    # Wait for service
    rospy.loginfo(f"Waiting for service /zj_humanoid/audio/version...")
    rospy.wait_for_service('/zj_humanoid/audio/version')
    
    try:
        # Create service proxy
        service_proxy = rospy.ServiceProxy('/zj_humanoid/audio/version', Trigger)
        
        # Create empty request (Trigger service has no request parameters)
        req = TriggerRequest()
        
        rospy.loginfo(f"Calling service /zj_humanoid/audio/version...")
        rospy.loginfo(f"Request: {req} (no parameters)")
        
        # Call service
        response = service_proxy(req)
        
        rospy.loginfo(f"Response: {response}")
        print(f"\nService call successful!")
        print(f"Success: {response.success}")
        print(f"Version: {response.message}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

