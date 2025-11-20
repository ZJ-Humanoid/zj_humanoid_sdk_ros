#!/usr/bin/env python3
"""
ROS Service Test Script

Service: /zj_humanoid/upperlimb/movej_by_path/neck
Type: upperlimb/MovejByPath
Description: 颈部关节路径运动

Usage:
    python3 service_test.py <request_data.yaml>

This script calls the service with request data from a YAML file.
"""

import rospy
import yaml
import sys
from upperlimb.srv import MovejByPath, MovejByPathRequest


def load_yaml_data(yaml_file):
    """Load service request data from YAML file."""
    try:
        with open(yaml_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception as e:
        rospy.logerr(f"Failed to load YAML file: {e}")
        sys.exit(1)


def dict_to_request(data, request_class):
    """Convert dictionary to service request."""
    req = request_class()
    if data:
        for key, value in data.items():
            if hasattr(req, key):
                setattr(req, key, value)
    return req


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <request_data.yaml>")
        sys.exit(1)
    
    yaml_file = sys.argv[1]
    
    # Initialize ROS node
    rospy.init_node('service_test_zj_humanoid_upperlimb_movej_by_path_neck', anonymous=True)
    
    # Load request data from YAML
    data = load_yaml_data(yaml_file)
    
    # Wait for service
    service_name = '/zj_humanoid/upperlimb/movej_by_path/neck'
    rospy.loginfo(f"Waiting for service {service_name}...")
    rospy.wait_for_service(service_name)
    
    try:
        # Create service proxy
        service_proxy = rospy.ServiceProxy(service_name, MovejByPath)
        
        # Create request
        req = dict_to_request(data, MovejByPathRequest)
        
        rospy.loginfo(f"Calling service {service_name}...")
        rospy.loginfo(f"Request: {req}")
        
        # Call service
        response = service_proxy(req)
        
        rospy.loginfo(f"Response: {response}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
