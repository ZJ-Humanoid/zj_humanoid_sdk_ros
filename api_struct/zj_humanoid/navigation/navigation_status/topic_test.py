#!/usr/bin/env python3
"""
ROS Topic Test Script

Topic: /zj_humanoid/navigation/navigation_status
Type: navigation/NavigationStatus
Description: 当前导航状态

Usage:
    python3 topic_test.py <data.yaml>

This script publishes the message data from a YAML file to the topic at 10Hz.
Press Ctrl+C to stop.
"""

import rospy
import yaml
import sys
from navigation.msg import NavigationStatus


def load_yaml_data(yaml_file):
    """Load message data from YAML file."""
    try:
        with open(yaml_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception as e:
        rospy.logerr(f"Failed to load YAML file: {e}")
        sys.exit(1)


def dict_to_msg(data, msg_class):
    """Convert dictionary to ROS message."""
    msg = msg_class()
    for key, value in data.items():
        if hasattr(msg, key):
            setattr(msg, key, value)
    return msg


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <data.yaml>")
        sys.exit(1)
    
    yaml_file = sys.argv[1]
    
    # Initialize ROS node
    rospy.init_node('topic_test_zj_humanoid_navigation_navigation_status', anonymous=True)
    
    # Load data from YAML
    data = load_yaml_data(yaml_file)
    
    # Create publisher
    pub = rospy.Publisher('/zj_humanoid/navigation/navigation_status', NavigationStatus, queue_size=10)
    
    # Create message
    msg = dict_to_msg(data, NavigationStatus)
    
    # Publish at 10Hz
    rate = rospy.Rate(10)
    
    rospy.loginfo(f"Publishing to /zj_humanoid/navigation/navigation_status at 10Hz...")
    rospy.loginfo(f"Message type: navigation/NavigationStatus")
    rospy.loginfo(f"Data: {msg}")
    
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
